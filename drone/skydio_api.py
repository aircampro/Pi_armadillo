#!/usr/bin/env python3
# example of checking information from skydio drone using the rest api
# ref:- https://apidocs.skydio.com/reference/introduction
#
import os
import csv
import argparse
from datetime import datetime
import requests

# ----------------- CONFIGURATION ------------------
# Get API token from environment variable
API_TOKEN = os.getenv("API_TOKEN")                                                                # Must be set before running the script
API_BASE = "https://api.skydio.com/api"                                                           # base of the skydio api
VERSION=0                                                                                         # 0 or 1

# ----------------- HELPER FUNCTIONS ------------------
def get_headers() -> dict:
    """Get headers for API requests."""
    if not API_TOKEN:
        raise EnvironmentError("API_TOKEN environment variable is not set.")
    return {"Accept": "application/json", "Authorization": f"ApiToken {API_TOKEN}"}

def get_all_batteries():
    """Fetch all batteries from the Skydio API"""
    r = requests.get(f"{API_BASE}/v{VERSION}/batteries", headers=get_headers())
    response = r.json()
    if response.get("status_code") != 200:
        print(
            f"Error fetching batteries: {response.get('error_message', 'Unknown error')}"
        )
        return []
    return response.get("data", {}).get("batteries", [])

def get_flight_info():
    """Fetch all flight info from the Skydio API"""
    r = requests.get(f"{API_BASE}/v{VERSION}/flights", params={"vehicle_serial": "Skydio2-1234", "takeoff_since": ""})
    response = r.json()
    if response.get("status_code") != 200:
        print(
            f"Error fetching flight info: {response.get('error_message', 'Unknown error')}"
        )
        return []
    return response

def get_tele(fid):
    """Fetch all telemetry info from the Skydio API"""
    r = requests.get(f"{API_BASE}/v{VERSION}/flight/{fid}/telemetry")
    response = r.json()
    if response.get("status_code") != 200:
        print(
            f"Error fetching telemetry: {response.get('error_message', 'Unknown error')}"
        )
        return []
    return response

def mission():
    """Perform mission as programmed via the Skydio API"""
    body = {
        "name": "My GPS Mission",
        "waypoints": [
            {
                "name": "Photo of HQ",
                "action": "PHOTO",
                "orientation": {
                    "gimbal_pitch_degrees": 20,
                    "heading_degrees": 60,
                },
                "position": {
                    "frame": "GPS",
                    "latitude": 37.534329,
                    "longitude": -122.331413,
                    # Since GPS altitude tends to be unreliable, you
                    # can also specify a z value in feet above the
                    # takeoff location, by setting z_frame to NAV
                    "z": 3,
                    "z_frame": "NAV"
                },
            }
        ],
    }
    r = requests.post(f"{API_BASE}/v{VERSION}/mission/template", data=body)
    return r

def format_flight_time(seconds):
    """Convert flight time from seconds to hours:minutes:seconds format"""
    hours = seconds // 3600
    minutes = (seconds % 3600) // 60
    secs = seconds % 60
    return f"{hours}h {minutes}m {secs}s"

def get_battery_health_status(cycles, min_voltage, max_temp):
    """Determine battery health status based on key metrics"""
    if cycles > 500 or min_voltage < 2.5 or max_temp > 65:
        return "Critical"
    elif cycles > 300 or min_voltage < 2.8 or max_temp > 60:
        return "Warning"
    elif cycles > 150 or min_voltage < 3.0 or max_temp > 55:
        return "Monitor"
    else:
        return "Good"

def display_battery_info(csv_file=None):
    """
    Display formatted battery information and optionally save to CSV

    Args:
        csv_file (str, optional): Path to CSV file where results will be saved
    """
    print("\n🔋 BATTERY FLEET REPORT 🔋")
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"Generated on: {timestamp}\n")

    batteries = get_all_batteries()

    if not batteries:
        print("No battery data available.")
        return

    # Prepare data for tabular display
    print(
        "{:<20} {:<20} {:<10} {:<10} {:<15} {:<10} {:<15} {:<10}".format(
            "Serial",
            "Name",
            "Cycles",
            "Flights",
            "Min Voltage",
            "Max Temp",
            "Flight Time",
            "Health",
        )
    )
    print("-" * 110)

    # Sort by health status priority
    def sort_key(battery):
        cycles = battery.get("cycles", 0)
        min_voltage = battery.get("min_voltage", 0)
        max_temp = battery.get("max_cell_temp", 0)
        status = get_battery_health_status(cycles, min_voltage, max_temp)
        priority = {"Critical": 0, "Warning": 1, "Monitor": 2, "Good": 3}
        return priority.get(status, 4)

    batteries.sort(key=sort_key)

    # Track health counts for summary
    health_counts = {"Critical": 0, "Warning": 0, "Monitor": 0, "Good": 0}

    for bat in batteries:
        serial = bat.get("battery_serial", "Unknown")
        name = bat.get("battery_name", "Unnamed")
        cycles = bat.get("cycles", 0)
        flight_count = bat.get("flight_count", 0)
        min_voltage = bat.get("min_voltage", 0)
        max_temp = bat.get("max_cell_temp", 0)
        total_flight_time = format_flight_time(bat.get("total_flight_time", 0))
        health_status = get_battery_health_status(cycles, min_voltage, max_temp)

        # Update health counts
        health_counts[health_status] += 1

        print(
            "{:<20} {:<20} {:<10} {:<10} {:<15.2f} {:<10} {:<15} {:<10}".format(
                serial,
                name,
                cycles,
                flight_count,
                min_voltage,
                f"{max_temp}°C",
                total_flight_time,
                health_status,
            )
        )

    # Summary statistics
    print("\n📊 SUMMARY:")
    print(f"Total Batteries: {len(batteries)}")
    print(
        f"Health Distribution: {health_counts['Good']} Good | {health_counts['Monitor']} Monitor | "
        f"{health_counts['Warning']} Warning | {health_counts['Critical']} Critical"
    )

    # Find battery with most cycles
    if batteries:
        most_used = max(batteries, key=lambda x: x.get("cycles", 0))
        print(
            f"Most Used Battery: {most_used.get('battery_name', 'Unnamed')} "
            f"({most_used.get('battery_serial')}) - {most_used.get('cycles', 0)} cycles"
        )

    # Export to CSV if requested
    if csv_file and batteries:
        export_to_csv(batteries, csv_file)

def export_to_csv(batteries, csv_file):
    """
    Export battery data to a CSV file

    Args:
        batteries (list): List of battery dictionaries
        csv_file (str): Path to CSV file
        health_counts (dict): Dictionary with counts of batteries by health status
        timestamp (str): Timestamp when the report was generated
    """
    try:
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(os.path.abspath(csv_file)), exist_ok=True)

        # Write battery data to CSV
        with open(csv_file, "w", newline="") as f:
            writer = csv.writer(f)

            # Write column headers only
            writer.writerow(
                [
                    "Serial",
                    "Name",
                    "Cycles",
                    "Flights",
                    "Min Voltage",
                    "Max Temp",
                    "Flight Time",
                    "Health",
                ]
            )

            # Write battery data rows
            for bat in batteries:
                serial = bat.get("battery_serial", "Unknown")
                name = bat.get("battery_name", "Unnamed")
                cycles = bat.get("cycles", 0)
                flight_count = bat.get("flight_count", 0)
                min_voltage = bat.get("min_voltage", 0)
                max_temp = bat.get("max_cell_temp", 0)
                total_flight_time = format_flight_time(bat.get("total_flight_time", 0))
                health_status = get_battery_health_status(cycles, min_voltage, max_temp)

                writer.writerow(
                    [
                        serial,
                        name,
                        cycles,
                        flight_count,
                        f"{min_voltage:.2f}",
                        f"{max_temp}°C",
                        total_flight_time,
                        health_status,
                    ]
                )

        print(f"\nCSV report saved to: {csv_file}")
    except Exception as e:
        print(f"Error saving CSV file: {e}")


# ----------------- MAIN SCRIPT ------------------
if __name__ == "__main__":
    global API_TOKEN
    parser = argparse.ArgumentParser(
        description="Generate a report of Skydio battery fleet"
    )
    parser.add_argument(
        "--csv_file", "-c", help="Path to CSV file where results will be saved",
        "--api_token", "-a", help="Skydio API token",
        "--flight_id", "-t", help="telemetry"
    )
    args = parser.parse_args()

    print("Fetching battery data from Skydio Cloud API...")
    try:
        if not API_TOKEN:
            API_TOKEN = args.api_token
        m = mission()
        print(m)
        display_battery_info(args.csv_file)
        f = get_flight_info()
        print(f)
        t = get_tele(args.flight_id)
        print(t)
    except EnvironmentError as e:
        print(f"Error: {e}")
        print(
            "Please set the API_TOKEN environment variable before running this script."
        )
    except Exception as e:
        print(f"Error: {e}")
        print("Tip: Make sure you have a valid API token and network connection.")

