#!/usr/bin/env python3
#
# Example of a route planner using Omlet
# adapted from ref:- https://routing.oaasis.cc/developers/doc/notebooks/cvrp.html
#
import random
import numpy as np
import os
import requests                                                                                                  # communicate with omlet via rest api
from mavsdk import System                                                                                        # to connect to a drone
import time
import threading                                                                                                 # thread to each drone
import ctypes
import matplotlib.pyplot as plt
from matplotlib import colormaps
import sys

# load .env file with API key. Alternatively, set the API key
# as OAASIS_API_KEY="..."
from dotenv import load_dotenv; load_dotenv() 
json_data = {}

# set the number of vehicles and locations to visit
num_visits = 10
num_vehicles = 4
locations=[[],[],[],[]]                                                                                        # make a array for each drones locations
# set the co-ordinates of what you want to vehicles to go to
visit_lat = [ 126.87538421608637, 126.7586682597194, 126.83670341665179, 126.99282261113129, 126.89680400453523, 127.08615384830541, 127.2285390779482, 127.0925898631831, 127.08178900165579, 127.16017127623809 ]
visit_lon = [ 37.5027991938211, 37.480904634677906, 37.68127434786349, 37.54914499041778, 37.418029234410476, 37.579985745533484, 37.44897092010246, 37.52467063904458, 37.54052150428011, 37.467395158985305 ]
# To fly drone 20m above the ground plane
flying_alt = absolute_altitude + 20.0
loc_db = 0.05                                                                                                  # location deadband
BAT_ENOUGH=30                                                                                                  # battery threshold for journey

# Class wrapper for threading 
# you can also kill task e.g. if op_task.is_alive(): op_task.raise_exception()
#
class twe(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs={}):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        return

    def run(self):
        self._target(*self.args, **self.kwargs)

    def get_id(self):
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id

    def raise_exception(self):
        thread_id = self.get_id()
        resu = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), ctypes.py_object(SystemExit))
        if resu > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(thread_id), 0)
            print('Failure in raising exception')

# move a drone to a co-ordinate
def move_drone(i, d, lock):
    while len(locations[i]) > 1:                                                                                       # we have at least one co-ord to goto
        with lock:
            d.action.goto_location(locations[i][0], locations[i][1], flying_alt, 0)
        print(f"drone {i+1} moving to {locations[i][0]}, {locations[i][1]}")
        not_there = True
        while not_there == True:
            for gps_info in d.telemetry.gps_info():                                                                     # get current location
                print(f"GPS info: {gps_info}")
                with lock:
                    if abs(gps_info[0] - locations[i][0]) < loc_db and abs(gps_info[1] - locations[i][1]) < loc_db:     # check if we have reached destination
                        not_there = False
        with lock:
            locations[i] = locations[i][2:]                                                                             # pop the first two items of movement as they have now completed
        d.gripper.release(instance=1)                                                                                   # release the item from the gripper
    d.action.land()                                                                                                     # land drone to ground after each location visited

t_lock = threading.Lock()                                                                                        # create a thread lock between each drone movement thread & the location array
depot = {                                                                                                        # Set Depot Location
    "name": "depot_1", 
    "index": 0, 
    "coordinate": {
        "lng": 126.734,                                                                                          # Longitude range of Seoul
        "lat": 37.715,                                                                                           # Latitude range of Seoul
    },
}
json_data["depot"] = depot

# Visits
visits = [
    {
        "name": f"visit_{visit_idx + 1}",
        "index": visit_idx + 1,
        "coordinate": {
            "lng": visit_lon[visit_idx],                                                                         # Longitude range of Seoul
            "lat": visit_lat[visit_idx],                                                                         # Latitude range of Seoul
        },
        "volume": 10,
    }
    for visit_idx in range(num_visits)
]
json_data["visits"] = visits

vehicles = [
    {
        "name": f"vehicle_{vehicle_idx + 1}",
        "volume_capacity": 77,
        "vehicle_type": "drone",
    }
    for vehicle_idx in range(num_vehicles)
]
json_data["vehicles"] = vehicles

# set solver config
json_data["option"] = {
    "objective_type": "minsum",
    "timelimit": 3,
    "distance_type": "euclidean"
}

# solve the problem
OAASIS_API_KEY = os.getenv("OAASIS_API_KEY", None)                                                 # read api key from environment
assert OAASIS_API_KEY is not None, "Please provide an API key!"
headers = {"X-API-KEY": OAASIS_API_KEY, "Accept": "application/vnd.omelet.v2+json"}
product_url = "https://routing.oaasis.cc/api/vrp"

drone1 = System()                                                                                  # connect to the transport
drone1_ip="10.0.1.3"
drone1.connect(system_address=f"udpin://{drone1_ip}:14540")
drone2 = System()
drone2_ip="10.0.1.4"
drone2.connect(system_address=f"udpin://{drone2_ip}:14540")
drone3 = System()
drone3_ip="10.0.1.7"
drone3.connect(system_address=f"udpin://{drone3_ip}:14540")
drone3 = System()
drone4_ip="10.0.1.6"
drone4.connect(system_address=f"udpin://{drone4_ip}:14540")
print("Waiting for drone to connect...")
drones = [ drone1, drone2, drone3, drone4 ]
for i, d in enumerate(drones):                                                                     # start each drone ready to go
    for state in d.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone {i}!")
            break

    for health in d.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"-- Global position state is good enough for flying. drone {i}")
            break

    for battery in d.telemetry.battery():
        print(f"Battery drone {i}: {battery.remaining_percent}")
        if battery.remaining_percent < BAT_ENOUGH:
            print(f"drone {i} battery too low remove and re-start plan or fit new battery")
            sys.exit(-1)

    print("Fetching amsl altitude at home location....")
    for terrain_info in d.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    print(f"-- Arming drone {i}")
    d.action.arm()

    print(f"-- drone {i} Taking off")
    d.action.takeoff()
    time.sleep(1)

response = requests.post(product_url, json=json_data, headers=headers)                              # request plan for data from the omlet server

if response.status_code == 200:
    print("Omlet Server Processed request succesfully!")
    print("Response:")
    j = response.json()
    print(j)                                                                                         # complete response
    for i in range(0, num_vehicles):                                                                 # print the route for each vehicle_
        z=j['routing_engine_result']['routes'][i]                                                    # read each vehicles route
        print("-----------------------------")
        print(f"{z['vehicle_name']} travels to route locations {z['route_index']} {z['route_name']}")  
        idx = 0        
        for ech in z['route_index']:
            if ech != 0:
                print(f"lat={visit_lat[ech - 1]}  lon={visit_lon[ech - 1]} ")  
                locations[i].append(visit_lat[ech - 1])                                             # fill the locations list
                locations[i].append(visit_lon[ech - 1])                
            else:
                print(f"lat={depot['coordinate']['lat']}  lon={depot['coordinate']['lng']} ") 
                if idx == 0:                                                                        # skip co-ord as we are already there
                    idx += 1
                else:
                    locations[i].append(depot['coordinate']['lat'])                                 # now add return to the depot to the journey
                    locations[i].append(depot['coordinate']['lng'])            
        print("-----------------------------")                
else:
    print("The request failed. Status code:", response.status_code)
    print(response.json())

# now command the drones to the co-ordinates, create a task list then start each one
threadlist = list()
for i, d in enumerate(drones):
    task = twe(name = f'Thread Drone Movements {i}', target=move_drone, args=(i, d, t_lock), kwargs={})
    threadlist.append(task)

for thread in threadlist:
    thread.start()

# and wait for all journey completion
for thread in threadlist:
    thread.join()

# visualize
resp = response.json()["routing_engine_result"]
fig, ax = plt.subplots(1, figsize=(4, 4))
coordinates = [[depot["coordinate"]["lat"], depot["coordinate"]["lng"]]]
coordinates.extend([
    [visit["coordinate"]["lat"], visit["coordinate"]["lng"]] for visit in visits
])
# Scatter the coordinates of the depot and visits
ax.scatter(coordinates[0][0], coordinates[0][1], color="tab:red", marker="x")
for i in range(1, len(coordinates)):
    ax.scatter(coordinates[i][0], coordinates[i][1], color="tab:blue")
# Plot the route
num_routes = len(resp["routes"])
colors = colormaps.get_cmap("tab10")
# colors = cm.get_cmap("tab10", num_routes)
for route_idx, route_obj in enumerate(resp["routes"]):
    routes = route_obj["route_index"]
    route_color = colors(route_idx)
    for step_idx in range(len(routes) - 1):
        src_idx = routes[step_idx]
        dst_idx = routes[step_idx + 1]
        src_x, src_y = coordinates[src_idx]
        dst_x, dst_y = coordinates[dst_idx]
        ax.annotate(
            "",
            xy=(dst_x, dst_y),
            xytext=(src_x, src_y),
            arrowprops=dict(
                arrowstyle="-|>", 
                color=route_color,
                lw=1.2,
            ),
            size=15,
            annotation_clip=False,
        )
# Label and title
ax.set_xlabel('Latitude')
ax.set_ylabel('Longitude')
ax.set_title(f'Cost = {resp["solution_cost_details"]["total_objective_cost"]}')
# Grid
ax.grid(axis='both', color='black', alpha=0.1)
plt.tight_layout()
