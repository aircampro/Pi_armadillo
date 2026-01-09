#!/usr/bin/python
#
# Example of using iridium modem 9602 satelite connection https://www.iridium.com/products/iridium-9602-module 
# adapted for ubuntu library is here https://github.com/justengel/pyiridium9602?tab=readme-ov-file
#

import pyiridium9602

# Create your own serial port object and give it as the first argument or just give it the port name.
MY_CONN_PORT="/dev/ttyUSB0"
iridium_port = pyiridium9602.IridiumCommunicator(MY_CONN_PORT)

# Message parser
def parse_data(data):
    print("data recived :", data)

def message_failed(msg_len, content, checksum, calc_check):
    print("Message Failed checksum or length!", msg_len, content, checksum, calc_check)

# Use the default signal class and override the Signal API methods or create your own object.
iridium_port.signal.connected = lambda: print("Connected!")
iridium_port.signal.disconnected = lambda: print("Disconnected!")
iridium_port.signal.serial_number_updated = lambda s: print("Serial Number:", s)
iridium_port.signal.system_time_updated = lambda s: print("System Time:", s)
iridium_port.signal.signal_quality_updated = lambda sig: print("Signal Quality (0-5):", sig)
iridium_port.signal.check_ring_updated = lambda tri, sri: print("Telephone Indicator:", tri, 
                                                                "\nSBD Indicator:", sri)
iridium_port.signal.message_received = parse_data
iridium_port.signal.message_receive_failed = message_failed
iridium_port.signal.notification = print

# NOTE: There is no thread in this example, so `connect()` creates a thread to Complete the connection process
iridium_port.connect() # Raises IridiumError if the port cannot be opened or if the ping did not find a response.

# Blocking methods example
sys_time = iridium_port.acquire_system_time()
print("System Time:", sys_time)

# Non blocking command requests
sq = iridium_port.request_signal_quality()
st = iridium_port.queue_system_time()
print(f"Quality {sq} System Time:{st}")
serial_number = iridium_port.acquire_serial_number()
print("Serial Number:", serial_number)

# If you run a request immediately after a request then the response will error
# This is because the first command will have it's value returned while the new request is the expected command
#iridium_port.request_serial_number()

# Blocking command (wait for previous command and wait to complete)
with iridium_port.wait_for_command():
    sq = iridium_port.request_signal_quality()
    print(f"Quality {sq}")

# Blocking command (wait for previous command and wait to complete)
with iridium_port.wait_for_command():
    iridium_port.check_ring()  # If an SBD ring is found automatically start the session to read the value.

# Blocking Command (Do not wait for previous `wait_for_previous=0`)
serial_number = iridium_port.acquire_response(pyiridium9602.Command.SERIAL_NUMBER, wait_for_previous=0)
print("Manual Serial Number:", serial_number)

# Pre-made Blocking Command
sig = iridium_port.acquire_signal_quality()
print("Manual Signal Quality (0 - 5):", sig)

# Stop the `iridium_port.listen_thread` and close the port
iridium_port.close()	