# AFDX ARINC 664 - Master scheduler for data - adapted for secure iOt use
#
# SYNC(device) -> asks device for data [ polled ]
# tells master of cipher <- ACK from sepcified device
# <- DATA is the device data message
#
# ref :- https://github.com/rifatsekerariot/ariot
#
import time
import socket
import json
import signal
# slot table shows devices and priority for scheduler
slot_table = {
    0: {"device_id": "TMP_PROBE_01", "priority": "B", "duration_ms": 10},
    1: {"device_id": "WIND_SPEED_01", "priority": "C", "duration_ms": 10},
    2: {"device_id": "TMP_PROBE_02", "priority": "B", "duration_ms": 10},
    3: {"device_id": "WIND_SPEED_02", "priority": "C", "duration_ms": 10}
}

SYNC_INTERVAL = 1.0
SLOT_MS = 10
UDP_PORT = 5005
BROADCAST_IP = "255.255.255.255"

RUN_F = True
def sig_handler(s):
    global RUN_F
    RUN_F = False
    
def send_sync(dev):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        message = json.dumps({
            "type": "SYNC",
            "timestamp": time.time(),
            "slots": slot_table,
            "device": dev
        }).encode()
        sock.sendto(message, (BROADCAST_IP, UDP_PORT))                        # braodcasts to all we could change to sepcific ips
        print(f"[SYNC] Broadcast sent @ {time.time():.3f}")

def allow_slot_transmission(slot_id):
    slot = slot_table.get(slot_id)
    if slot:
        print(f"[SLOT] Allow TX: Slot {slot_id} | Device: {slot['device_id']} | Priority: {slot['priority']}")
        send_sync(slot['device_id'])                                          # ask this device to send me data

def slot_loop():
    global RUN_F
    signal.signal(signal.SIGUSR1, sig_handler)
    signal.signal(signal.SIGUSR2, sig_handler)
    while RUN_F == True:
        start_time = time.time()
        # send_sync() - do this if you want to change to sync the time with all units
        for slot_id in sorted(slot_table.keys()):
            slot_start = start_time + (slot_id * SLOT_MS / 1000.0)
            time.sleep(max(0, slot_start - time.time()))
            allow_slot_transmission(slot_id)
        elapsed = time.time() - start_time
        if elapsed < SYNC_INTERVAL:
            time.sleep(SYNC_INTERVAL - elapsed)

if __name__ == "__main__":
    print("[GATEWAY] AFDX-lite-IoT slot scheduler active")
    slot_loop()