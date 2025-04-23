#!/usr/bin/env python3

import socket
import json
import time
from pymavlink import mavutil

# MAVLink connection
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print(f"Connected to drone {connection.target_system}")

# Drone ID
SysID = 1

# UDP broadcast setup
UDP_IP = '<broadcast>'
UDP_PORT = 14550

broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

sample_interval = 0.5

try:
    while True:
        gps_msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

        if gps_msg:
            data_entry = {
                'timestamp': time.time(),
                'SysID': SysID,
                'lat': gps_msg.lat / 1e7,
                'lon': gps_msg.lon / 1e7,
                'alt': gps_msg.alt / 1000.0
            }

            broadcast_message = json.dumps(data_entry).encode('utf-8')
            broadcast_sock.sendto(broadcast_message, ('<broadcast>', 14550))
            print(f"Broadcasting: {data_entry}")

        time.sleep(sample_interval)

except KeyboardInterrupt:
    print("\nExiting Drone 1 broadcaster.")
