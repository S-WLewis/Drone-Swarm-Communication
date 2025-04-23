#!/usr/bin/env python3

import socket
import threading
import json
import time
import math
from pymavlink import mavutil

# MAVLink connection
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print(f"Connected to system {connection.target_system}")

# IDs
SysID = 2  # This drone
Target_SysID = 1  # Drone to follow

# Global positions
other_lat, other_lon = None, None

# Listening socket setup
listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
listen_sock.bind(('', 14550))

# Listener thread function
def listener():
    global other_lat, other_lon
    other_lat, other_lon = None, None
    while True:
        data, _ = listen_sock.recvfrom(1024)
        message = json.loads(data.decode('utf-8'))
        if message['SysID'] == Target_SysID:
            other_lat, other_lon = message['lat'], message['lon']

# Start listener thread
listener_thread = threading.Thread(target=listener, daemon=True)
listener_thread.start()

# Calculate distance and bearing function (correctly defined)
def calculate_distance_and_bearing(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(delta_lambda / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

    return distance, bearing

# GPS averaging function
def get_average_gps(samples=5):
    latitudes, longitudes = [], []
    for _ in range(samples):
        gps_msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if gps_msg:
            latitudes.append(gps_msg.lat / 1e7)
            longitudes.append(gps_msg.lon / 1e7)
        time.sleep(0.1)

    avg_lat = sum(latitudes) / len(latitudes)
    avg_lon = sum(longitudes) / len(longitudes)
    return avg_lat, avg_lon

sample_interval = 1.0

# Main control loop
try:
    while True:
        if other_lat is not None and other_lon is not None:
            own_lat, own_lon = get_average_gps(samples=5)
            distance, bearing = calculate_distance_and_bearing(own_lat, own_lon, other_lat, other_lon)
            print(f"Distance to Drone {Target_SysID}: {distance:.2f} m, Bearing: {bearing:.2f}°")

            if distance > 1.0:
                print("Driving towards target drone.")
                connection.mav.set_position_target_global_int_send(
                    0,
                    connection.target_system,
                    connection.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    int(0b110111111000),
                    int(other_lat * 1e7),
                    int(other_lon * 1e7),
                    0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0
                )
            else:
                print("Reached target drone, stopping.")
                connection.mav.set_position_target_global_int_send(
                    0,
                    connection.target_system,
                    connection.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    int(0b110111111000),
                    int(own_lat * 1e7),
                    int(own_lon * 1e7),
                    0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0
                )
        else:
            print("Waiting for target drone GPS data...")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nExiting and stopping rover.")
