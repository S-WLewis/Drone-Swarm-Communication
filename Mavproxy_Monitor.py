#!/usr/bin/env python3

import socket
import threading
import json
import time
import math
from pymavlink import mavutil
from collections import deque
import pandas as pd

# MAVLink connection
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print(f"Connected to system {connection.target_system}, component {connection.target_component}")

# System ID (Unique per drone)
SysID = 2  # Change this for each drone in your swarm

# Data buffer
buffer_size = 100
data_buffer = deque(maxlen=buffer_size)

# Drone's own position (initialized)
own_lat, own_lon, own_alt = None, None, None

# UDP setup
UDP_IP = '<broadcast>'
UDP_PORT = 14550

# Broadcast socket
broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Listener socket
listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
listen_sock.bind(('', UDP_PORT))

# Calculate distance and bearing between two GPS coordinates
def calculate_distance_and_bearing(lat1, lon1, lat2, lon2):
    R = 6371000  # Earth radius in meters
    phi1, phi2 = map(math.radians, [lat1, lat2])
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(delta_lambda)
    bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

    return distance, bearing

# Listener thread function
def listener():
    global own_lat, own_lon, own_position_available
    print("Listener started, awaiting broadcasts from other drones.")
    while True:
        try:
            data, addr = listen_sock.recvfrom(1024)
            message = json.loads(data.decode('utf-8'))

            # Ignore messages from self
            if message.get('SysID') != SysID and 'lat' in message:
                other_lat, other_lon = message['lat'], message['lon']

                if own_lat is not None and own_lon is not None:
                    distance, bearing = calculate_distance_and_bearing(
                        own_lat, own_lon, other_lat, other_lon
                    )
                    print(f"Drone {message['SysID']} | Distance: {distance:.2f} m | Bearing: {bearing:.2f}°")

        except Exception as e:
            print(f"Listener error: {e}")

# Start listener thread
listener_thread = threading.Thread(target=listener, daemon=True)
listener_thread.start()

sample_interval = 1.0  # seconds

try:
    while True:
        battery_msg = connection.recv_match(type='SYS_STATUS', blocking=False)
        attitude_msg = connection.recv_match(type='ATTITUDE', blocking=False)
        gps_msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)

        data_entry = {'timestamp': time.time(), 'SysID': SysID}

        if battery_msg:
            data_entry['voltage_battery'] = battery_msg.voltage_battery / 1000.0

        if attitude_msg:
            data_entry.update({
                'roll': attitude_msg.roll,
                'pitch': attitude_msg.pitch,
                'yaw': attitude_msg.yaw
            })

        if gps_msg:
            own_lat = gps_msg.lat / 1e7
            own_lon = gps_msg.lon / 1e7
            own_alt = gps_msg.alt / 1000.0

            data_entry.update({
                'lat': own_lat,
                'lon': own_lon,
                'alt': own_alt
            })

        # Only broadcast if we have GPS data (to ensure location is valid)
        if 'lat' in data_entry and 'lon' in data_entry:
            data_buffer.append(data_entry)
            broadcast_message = json.dumps(data_entry).encode('utf-8')
            broadcast_sock.sendto(broadcast_message, (UDP_IP, UDP_PORT))
            # Optionally print own broadcast
            # print(f"Broadcasting: {data_entry}")

        time.sleep(sample_interval)

except KeyboardInterrupt:
    print("\nExiting...")
    df = pd.DataFrame(data_buffer)
    df.to_csv(f'telemetry_data_SysID_{SysID}.csv', index=False)
    print(f"Telemetry data saved to 'telemetry_data_SysID_{SysID}.csv'")
