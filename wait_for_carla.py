#!/usr/bin/env python
import socket
import sys
import time

host = "localhost"
port = 2000
max_attempts = 60
sleep_time = 2

for attempt in range(max_attempts):
    try:
        with socket.create_connection((host, port), timeout=1):
            print("CARLA is ready.")
            sys.exit(0)
    except OSError:
        print(f"Waiting for CARLA ({attempt + 1}/{max_attempts})...")
        time.sleep(sleep_time)

print("Timeout: CARLA did not start.")
sys.exit(1)
