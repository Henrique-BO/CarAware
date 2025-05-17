import numpy as np
import zmq
import threading
import time

def read_observations():
    while True:
        try:
            data = socket.recv_json(flags=zmq.NOBLOCK)
            observations = data["observations"]

            assert isinstance(observations, list), "Observations should be a list"
            print(f"Observations: {observations}")
        except zmq.Again:
            pass
        except Exception as e:
            print(f"Error reading ZMQ data: {e}")

if __name__ == "__main__":
    # Connect to the server
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    addr = "tcp://localhost:5000"  # Update to match the observation stream port
    socket.connect(addr)
    socket.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages
    obs_thread = threading.Thread(target=read_observations, daemon=True)
    obs_thread.start()
    print(f"Connected to observation stream on {addr}")

    while True:
        time.sleep(1)
