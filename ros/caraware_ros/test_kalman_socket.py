import numpy as np
import socket
import json

def test_socket_connection():
    # Connect to the server
    host = 'localhost'
    port = 5000

    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        print(f"Connected to server at {host}:{port}")

        # # Example request to compute localization error
        # request = {
        #     "get_error": True,
        #     "duration": 5  # Duration in seconds
        # }
        # client_socket.send(json.dumps(request).encode("utf-8"))
        # response = client_socket.recv(1024).decode("utf-8")
        # print("Response from server:", json.loads(response))

        # Example request to update process noise covariance
        Q = [[0.5,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.5,   0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.6,   0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.03,   0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.03,   0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.06,   0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.025,   0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.025,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.04,   0.0,    0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.01,   0.0,    0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.01,   0.0,    0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.02,   0.0,    0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.01,   0.0,    0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.01,   0.0],
             [0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.0,     0.0,     0.0,    0.0,    0.0,    0.0,    0.0,    0.0,    0.015]]
        request = {
            "update_Q": True,
            "Q": Q
        }
        client_socket.send(json.dumps(request).encode("utf-8"))
        response = client_socket.recv(1024).decode("utf-8")
        print("Response from server:", json.loads(response))

        # Example request to compute localization error
        request = {
            "get_error": True,
            "duration": 5  # Duration in seconds
        }
        client_socket.send(json.dumps(request).encode("utf-8"))
        response = client_socket.recv(1024).decode("utf-8")
        print("Response from server:", json.loads(response))

    except Exception as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()
        print("Connection closed")

if __name__ == "__main__":
    test_socket_connection()