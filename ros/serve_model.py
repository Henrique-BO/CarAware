import argparse
import socket
import os
import sys
import json
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from rl.ppo import PPO
from rl.CarlaEnv.carla_env import CarlaEnv


def parse_args():
    """
    Parse command line arguments.
    """
    parser = argparse.ArgumentParser(description="Serve a trained PPO model.")

    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host to bind the server.")
    parser.add_argument("--port", type=int, default=5000, help="Port to bind the server.")
    parser.add_argument("--model", type=str, required=True, help="Trained model directory.")
    return parser.parse_args()


class ModelServer:
    """
    Class to serve the trained model.
    """

    def __init__(self, host, port, model_name):
        self.host = host
        self.port = port
        self.model_name = model_name

    def start(self):
        # Load the trained model
        print("Loading model...")
        self.load_model(self.model_name)
        print("Model loaded successfully.")

        # Set up the server
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.host, self.port))
        self.server.listen(5)
        print(f"Server listening on {self.host}:{self.port}")

        try:
            while True:
                # Accept a new client connection
                client_socket, addr = self.server.accept()
                print(f"Connection from {addr[0]}:{addr[1]}")

                # Handle the client in a separate thread or process
                self.handle_client(client_socket)
        except KeyboardInterrupt:
            print("Shutting down server...")
        finally:
            self.server.close()

    def load_model(self, model_name):
        """
        Load the trained PPO model.
        """
        self.env = CarlaEnv(map="Town01")
        input_shape = self.env.observation_space.shape[0]
        action_space = self.env.action_space
        model_dir = os.path.join("models", model_name)

        self.model = PPO(input_shape=input_shape, action_space=action_space, model_dir=model_dir)
        self.model.init_session()
        self.model.load_latest_checkpoint()

    def handle_client(self, client_socket):
        """
        Handle incoming client requests.
        """
        try:
            while True:
                # Receive data from the client
                data = client_socket.recv(1024).decode("utf-8")
                if not data:
                    break

                # Parse the input JSON
                input_data = json.loads(data)
                input_states = np.array(input_data["state"]).reshape(1, -1)

                # Predict action
                action, _ = self.model.predict(input_states, greedy=False)
                action = self.env.network_to_carla(action)

                # Send the action back to the client
                response = {"action": action}
                client_socket.send(json.dumps(response).encode("utf-8"))
        except Exception as e:
            print(f"Error: {e}")
        finally:
            client_socket.close()


if __name__ == "__main__":
    args = parse_args()
    server = ModelServer(args.host, args.port, args.model)
    server.start()