import argparse
import os
import sys
import json
import numpy as np
import zmq

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
# from rl.ppo import PPO, ModelWrapper
# from rl.CarlaEnv.carla_env import CarlaEnv


def parse_args():
    """
    Parse command line arguments.
    """
    parser = argparse.ArgumentParser(description="Serve a trained PPO model.")

    parser.add_argument("--host", type=str, default="localhost", help="Host to bind the server.")
    parser.add_argument("--port", type=int, default=5001, help="Port to bind the server.")
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
        # print("Loading model...")
        # self.load_model(self.model_name)
        # print("Model loaded successfully.")

        # Set up the server
        context = zmq.Context()
        self.socket = context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:5050")
        print(f"Server listening on tcp://*:5050")

        try:
            while True:
                # Wait for a request from the client
                message = self.socket.recv_json()
                print(f"Received request: {message}")

                # Process the request and send a response
                response = self.handle_request(message)
                self.socket.send_json(response)
        except KeyboardInterrupt:
            print("Shutting down server...")
        finally:
            self.socket.close()

    def load_model(self, model_name):
        """
        Load the trained PPO model.
        """
        self.env = CarlaEnv(map="Town01")
        num_inputs = self.env.observation_space.shape[0]
        sequence_length = 4 # TODO get from model manifest
        action_space = self.env.action_space
        model_dir = os.path.join("models", model_name)

        self.model = PPO(num_inputs, sequence_length, action_space=action_space, model_dir=model_dir)
        self.model.init_session()
        self.model.load_latest_checkpoint()
        self.model_wrapper = ModelWrapper(self.model)

    def handle_request(self, input_data):
        """
        Handle incoming client requests and return a response.
        """
        try:
            observations = np.array(input_data["observations"])

            # Parse the input JSON
            # input_states = np.array(input_data["state"])
            # position = np.array(input_data["position"]).reshape(1, -1)

            # # Predict action
            # _, prediction, _, _ = self.model_wrapper.predict(position, input_states, greedy=True)
            # prediction = self.env.network_to_carla(prediction)
            prediction = [observations[0], observations[1]]

            # Prepare the response
            response = {"prediction": prediction}
            return response
        except Exception as e:
            print(f"Error: {e}")
            return {"error": str(e)}

if __name__ == "__main__":
    args = parse_args()
    server = ModelServer(args.host, args.port, args.model)
    server.start()