import argparse
import os
import sys
import yaml
import zmq
import time
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from rl.ppo import PPO
from rl.CarlaEnv.carla_env import CarlaEnv

def parse_args():
    """
    Parse command line arguments.
    """
    parser = argparse.ArgumentParser(description="Serve a trained PPO model.")

    parser.add_argument("--obs_port", type=int, default=5000, help="Port to receive observations.")
    parser.add_argument("--pred_port", type=int, default=5001, help="Port to send predictions.")
    parser.add_argument("--model", type=str, required=True, help="Trained model directory.")
    parser.add_argument("--checkpoint", type=str, default=None, help="Checkpoint to load.")
    return parser.parse_args()

class ModelServer:
    """
    Class to serve the trained model.
    """

    def __init__(self, obs_port, pred_port, model_name, checkpoint=None):
        self.obs_port = obs_port
        self.pred_port = pred_port
        self.model_name = model_name
        self.checkpoint = checkpoint

    def start(self):
        # Load the trained model
        print("Loading model...")
        self.load_model(self.model_name)
        print("Model loaded successfully.")

        self.times = []
        self.count = 0

        # Set up ZMQ sockets
        context = zmq.Context()

        # SUB socket for receiving observations
        self.sub_socket = context.socket(zmq.SUB)
        obs_addr = f"tcp://localhost:{self.obs_port}"
        self.sub_socket.connect(obs_addr)
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, '')  # Subscribe to all topics
        print(f"Listening for observations on {obs_addr}")

        # PUB socket for sending predictions
        self.pub_socket = context.socket(zmq.PUB)
        pred_addr = f"tcp://*:{self.pred_port}"
        self.pub_socket.bind(pred_addr)
        print(f"Publishing predictions on {pred_addr}")

        try:
            while True:
                # Wait for observations from the client
                message = self.sub_socket.recv_json()

                # Process the observations and send predictions
                response = self.handle_request(message)
                self.pub_socket.send_json(response)
        except KeyboardInterrupt:
            print("Shutting down server...")
        finally:
            self.sub_socket.close()
            self.pub_socket.close()

    def load_model(self, model_name):
        """
        Load the trained PPO model.
        """
        model_dir = os.path.join("models", model_name)
        with open(os.path.join(model_dir, "training.yaml"), 'r') as file:
            train_params = yaml.safe_load(file)

        hyperparameters = train_params["hyperparameters"]
        pi_hidden_sizes = hyperparameters["pi_hidden_sizes"]
        vf_hidden_sizes = hyperparameters["vf_hidden_sizes"]
        history_length = hyperparameters["history_length"]

        self.env = CarlaEnv(history_length=history_length, map="Town02_Opt")
        input_shape = self.env.observation_space.shape[0]
        action_space = self.env.action_space

        self.model = PPO(input_shape, action_space,
                        pi_hidden_sizes=pi_hidden_sizes, vf_hidden_sizes=vf_hidden_sizes,
                        history_length=history_length,
                        model_dir=os.path.join("models", model_name))
        self.model.init_session()
        if self.checkpoint:
            print(f"Loading model {model_name} from checkpoint {self.checkpoint}")
            self.model.load_custom_checkpoint(self.checkpoint)
        else:
            print(f"Loading model {model_name} from latest checkpoint")
            self.model.load_latest_checkpoint()

    def handle_request(self, input_data):
        """
        Handle incoming observations and return predictions.
        """
        try:
            t0 = time.time()
            state = self.env.observation

            # Predict action
            action, _ = self.model.predict(state, greedy=True)
            prediction = self.env.network_to_carla(action)
            # prediction = self.env.network_to_carla(action, state)

            # Prepare the response
            response = {"prediction": prediction}

            self.count += 1
            if self.count > 1000 and len(self.times) < 1000:
                self.times.append((time.time() - t0)*1e3)
                print(f"#{len(self.times)}: Mean= {np.mean(self.times):.4f}\t"
                      f"Std= {np.std(self.times):.4f}\t"
                      f"Min= {np.min(self.times):.4f}\t"
                      f"Max= {np.max(self.times):.4f}")

            return response
        except Exception as e:
            print(f"Error: {e}")
            return {"error": str(e)}

if __name__ == "__main__":
    args = parse_args()
    server = ModelServer(args.obs_port, args.pred_port, args.model, args.checkpoint)
    server.start()