import time
from collections import deque

import gym
from gym.utils import seeding
import reward_functions as reward_functions
import math
import numpy as np
import zmq
import threading

# TODO:
# - Some solution to avoid using the same env instance for training and eval
# - Just found out gym provides ObservationWrapper and RewardWrapper classes.
#   Should replace encode_state_fn and reward_fn with these.

class CarlaEnv(gym.Env):
    """
        This is a simple CARLA environment where the goal is to drive in a lap
        around the outskirts of Town07. This environment can be used to compare
        different models/reward functions in a realtively predictable environment.

        To run an agent in this environment, either start start CARLA beforehand with:

        Synchronous:  $> ./CarlaUE4.sh Town07 -benchmark -fps=30
        Asynchronous: $> ./CarlaUE4.sh Town07

        Or, pass argument -start_carla in the command-line.
        Note that ${CARLA_ROOT} needs to be set to CARLA's top-level directory
        in order for this option to work.

        And also remember to set the -fps and -synchronous arguments to match the

                command-line arguments of the simulator (not needed with -start_carla.)

        Note that you may also need to add the following line to
        Unreal/CarlaUE4/Config/DefaultGame.ini to have the map included in the package:

        +MapsToCook=(FilePath="/Game/Carla/Maps/Town07")
    """

    metadata = {
        "render.modes": ["human", "rgb_array", "rgb_array_no_hud", "state_pixels"]
    }

    def __init__(self, host="127.0.0.1", port=2000,
                 reward_fn=None, #encode_state_fn=None,
                 synchronous=False, fps=30, simulation=None, action_smoothing=0.9, top_view = None,
                 ego_num = 0,map = "",last_positions_training = False, history_length=1): #start_carla=False, viewer_res=(1280, 720), obs_res=(1280, 720),
        """
            Initializes a gym-like environment that can be used to interact with CARLA.

            Connects to a running CARLA enviromment (tested on version 0.9.5) and
            spwans a lincoln mkz2017 passenger car with automatic transmission.

            This vehicle can be controlled using the step() function,
            taking an action that consists of [steering_angle, throttle].

            host (string):
                IP address of the CARLA host
            port (short):
                Port used to connect to CARLA
            viewer_res (int, int):
                Resolution of the spectator camera (placed behind the vehicle by default)
                as a (width, height) tuple
            obs_res (int, int):
                Resolution of the observation camera (placed on the dashboard by default)
                as a (width, height) tuple
            reward_fn (function):
                Custom reward function that is called every step.
                If None, no reward function is used.
            encode_state_fn (function):
                Function that takes the image (of obs_res resolution) from the
                observation camera and encodes it to some state vector to returned
                by step(). If None, step() returns the full image.
            action_smoothing:
                Scalar used to smooth the incoming action signal.
                1.0 = max smoothing, 0.0 = no smoothing
            fps (int):
                FPS of the client. If fps <= 0 then use unbounded FPS.
                Note: Sensors will have a tick rate of fps when fps > 0,
                otherwise they will tick as fast as possible.
            synchronous (bool):
                If True, run in synchronous mode (read the comment above for more info)
            start_carla (bool):
                Automatically start CALRA when True. Note that you need to
                set the environment variable ${CARLA_ROOT} to point to
                the CARLA root directory for this option to work.
        """

        # Disponibiliza classe para demais funções
        self._simulation = simulation
        self._top_view = top_view

        # Initialize prediction variables
        self.distance = 0
        self.ego_num = ego_num
        self.last_reward = []
        self.last_distance = 0
        self.reward_fn = reward_fn
        self.observation = []
        self.last_positions_training = last_positions_training
        self.history_length = history_length

        # Setup gym environment
        self.synchronous = synchronous
        self.seed()

        # Read observation stream from ZMQ SUB socket
        self._observations = None
        self.new_observations = False
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.obs_addr = "tcp://localhost:5000"
        self.socket.connect(self.obs_addr)
        self.obs_thread = threading.Thread(target=self.read_observation_stream, daemon=True)
        self.obs_thread.start()

        # ZMQ REQ socket for reset requests
        self.reset_socket = self.context.socket(zmq.REQ)
        self.reset_addr = "tcp://localhost:5002"
        self.reset_socket.connect(self.reset_addr)

        if not last_positions_training:
            # Configura vetor unitário de entrada dos espaços de observação e ação - COMPLETO
            # GNSS_X, GNSS_Y, accel_x, accel_y, accel_z, GYRO_pitch, GYRO_yaw, GYRO_roll, compass, speed, stw_angle
            obs_low = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1] * self.history_length
            obs_high = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1] * self.history_length

            # Configura vetor unitário de entrada dos espaços de observação e ação - COMPLETO
            # accel_x, accel_y, accel_z, GYRO_pitch, GYRO_yaw, GYRO_roll, compass, speed, stw_angle
            # obs_low = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
            # obs_high = [1, 1, 1, 1, 1, 1, 1, 1, 1]

            # Configura vetor unitário de entrada dos espaços de observação e ação - SMALL
            # GNSS_X, GNSS_Y, compass, speed, stw_angle
            #obs_low = [-1, -1, -1, -1, -1]    # [-15, 90, -99.9, -99.9, -99.9, -99.9, -99.9, -99.9, 0, 0, 0]
            #obs_high = [1, 1, 1, 1, 1]  # [210, 310, 99.9, 99.9, 99.9, 99.9, 99.9, 99.9, 360, 100, 360]

        else:  # com treinamento de posição
            # Configura vetor unitário de entrada dos espaços de observação e ação - COMPLETO
            # GNSS_X, GNSS_Y, accel_x, accel_y, accel_z, GYRO_pitch, GYRO_yaw, GYRO_roll, compass, speed, stw_angle
            #obs_low = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]
            #obs_high = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

            # Configura vetor unitário de entrada dos espaços de observação e ação - SMALL
            # GNSS_X, GNSS_Y, compass, speed, stw_angle, pos1_x, pos1_y, pos2_x, pos2_y, pos3_x, pos3_y, pos4_x, pos4_y
            obs_low = [-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1]    # [-15, 90, -99.9, -99.9, -99.9, -99.9, -99.9, -99.9, 0, 0, 0]
            obs_high = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]  # [210, 310, 99.9, 99.9, 99.9, 99.9, 99.9, 99.9, 360, 100, 360]

        act_low = [-1, -1]  # [-15, 90]
        act_high = [1, 1]  # [210, 310]

        # Valores reais em formato de vetor
        #self.vetor_obs_low = [-15, 90, -99.9, -99.9, -99.9, -99.9, -99.9, -99.9, 0, 0, 0, 0]
        #self.vetor_obs_high = [210, 310, 99.9, 99.9, 99.9, 99.9, 99.9, 99.9, 360, 100, 360, 10]  # 10 = ego_num-1
        #self.vetor_act_low = [-15, 90]
        #self.vetor_act_high = [210, 310]

        self.scale = 500 # Map width and height in meters
        self.map_offsets = {
            "Town01": (200, 200), # x in (-50, 450), y in (-50, 450)
            "Town02": (200, 200), # x in (-50, 450), y in (-50, 450)
            "Town02_Opt": (200, 200), # x in (-50, 450), y in (-50, 450)
            "Town10HD_Opt": (200, 200), # x in (-50, 450), y in (-50, 450)
        }
        self.default_offset = (0, 0) # x in (-250, 250), y in (-250, 250)
        offset = self.map_offsets.get(map, self.default_offset)
        self.compute_normalization(offset[0], offset[1])  # Compute normalization based on map offset

        # self.vetor_act_low = [offset[0] - self.scale / 2.0, offset[1] - self.scale / 2.0]
        # self.vetor_act_high = [offset[0] + self.scale / 2.0, offset[1] + self.scale / 2.0]

        # ACT - Define limites de coordenadas para mapa escolhido

        if simulation:
            waypoint_list = simulation.world.get_map().generate_waypoints(2.0)  # Gera waypoints para o mapa
            min_x = min(waypoint.transform.location.x for waypoint in waypoint_list)
            max_x = max(waypoint.transform.location.x for waypoint in waypoint_list)
            print(f"Map {map} X limits: {min_x} to {max_x}")
            min_y = min(waypoint.transform.location.y for waypoint in waypoint_list)
            max_y = max(waypoint.transform.location.y for waypoint in waypoint_list)
            print(f"Map {map} Y limits: {min_y} to {max_y}")

            self.map_min = [min_x - 20, min_y - 20]
            self.map_max = [max_x + 20, max_y + 20]
            print(f"Map {map} limits: {self.map_min} to {self.map_max}")
        else:
            if map == "Town01":
                self.map_min = [-20, -10]  #[-15, 90]  # Coordenadas X,Y
                self.map_max = [410, 340]  #[210, 310]  # Coordenadas X,Y
            elif map == "Town02":
                self.map_min = [-15, 95]  # Coordenadas X,Y
                self.map_max = [205, 315]  # Coordenadas X,Y
            elif map == "Town10HD_Opt":
                self.map_min = [-130, -90]  # Coordenadas X,Y
                self.map_max = [125, 155]  # Coordenadas X,Y
            elif map == "Random" or map == "Gradual_Random":
                chosen_map = simulation.chosen_random_map
                if chosen_map == "Town01":
                    self.map_min = [-15, 90]  # Coordenadas X,Y
                    self.map_max = [210, 310]  # Coordenadas X,Y
                elif chosen_map == "Town02":
                    self.map_min = [-20, -10]  # Coordenadas X,Y
                    self.map_max = [410, 340]  # Coordenadas X,Y
                elif chosen_map == "Town10HD_Opt":
                    self.map_min = [-130, -90]  # Coordenadas X,Y
                    self.map_max = [125, 155]  # Coordenadas X,Y
            else:
                self.map_min = [-250, -250]  # Coordenadas X,Y
                self.map_max = [250, 250]  # Coordenadas X,Y

        assert(self.map_min[0] < self.map_max[0] and self.map_min[1] < self.map_max[1]), \
            f"Map limits are invalid: {self.map_min} to {self.map_max}"
        assert(self.map_max[0] - self.map_min[0] <= self.scale and self.map_max[1] - self.map_min[1] <= self.scale), \
            f"Map size exceeds scale: {self.map_max[0] - self.map_min[0]}x{self.map_max[1] - self.map_min[1]} > {self.scale}x{self.scale}"

        # if map == "Town01":
        #     self.map_min = [-20, -10]  #[-15, 90]  # Coordenadas X,Y
        #     self.map_max = [410, 340]  #[210, 310]  # Coordenadas X,Y
        # elif map == "Town02":
        #     self.map_min = [-15, 95]  # Coordenadas X,Y
        #     self.map_max = [205, 315]  # Coordenadas X,Y
        # elif map == "Town10HD_Opt":
        #     self.map_min = [-130, -90]  # Coordenadas X,Y
        #     self.map_max = [125, 155]  # Coordenadas X,Y
        # elif map == "Random" or map == "Gradual_Random":
        #     chosen_map = simulation.chosen_random_map
        #     if chosen_map == "Town01":
        #         self.map_min = [-15, 90]  # Coordenadas X,Y
        #         self.map_max = [210, 310]  # Coordenadas X,Y
        #     elif chosen_map == "Town02":
        #         self.map_min = [-20, -10]  # Coordenadas X,Y
        #         self.map_max = [410, 340]  # Coordenadas X,Y
        #     elif chosen_map == "Town10HD_Opt":
        #         self.map_min = [-130, -90]  # Coordenadas X,Y
        #         self.map_max = [125, 155]  # Coordenadas X,Y
        # else:
        #     self.map_min = [-250, -250]  # Coordenadas X,Y
        #     self.map_max = [250, 250]  # Coordenadas X,Y

        # self.diagonal = np.linalg.norm(np.array(self.vetor_act_high) - np.array(self.vetor_act_low))

        # if not last_positions_training:
        #     # Valores reais em formato de vetor
        #     # OBS COMPLETO
        #     # GNSS_X, GNSS_Y, accel_x, accel_y, accel_z, GYRO_pitch, GYRO_yaw, GYRO_roll, compass, speed, stw_angle
        #     # self.vetor_obs_low = [self.vetor_act_low[0], self.vetor_act_low[1], -99.9, -99.9, -99.9, -99.9, -99.9, -99.9, 0, 0, -180] * self.history_length
        #     # self.vetor_obs_high = [self.vetor_act_high[0], self.vetor_act_high[1], 99.9, 99.9, 99.9, 99.9, 99.9, 99.9, 360, 100, 180] * self.history_length

        #     # EKF_x, EKF_y, accel_x, accel_y, accel_z, GYRO_pitch, GYRO_yaw, GYRO_roll, compass, speed, stw_angle
        #     self.vetor_obs_low =  [self.vetor_act_low[0],  self.vetor_act_low[1], -100, -100, -100, -1.0, -1.0, -1.0, -180,  0, -180] * self.history_length
        #     self.vetor_obs_high = [self.vetor_act_high[0], self.vetor_act_high[1], 100,  100,  100,  1.0,  1.0,  1.0,  180, 10,  180] * self.history_length

        #     # OBS SMALL
        #     # GNSS_X, GNSS_Y, compass, speed, stw_angle
        #     #self.vetor_obs_low = [-15, 90, 0, 0, 0]
        #     #self.vetor_obs_high = [210, 310, 360, 100, 360]  # 10 = ego_num-1
        # else:
        #     # OBS SMALL
        #     self.vetor_obs_low = [-15, 90, 0, 0, -180]
        #     self.vetor_obs_low.extend(self.vetor_act_low)
        #     self.vetor_obs_low.extend(self.vetor_act_low)
        #     self.vetor_obs_low.extend(self.vetor_act_low)
        #     self.vetor_obs_low.extend(self.vetor_act_low)

        #     self.vetor_obs_high = [210, 310, 360, 100, 180]  # 10 = ego_num-1
        #     self.vetor_obs_high.extend(self.vetor_act_high)
        #     self.vetor_obs_high.extend(self.vetor_act_high)
        #     self.vetor_obs_high.extend(self.vetor_act_high)
        #     self.vetor_obs_high.extend(self.vetor_act_high)

        # self.action_length = len(self.vetor_act_low)  # será usado para remontar a matriz

        '''
        # preenche vetor total de observação e ação
        observ_low = []
        observ_high = []
        action_low = []
        action_high = []

        for _ in range(ego_num):
            observ_low.extend(obs_low)
            observ_high.extend(obs_high)
            action_low.extend(act_low)
            action_high.extend(act_high)
        '''

        self.action_space = gym.spaces.Box(low=np.array(act_low), high=np.array(act_high))  # 200 a 1500 / 0 a 3   , dtype=np.float32
        # GNSS_X,GNSS_Y, Tipo

        self.observation_space = gym.spaces.Box(low=np.array(obs_low), high=np.array(obs_high))  # 200 a 1500 / 0 a 3

        self.metadata["video.frames_per_second"] = self.fps = self.average_fps = fps

        self.veh = None
        self.last_observation = [0] * self.observation_space.shape[0]
        self.carla_state = [0] * self.observation_space.shape[0]

        # Reset env to set initial state
        self.reset()

    def compute_normalization(self, offset_x, offset_y):
        """
        Compute the normalization factor based on the offset values.
        This is used to scale the action space to the CARLA map limits.
        """
        self.vetor_act_low = [offset_x - self.scale / 2.0, offset_y - self.scale / 2.0]
        self.vetor_act_high = [offset_x + self.scale / 2.0, offset_y + self.scale / 2.0]
        print(f"Action space limits set to: {self.vetor_act_low} to {self.vetor_act_high}")

        # EKF_x, EKF_y, accel_x, accel_y, accel_z, GYRO_pitch, GYRO_yaw, GYRO_roll, compass, speed, stw_angle
        self.vetor_obs_low =  [self.vetor_act_low[0],  self.vetor_act_low[1], -10, -10, -10, -1.0, -1.0, -1.0, -180,  0, -180] * self.history_length
        self.vetor_obs_high = [self.vetor_act_high[0], self.vetor_act_high[1], 10,  10,  10,  1.0,  1.0,  1.0,  180, 10,  180] * self.history_length


    def seed(self, seed=None):

        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self, is_training=True, randomize_offset=False):

        self.terminal_state = False  # Set to True when we want to end episode
        self.closed = False         # Set to True when ESC is pressed
        self.start_t = time.time()
        self.step_count = 0
        self.is_training = is_training
        initial_reward = 0

        #initial_state = [0, 0, 0, 0, 0]  # Small
        if not self.last_positions_training:
            initial_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        else:  # Treinando com últimas posições
            initial_state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


        # Initialize the observation history filled with zeros
        self.observation_history = deque(maxlen=self.history_length)
        for _ in range(self.history_length):
            self.observation_history.append(initial_state)
        initial_state = np.concatenate(self.observation_history).tolist()

        # Send reset request via ZMQ
        try:
            self.reset_socket.send_json({"command": "reset"})
            response = self.reset_socket.recv_json()
            if response.get("status") != "success":
                raise RuntimeError("Failed to reset EKF: Received negative response from reset service.")
        except Exception as e:
            raise RuntimeError(f"Failed to reset EKF: {e}")

        # Randomize offset, respecting map limits
        # Scale/2 = 250
        # Town01: x range (-20, 410), y range (-10, 340)
        # x_offset can be between 160 and 230
        # y_offset can be between 90 and 240
        # 160 = map_max[0] - self.scale / 2.0
        # 230 = map_min[0] + self.scale / 2.0
        if randomize_offset:
            offset_x = np.random.uniform(self.map_max[0] - self.scale / 2.0, self.map_min[0] + self.scale / 2.0)
            offset_y = np.random.uniform(self.map_max[1] - self.scale / 2.0, self.map_min[1] + self.scale / 2.0)
            print(f"Randomized offset: ({offset_x}, {offset_y})")
            self.compute_normalization(offset_x, offset_y)  # Compute normalization based on map offset

        #state_lst = []
        #terminal_state = []
        #for veh in range(self.ego_num):
        #    state_lst.append(initial_state)
        #    terminal_state.append(self.terminal_state)
        # Return initial observation - OBS, Rewards, terminal_state
        return initial_state, self.terminal_state, initial_reward
        #self.step(None)[0]

    def step(self, action, veh = None, veh_num = None):  # , single_veh = 10)
        if self.closed:
            raise Exception("CarlaEnv.step() called after the environment was closed." +
                            "Check for info[\"closed\"] == True in the learning loop.")

        self.veh = veh
        if action is not None:
            veh.prediction = self.network_to_carla(action)
            #actions = self.unflatten(actions)  # volta a ser um array 2D
            #for vehicle,action in zip(self._simulation.ego_vehicle,actions):
            #    vehicle.prediction = action

        #simulation.ego_vehicle[current_veh].prediction = action  # salva valor para desenhar prediction no Top-View

        # Garante que os dados são válidos para servirem de input pra rede neural
        while True:
            # self._get_observation(veh, veh_num)  # get most recent observations  - , single_veh)
            if self.new_observations:
                self.observation = np.concatenate(self.observation_history).tolist()  # Flatten the deque into a single list
                assert len(self.observation_history) == self.history_length, \
                        f"Expected {self.history_length} observations, got {len(self.observation_history)}"
                input_valido = True
                for obs in self.observation:
                    if obs is None or math.isnan(obs):
                        input_valido = False
                        break
                if input_valido:
                    self.new_observations = False
                    break

        # Call external reward fn
        reward, self.distance = reward_functions.calculate_reward(self, self.reward_fn, self.last_reward, self.last_distance, veh, veh_num)
        reward = reward * self.scale  # Normaliza a recompensa pela diagonal do mapa
        # print(f"\tNormalized reward: {reward}")

        self.last_reward = reward  # variável usada pra calcular a condição negativa
        self.last_distance = self.distance
        #self.total_reward += reward
        self.step_count += 1

        veh.pred_distance = self.distance

        #if self.distance is not None:
        #    for vehicle,distance in zip(self._simulation.ego_vehicle,self.distance):
        #        vehicle.pred_distance = distance
        """
        # Check for ESC press
        pygame.event.pump()
        if pygame.key.get_pressed()[K_ESCAPE]:
            self.close()
            self.terminal_state = True
        """
        return self.observation, reward, self.terminal_state

       # Transforma a lista de listas em um vetor de X por 1

    def read_observation_stream(self):
        """
        Continuously read the observation stream from the ZMQ SUB socket.
        """
        print(f"Listening for observations on {self.obs_addr}")
        while True:
            try:
                data = self.socket.recv_json(flags=zmq.NOBLOCK)
                observation = data["observations"]
                assert isinstance(observation, list), "Observations should be a list"
                assert len(observation) * self.history_length == self.observation_space.shape[0], \
                    f"Expected {self.observation_space.shape[0] / self.history_length} observations, got {len(observation)}"

                self.last_observation = observation
                if self.veh:
                    gt = self.veh.get_location()
                    self.carla_state = [
                        gt.x, gt.y,
                        self.veh.sens_imu.ue_accelerometer[0], self.veh.sens_imu.ue_accelerometer[1], self.veh.sens_imu.ue_accelerometer[2],
                        self.veh.sens_imu.ue_gyroscope[0], self.veh.sens_imu.ue_gyroscope[1], self.veh.sens_imu.ue_gyroscope[2],
                        self.veh.sens_imu.ue_compass_degrees,
                        self.veh.sens_spd_sas_speed,
                        self.veh.sens_spd_sas_angle
                    ]

                observation = self.carla_to_network(observation)  # Convert to network format
                self.observation_history.append(observation)  # Add the new observation to the deque
                self.observation = np.concatenate(self.observation_history).tolist()
                self.new_observations = True
            except zmq.Again:
                pass
            except Exception as e:
                print(f"Error reading ZMQ observation stream: {e}")

    def _get_observation(self, veh, veh_num):  # ,single_veh)

        # if not self.new_observations or not self._observations:
        #     print("No new observations or observations are None")
        #     return False
        #self.observation = {"GNSS": []}
        observation = []
        #for vehicle, idx in zip(self._simulation.ego_vehicle, range(len(self._simulation.ego_vehicle))):
        #for vehicle in self._simulation.ego_vehicle:

        # Varia input de número de veículo se estiver treinando com apenas um, para aumentar generalização
        #if single_veh != 10:
            #veh_idx = single_veh
        #else:
            #veh_idx = veh_num
        try:
            if veh.sens_gnss_input is not None and self._ekf_data is not None:
                if not self.last_positions_training:
                    #COMPLETO
                    # observation.append([veh.sens_gnss_input.x, veh.sens_gnss_input.y, veh.sens_imu.ue_accelerometer[0],
                    #                     veh.sens_imu.ue_accelerometer[1], veh.sens_imu.ue_accelerometer[2],
                    #                     veh.sens_imu.ue_gyroscope[0], veh.sens_imu.ue_gyroscope[1],
                    #                     veh.sens_imu.ue_gyroscope[2], veh.sens_imu.ue_compass_degrees,
                    #                     veh.sens_spd_sas_speed, veh.sens_spd_sas_angle])
                    self.observation = self._observations

                    #SMALL
                    #observation.append([veh.sens_gnss_input.x, veh.sens_gnss_input.y, veh.sens_imu.ue_compass_degrees,
                    #                    veh.sens_spd_sas_speed, veh.sens_spd_sas_angle])

                else:  #com positions training
                    #COMPLETO
                    #observation.append([veh.sens_gnss_input.x, veh.sens_gnss_input.y, veh.sens_imu.ue_accelerometer[0],
                    #                    veh.sens_imu.ue_accelerometer[1], veh.sens_imu.ue_accelerometer[2],
                    #                    veh.sens_imu.ue_gyroscope[0], veh.sens_imu.ue_gyroscope[1],
                    #                    veh.sens_imu.ue_gyroscope[2], veh.sens_imu.ue_compass_degrees,
                    #                    veh.sens_spd_sas_speed, veh.sens_spd_sas_angle])
                    # SMALL
                    observation.append([veh.sens_gnss_input.x, veh.sens_gnss_input.y, veh.sens_imu.ue_compass_degrees,
                                        veh.sens_spd_sas_speed, veh.sens_spd_sas_angle])
                    positions_list = []
                    for position_pair in veh.stacked_positions:
                        for position in position_pair:
                            positions_list.append(position)
                    observation[0].extend(positions_list)

            else:
                if not self.last_positions_training:
                    observation.append([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])  # COMPLETO
                    # observation.append([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])  # COMPLETO
                    #observation.append([[0, 0, 0, 0, 0]])  # SMALL
                else:
                    observation.append([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])  # COMPLETO

            # self.observation = observation
            # print("observation: ", observation)
            # self.observation = self.carla_to_network(observation[0])
            self.new_observations = False
            return True
        except:
            #print("Falha Sensores")
            self.observation = None
            return False


    def carla_to_network(self, data):  # comprime observação para espaço de -1 a 1

       network_data = []
       for obs_low, obs_high, data_sens in zip(self.vetor_obs_low, self.vetor_obs_high, data):
           range_data = (obs_high - obs_low)/2  # normaliza para -1 a +1
           norm_factor = -(range_data + obs_low)
           network_data.append((data_sens+norm_factor)/range_data)

       return network_data

       #return [item for sublist in data for item in sublist]


    def network_to_carla(self, data):  # lê o valor fornecido de -1 a 1 e converte para valores do Carla
        # length = len(data)/self.action_length

        carla_data = []
        for act_low, act_high, data_res in zip(self.vetor_act_low, self.vetor_act_high, data):
            range_data = (act_high - act_low) / 2  # normaliza para -1 a +1
            norm_factor = -(range_data + act_low)
            carla_data.append((data_res * range_data) - norm_factor)

            # flattened_data.append((data_sens+norm_factor)/range_data)

        return carla_data

