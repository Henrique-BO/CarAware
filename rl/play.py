import os
import random
import time

import numpy as np
import tensorflow as tf

from rl.ppo import PPO
from rl.run_eval import run_eval
from rl.CarlaEnv.carla_env import CarlaEnv as CarlaEnv


def play(train_params, sim_params, simulation, top_view):  # start_carla=True
    model_name = train_params["model_name"]
    # train_model = train_params["model_name"]
    train_model = "Latest"
    record_play_stats = train_params["record_play_stats"]

    hyper_params = train_params["hyperparameters"]

    learning_rate = hyper_params["learning_rate"]
    history_length = hyper_params["history_length"]
    pi_hidden_sizes = hyper_params["pi_hidden_sizes"]
    vf_hidden_sizes = hyper_params["vf_hidden_sizes"]
    lr_decay = hyper_params["lr_decay"]
    discount_factor = hyper_params["discount_factor"]
    gae_lambda = hyper_params["gae_lambda"]
    ppo_epsilon = hyper_params["ppo_epsilon"]
    initial_std = hyper_params["initial_std"]
    value_scale = hyper_params["value_scale"]
    entropy_scale = hyper_params["entropy_scale"]
    horizon = hyper_params["horizon"]
    num_training = hyper_params["num_training"]
    num_epochs = hyper_params["num_epochs"]
    batch_size = hyper_params["batch_size"]
    synchronous = hyper_params["synchronous"]
    action_smoothing = hyper_params["action_smoothing"]
    reward_fn = hyper_params["reward_fn"]
    seed = hyper_params["seed"]
    eval_interval = hyper_params["eval_interval"]
    record_eval = hyper_params["record_eval"]

    map = sim_params["episodes"]["map"]
    num_episodes = sim_params["episodes"]["num_episodes"]
    fps = sim_params["top_view"]["fps"]
    ego_num = sim_params["vehicles"]["ego_vehicle_num"]
    eval_time        = 999999999  # roda por tempo indefinido até ESC ser pressionado

    simulation.eval = True

    # Set seeds
    if isinstance(seed, int):
        tf.random.set_random_seed(seed)
        np.random.seed(seed)
        random.seed(seed)

    # Create env
    print("Creating environment")
    env = CarlaEnv(#obs_res=(160, 80),
                   action_smoothing=action_smoothing,
                   #encode_state_fn=encode_state_fn,
                   reward_fn=reward_fn,
                   synchronous=synchronous,
                   fps=fps,
                   #start_carla=start_carla
                   simulation=simulation, top_view=top_view,
                   ego_num=ego_num,
                   map=map)

    if isinstance(seed, int):
        env.seed(seed)
    best_eval_reward = -float("inf")

    # Environment constants
    input_shape = env.observation_space.shape[0]
    num_actions = env.action_space.shape[0]

    #input_shape = env.observation_space["GNSS"].shape[0] + 1  # input_shape = np.array([vae.z_dim + len(measurements_to_include)])
    #num_actions = env.action_space["Obj_Coord"].shape[0] + 1 # antes era +1

    # Create model
    print("Creating model")
    model = PPO(input_shape, env.action_space,
                pi_hidden_sizes=pi_hidden_sizes, vf_hidden_sizes=vf_hidden_sizes,
                history_length=history_length,
                learning_rate=learning_rate, lr_decay=lr_decay,
                epsilon=ppo_epsilon, initial_std=initial_std,
                value_scale=value_scale, entropy_scale=entropy_scale,
                model_dir=os.path.join("models", model_name))

    model.init_session()

    if train_model == "Latest":
        model.load_latest_checkpoint()
    else:  # Custom model
        model.load_custom_checkpoint(train_model)

    print("Rodando em modo PREVIEW com modelo: \"{}\".".format(model_name))
    time.sleep(5)

    simulation.simulation_status = "Play"
    play = True
    print(simulation.ego_vehicle)
    run_eval(env, model, None, eval_time, simulation, ego_num, play, record_play_stats)

    #eval_reward = run_eval(env, model, eval_time=eval_time)
    #print("Reward final: ", eval_reward)

    # Reprodução finalizada
    simulation.simulation_status = "Complete"