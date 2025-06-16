import csv
import os
import random
import shutil

import numpy as np
import tensorflow as tf
import time
from datetime import datetime
import mlflow
import zmq

from rl.ppo import PPO
from rl.run_eval import run_eval
from rl.utils import compute_gae
from rl.CarlaEnv.carla_env import CarlaEnv


def log_params_recursive(params, prefix=""):
    for k, v in params.items():
        if isinstance(v, dict):
            log_params_recursive(v, prefix + k + "/")
        else:
            mlflow.log_param(prefix + k, v)

def train(train_params, sim_params, sens_params, simulation, top_view):  # start_carla=True
    model_name       = train_params["model_name"]
    train_model      = train_params.get("checkpoint", "latest")
    restart          = train_params["restart"]
    last_positions_training = train_params["last_positions_training"]
    randomize_offset = train_params["randomize_offset"]
    # randomize_offset = False

    hyper_params = train_params["hyperparameters"]
    pi_hidden_sizes = hyper_params["pi_hidden_sizes"]
    vf_hidden_sizes = hyper_params["vf_hidden_sizes"]
    history_length   = hyper_params["history_length"]
    learning_rate    = hyper_params["learning_rate"]
    lr_decay         = hyper_params["lr_decay"]
    discount_factor  = hyper_params["discount_factor"]
    gae_lambda       = hyper_params["gae_lambda"]
    ppo_epsilon      = hyper_params["ppo_epsilon"]
    initial_std      = hyper_params["initial_std"]
    value_scale      = hyper_params["value_scale"]
    entropy_scale    = hyper_params["entropy_scale"]
    horizon          = hyper_params["horizon"]
    num_training     = hyper_params["num_training"]
    num_epochs       = hyper_params["num_epochs"]
    batch_size       = hyper_params["batch_size"]
    synchronous      = hyper_params["synchronous"]
    action_smoothing = hyper_params["action_smoothing"]
    reward_fn        = hyper_params["reward_fn"]
    seed             = hyper_params["seed"]
    eval_interval    = hyper_params["eval_interval"]
    record_eval      = hyper_params["record_eval"]
    eval_time        = hyper_params["eval_time"]
    target_std       = hyper_params["target_std"]

    map              = sim_params["episodes"]["map"]
    num_episodes     = sim_params["episodes"]["num_episodes"]
    fps              = sim_params["top_view"]["fps"]
    ego_num          = sim_params["vehicles"]["ego_vehicle_num"]
    reset            = sim_params["episodes"]["episode_reset"]
    vehicle_agent    = sim_params["vehicles"]["vehicle_agent"]

    # Set seeds
    if isinstance(seed, int):
        tf.random.set_random_seed(seed)
        np.random.seed(seed)
        random.seed(0)

    # Define primeiro episódio independente de ser continuação, para não gravar o modelo novamente no início
    First_Episode = True

    # Load VAE
    #vae = load_vae(vae_model, vae_z_dim, vae_model_type)

    # Override params for logging
    #params["vae_z_dim"] = vae.z_dim
    #params["vae_model_type"] = "mlp" if isinstance(vae, MlpVAE) else "cnn"

    mlflow.set_experiment("CarAware training")
    with mlflow.start_run() as run:
        # Log parameters
        log_params_recursive(sim_params)
        log_params_recursive(train_params)
        log_params_recursive(sens_params)

        print("")
        print("Training parameters:")
        for k, v, in hyper_params.items(): print(f"  {k}: {v}")
        print("")

        print("")
        print("Simulation parameters:")
        for k, v, in sim_params.items(): print(f"  {k}: {v}")
        print("")

        # Create state encoding fn
        #measurements_to_include = set(["steer", "throttle", "speed"])
        #encode_state_fn = create_encode_state_fn(vae, measurements_to_include)

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
                    map=map,
                    last_positions_training=last_positions_training,
                    history_length=history_length)

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
        print("Model created")

        """ # Prompt to load existing model if any
        if not restart:
            if os.path.isdir(model.log_dir) and len(os.listdir(model.log_dir)) > 0:
                answer = input("Model \"{}\" already exists. Do you wish to continue (C) or restart training (R)? ".format(model_name))
                if answer.upper() == "C":
                    pass
                elif answer.upper() == "R":
                    restart = True
                else:
                    raise Exception("There are already log files for model \"{}\". Please delete it or change model_name and try again".format(model_name))
        """

        if restart:
            shutil.rmtree(model.model_dir)
            for d in model.dirs:
                os.makedirs(d)
        model.init_session()
        if not restart:
            if train_model.lower() == "latest":
                model.load_latest_checkpoint()
            else:  # Custom model
                model.load_custom_checkpoint(train_model)
        model.write_dict_to_summary("hyper-parameters", hyper_params, 0)
        model.write_dict_to_summary("simulation-parameters", sim_params, 0)

        episode_idx = model.get_episode_idx()+1
        simulation.episodio_atual = int(episode_idx/num_training)

        '''
        if restart == False:
            if os.path.exists(os.path.join(model.model_dir, "training_log.csv")):
                csv_file = open(os.path.join(model.model_dir, "training_log.csv"), "r")
                data = [line.strip().split(';')[0] for line in csv_file.readlines()]
                simulation.episodio_atual = int(data[-2])
            else:
                simulation.episodio_atual = 0
        '''
        # Faz a leitura da hora que terminou o último treinamento pra contabilizar o tempo total
        if os.path.exists(os.path.join(model.model_dir, "training_log.csv")) and restart == False:
            csv_file = open(os.path.join(model.model_dir, "training_log.csv"), "r")
            try:
                data = [line.strip().split(';') for line in csv_file.readlines()]
                #simulation.sim_last_total_time = 0
                simulation.sim_last_total_time = int(data[-1][3])
                if map == "Gradual_Random":
                    # último episódio foi do tipo Gradual Random, continua de onde parou
                    read_current_grad_random = int(data[-1][5])
                    read_init_grad_random = int(data[-1][6])
                    if read_current_grad_random == 999 or read_init_grad_random == 999:
                        print("Iniciando novo gradual random.")
                    else:
                        simulation.current_gradual_random_ep = read_current_grad_random
                        simulation.init_gradual_random_ep = read_init_grad_random
                #csv_file.close()
            except:
                #csv_file.close()
                simulation.sim_last_total_time = 0
        else:
            simulation.sim_last_total_time = 0  # Log de tempo total de treinamento em CSV

        # Configura se um novo CSV será criado ou um existente editado
        if os.path.exists(os.path.join(model.model_dir, "training_log.csv")):
            csv_file = open(os.path.join(model.model_dir, "training_log.csv"), "a", newline="")
        else:
            csv_file = open(os.path.join(model.model_dir, "training_log.csv"), "w", newline="")

        csv_writer = csv.writer(csv_file, delimiter=";")

        if restart:
            if map == "Gradual_Random":
                csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                    str(datetime.now().strftime("%H:%M:%S")), simulation.sim_total_time, "Start",
                                    simulation.current_gradual_random_ep, simulation.init_gradual_random_ep])
            else:
                csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                    str(datetime.now().strftime("%H:%M:%S")), simulation.sim_total_time, "Start", 999, 999])
        else:
            if map == "Gradual_Random":
                csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                    str(datetime.now().strftime("%H:%M:%S")), simulation.sim_total_time, "Continue",
                                    simulation.current_gradual_random_ep, simulation.init_gradual_random_ep])
            else:
                csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                    str(datetime.now().strftime("%H:%M:%S")), simulation.sim_total_time, "Continue", 999, 999])

        # eval_cont = 0 # conta se chegou no reward desejado N evaluations

        # Create prediction PUB socket
        context = zmq.Context()
        pred_socket = context.socket(zmq.PUB)
        pred_addr = f"tcp://*:5001"
        pred_socket.bind(pred_addr)
        print(f"Publishing predictions on {pred_addr}")

        # For every episode
        #while num_episodes <= 0 or model.get_episode_idx() < num_episodes:
        while num_episodes <= 0 or simulation.episodio_atual < num_episodes:
            print(f"Episode {simulation.episodio_atual}")
            # Espera simulação ser carregada para começar o treinamento
            while not simulation.simulation_status == "Ready":
                time.sleep(0.01)

            episode_idx = model.get_episode_idx() + 1
            simulation.episodio_atual = int(episode_idx / num_training)

            # Sinaliza começo do treinamento
            simulation.simulation_status = "Training"
            time.sleep(2) # espera primeira captura do ground_truth

            # Run evaluation periodically
            if simulation.episodio_atual % eval_interval == 0 and simulation.episodio_atual != 0 and First_Episode == False:
                print("Evaluating model")

                # Indica para o módulo top-view que está acontecendo a fase de evaluation
                simulation.eval = True

                video_filename = os.path.join(model.video_dir, "episode{}.avi".format(simulation.episodio_atual))
                eval_reward = run_eval(env, model, video_filename, eval_time, simulation, ego_num, pred_socket=pred_socket)
                model.write_value_to_summary("eval/reward", eval_reward, simulation.episodio_atual)
                # goal_cont = 0

                # Atualiza informações de reward que serão exibidas no HUD - Evaluation
                simulation.best_reward = best_eval_reward
                simulation.reward_atual = eval_reward

                # Registra log de avaliações e tempos
                """
                if (reset and simulation.simulation_reset and reset_mode == "Target"):  # Registra reset
                    csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()), datetime.now().strftime("%H:%M:%S"), "Reset"])
                else:
                    csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()), datetime.now().strftime("%H:%M:%S"), "Evaluation"])
                """
                if map == "Gradual_Random":
                    csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                        datetime.now().strftime("%H:%M:%S"), simulation.sim_total_time, "Evaluation",
                                        simulation.current_gradual_random_ep, simulation.init_gradual_random_ep])
                else:
                    csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                        datetime.now().strftime("%H:%M:%S"), simulation.sim_total_time, "Evaluation", 999, 999])

                # salva a cada evaluation interval ou se melhor reward for atingido
                #if eval_reward > best_eval_reward or episode_idx % (eval_interval*save_eval_interval):
                #if episode_idx % eval_interval*4:
                if eval_reward > best_eval_reward:
                    reason = "Best"
                    best_eval_reward = eval_reward
                else:
                    reason = "Interval"
                time_raw = simulation.sim_total_time
                sim_time_now = '{:02}_{:02}_{:02}'.format(time_raw // 3600, time_raw % 3600 // 60, time_raw % 60)
                model.save(simulation.episodio_atual, reason, sim_time_now)
                if map == "Gradual_Random":
                    csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                        datetime.now().strftime("%H:%M:%S"), simulation.sim_total_time, reason,
                                        simulation.current_gradual_random_ep, simulation.init_gradual_random_ep])
                else:
                    csv_writer.writerow(
                        [str(simulation.episodio_atual), datetime.date(datetime.now()), datetime.now().strftime("%H:%M:%S"),
                        simulation.sim_total_time, reason, 999, 999])

                # Indica para o módulo top-view que finalizou a fase de evaluation
                simulation.eval = False
            # Reset environment
            state, terminal_state, total_reward = env.reset(randomize_offset=randomize_offset)

            # While episode not done
            print(f"Training Episode {simulation.episodio_atual} (Step {model.get_train_step_idx()})")

            #while not terminal_state:
            for idx_training in range(num_training):
                episode_idx = model.get_episode_idx()+1
                First_Episode = False  # variável usado para evitar gravação de modelo à toa
                simulation.training_atual = idx_training
                states, taken_actions, values, rewards, dones = [], [], [], [], []
                current_veh = 0
                #if ego_num == 1:
                #    single_veh = 10
                    #single_veh = random.randint(0, 9)
                #else:
                #    single_veh = 10
                for idx_horizon in range(horizon):
                    # print(f"Horizon {idx_horizon}")
                    simulation.horizonte_atual = idx_horizon
                    #action_lst, value_lst = [], []
                    #for state, vehicle in zip(state_lst,simulation.ego_vehicle):  # Roda N vezes, para N veículos simulados

                    # Print CARLA and ROS observation states in a tabular format
                    # print(f"\t----------------------------------------------------------------")
                    # print(f"\t| Metric               | CARLA State       | ROS State         |")
                    # print(f"\t|----------------------|-------------------|-------------------|")
                    # print(f"\t| Position X           | {env.carla_state[0]:<17.2f} | {env.last_observation[0]:<17.2f} |")
                    # print(f"\t| Position Y           | {env.carla_state[1]:<17.2f} | {env.last_observation[1]:<17.2f} |")
                    # print(f"\t| Acceleration X       | {env.carla_state[2]:<17.2f} | {env.last_observation[2]:<17.2f} |")
                    # print(f"\t| Acceleration Y       | {env.carla_state[3]:<17.2f} | {env.last_observation[3]:<17.2f} |")
                    # print(f"\t| Acceleration Z       | {env.carla_state[4]:<17.2f} | {env.last_observation[4]:<17.2f} |")
                    # print(f"\t| Gyroscope X          | {env.carla_state[5]:<17.2f} | {env.last_observation[5]:<17.2f} |")
                    # print(f"\t| Gyroscope Y          | {env.carla_state[6]:<17.2f} | {env.last_observation[6]:<17.2f} |")
                    # print(f"\t| Gyroscope Z          | {env.carla_state[7]:<17.2f} | {env.last_observation[7]:<17.2f} |")
                    # print(f"\t| Compass Degrees      | {env.carla_state[8]:<17.2f} | {env.last_observation[8]:<17.2f} |")
                    # print(f"\t| Speed                | {env.carla_state[9]:<17.2f} | {env.last_observation[9]:<17.2f} |")
                    # print(f"\t| Steering Angle       | {env.carla_state[10]:<17.2f} | {env.last_observation[10]:<17.2f} |")
                    # print(f"\t----------------------------------------------------------------")

                    # print(f"\tNormalized state: {state}")
                    action, value = model.predict(state, write_to_summary=True)
                    # print(f"\tAction: {action}")

                    # Convert action to CARLA format
                    prediction = env.network_to_carla(action)
                    pred_socket.send_json({"prediction": prediction})
                    # print(f"\tPrediction: {prediction}")

                    # Perform action
                    #new_state_lst, reward_lst, terminal_state_lst = [], [], []
                    #for action, vehicle, veh_num in zip(action_lst,simulation.ego_vehicle, enumerate(simulation.ego_vehicle)):  # Roda N vezes, para N veículos simulados
                    new_state, reward, terminal_state = env.step(action, simulation.ego_vehicle[current_veh], current_veh)  # , single_veh)
                    #new_state_lst.append(new_state)
                    #reward_lst.append(reward)
                    #terminal_state_lst.append(terminal_state)
                    # print(f"\tReward: {reward}")

                    # gt = simulation.ego_vehicle[current_veh].get_location()
                    # print(f"Ground truth: {gt.x:.2f}, {gt.y:.2f}")
                    # print(f"Prediction: {prediction[0]:.2f}, {prediction[1]:.2f}")

                    total_reward += reward
                    #total_reward += np.mean(np.array(reward))

                    #print(action)

                    #for state1, action1, value1,reward1,terminal_state1,new_state1 in zip(state_lst, action_lst, value_lst,
                    #                                                                reward_lst, terminal_state_lst,
                    #                                                                new_state_lst):
                    states.append(state)         # [T, *input_shape]
                    taken_actions.append(action) # [T,  num_actions]
                    values.append(value)         # [T]
                    rewards.append(reward)       # [T]
                    dones.append(terminal_state) # [T]
                    state = new_state

                    #idx = 0
                    #if len(states) == horizon: # adiciona na lista apenas o número de horizons definido
                    #    break
                    #else:
                    #    idx+=1  # identifica ultimo estado

                    # Acrescenta Reward instantâneo no HUD
                    simulation.reward_inst = total_reward

                    # Lógica para voltar a ciclagem de treinamento para o primeiro veículo
                    if current_veh == ego_num-1:
                        current_veh = 0
                    else:
                        current_veh += 1

                    if terminal_state:
                        break

                    if top_view.input_control.quit:  # Evento tecla ESC ou crash detectados
                        break
                    #print("prediction: ", action)

                if top_view.input_control.quit:  # Evento tecla ESC ou crash detectados
                    break

                # Calculate last value (bootstrap value)
                _, last_values = model.predict(state, greedy=True)  # usa último estado gerado pelo último carro
                #print("last_values: ", last_values)
                #print("state: ", state)
                #print("values_ant: ", values)

                # Compute GAE
                advantages = compute_gae(rewards, values, last_values, dones, discount_factor, gae_lambda)
                returns = advantages + values

                # Normalize
                # returns = (returns - returns.mean()) / (returns.std() + 1e-8)
                # advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

                # Flatten arrays
                states        = np.array(states)
                taken_actions = np.array(taken_actions)
                returns       = np.array(returns)
                advantages    = np.array(advantages)

                T = len(rewards)
                #print("T: ", T)
                #print("states shape: ",states.shape)
                #print("input_shape: ", input_shape)  #*input_shape
                #print("input shape: ", input_shape)
                #print("taken actions: ", taken_actions.shape)
                #print("num actions: ", num_actions)
                #print("taken actions: ", taken_actions)

                assert states.shape == (T, input_shape)  # assert states.shape == (T, *input_shape)
                assert taken_actions.shape == (T, num_actions)
                assert returns.shape == (T,)
                assert advantages.shape == (T,)

                # Train for some number of epochs
                model.update_old_policy() # θ_old <- θ
                for _ in range(num_epochs):
                    num_samples = len(states)
                    indices = np.arange(num_samples)
                    np.random.shuffle(indices)
                    for i in range(int(np.ceil(num_samples / batch_size))):
                        # Sample mini-batch randomly
                        begin = i * batch_size
                        end   = begin + batch_size
                        if end > num_samples:
                            end = None
                        mb_idx = indices[begin:end]

                        # Optimize network
                        model.train(states[mb_idx], taken_actions[mb_idx],
                                    returns[mb_idx], advantages[mb_idx])

                # Write episodic values
                #training_atual = (simulation.episodio_atual*num_training) + idx_training
                model.write_value_to_summary("train/reward", total_reward, episode_idx)
                model.write_value_to_summary("train/distance", env.distance, episode_idx)
                model.write_episodic_summaries()

                if map == "Gradual_Random":
                    csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                        datetime.now().strftime("%H:%M:%S"), simulation.sim_total_time, "Episode",
                                        simulation.current_gradual_random_ep, simulation.init_gradual_random_ep])
                else:
                    csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                        datetime.now().strftime("%H:%M:%S"), simulation.sim_total_time, "Episode", 999, 999])

                mlflow.log_metric("train/std", model.current_std, step=episode_idx)
                mlflow.log_metric("train/loss/policy", model.pl, step=episode_idx)
                mlflow.log_metric("train/loss/value", model.vl, step=episode_idx)
                mlflow.log_metric("train/loss/entropy", model.el, step=episode_idx)
                mlflow.log_metric("train/loss/total", model.l, step=episode_idx)
                mlflow.log_metric("train/kl_divergence", model.kl, step=episode_idx)
                mlflow.log_metric("train/clip_ratio", model.clip, step=episode_idx)
                # mlflow.log_metric("train/action_mean_x", model.act_mean[0], step=episode_idx)
                # mlflow.log_metric("train/action_mean_y", model.act_mean[1], step=episode_idx)
                mlflow.log_metric("train/value_rmse", np.sqrt(np.mean(np.square(model.v - returns[mb_idx]))), step=episode_idx)
                mlflow.log_metric("train/advantage_mean", np.mean(advantages), step=episode_idx)
                mlflow.log_metric("train/advantage_std", np.std(advantages), step=episode_idx)

                # Finaliza simulação baseado no valor desejado de desvio padrão
                # print(model.current_std)
                # if model.current_std <= target_std:
                #     top_view.input_control.quit = True

            if top_view.input_control.quit == True:  # Evento tecla ESC ou crash detectados
                break

            if simulation.simulation_status != "Restart":
                simulation.simulation_status = "Done"

        #Salva último estado do modelo e faz log
        time_raw = simulation.sim_total_time
        sim_time_now = '{:02}_{:02}_{:02}'.format(time_raw // 3600, time_raw % 3600 // 60, time_raw % 60)
        model.save(simulation.episodio_atual, "closing", sim_time_now)
        if map == "Gradual_Random":
            csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                datetime.now().strftime("%H:%M:%S"), simulation.sim_total_time, "Closing",
                                simulation.current_gradual_random_ep, simulation.init_gradual_random_ep])
        else:
            csv_writer.writerow([str(simulation.episodio_atual), datetime.date(datetime.now()),
                                datetime.now().strftime("%H:%M:%S"), simulation.sim_total_time, "Closing", 999, 999])

        # mlflow.tensorflow.log_model(tf_saved_model_dir=model.model_dir, artifact_path="model")

        # Fecha arquivo de evaluations
        csv_file.close()

        # Treinamento finalizado
        simulation.simulation_status = "Complete"
