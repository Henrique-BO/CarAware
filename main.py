import argparse
import time
import carla
import logging
import cv2
import numpy as np
import math
import yaml

import simulation.simulation as simulation
import rl.train as train_RL
import rl.play as play_RL

from threading import Thread

# Configure logging
import tensorflow as tf
tf.compat.v1.logging.set_verbosity(tf.compat.v1.logging.ERROR)
np.set_printoptions(threshold=np.inf)
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')


def parse_args():
    parser = argparse.ArgumentParser(description="CarAware Simulation Framework")

    # Define subcommands
    subparsers = parser.add_subparsers(dest="command", required=True, help="Subcommands: simulation, train, play")

    # Common arguments for all subcommands
    for subcommand in ["simulation", "train", "play"]:
        subparser = subparsers.add_parser(subcommand, help=f"{subcommand.capitalize()} mode")
        subparser.add_argument('--simulation', type=str, default='config/simulation.yaml', help='Path to the main configuration file')
        subparser.add_argument('--sensors', type=str, default='config/sensors.yaml', help='Path to the sensors configuration file')
        subparser.add_argument('--training', type=str, default='config/training.yaml', help='Path to the hyperparameters file')
        subparser.add_argument('--override', type=str, nargs='*', help='Override specific configurations (e.g., key1=value1 key2=value2)')

    return parser.parse_args()

def load_config(simulation_config, sensor_config, train_config=None):
    # Load the simulation configuration
    with open(simulation_config, 'r') as sim_file:
        SIM_PARAMS = yaml.safe_load(sim_file)

    # Load the sensors configuration
    with open(sensor_config, 'r') as sens_file:
        SENS_PARAMS = yaml.safe_load(sens_file)

    # Load the training configuration if provided
    TRAIN_PARAMS = {}
    if train_config:
        with open(train_config, 'r') as train_file:
            TRAIN_PARAMS = yaml.safe_load(train_file)

    return SIM_PARAMS, SENS_PARAMS, TRAIN_PARAMS

def apply_overrides(SIM_PARAMS, SENS_PARAMS, TRAIN_PARAMS, overrides):
    """Apply command-line overrides to the configuration dictionary."""
    if overrides:
        for override in overrides:
            key, value = override.split('=')

            # Define which dictionary to update based on the key
            if key.startswith("simulation."):
                config = SIM_PARAMS
                key = key[len("simulation."):]
            elif key.startswith("sensors."):
                config = SENS_PARAMS
                key = key[len("sensors."):]
            elif key.startswith("training."):
                config = TRAIN_PARAMS
                key = key[len("training."):]
            else:
                raise ValueError(f"Invalid override key: {key}")

            # Convert value to int, float, or keep as string
            try:
                value = eval(value)
            except:
                pass
            keys = key.split('.')  # Support nested keys (e.g., "SIM_PARAMS.MAP")
            d = config
            for k in keys[:-1]:
                d = d.setdefault(k, {})
            d[keys[-1]] = value
    return SIM_PARAMS, SENS_PARAMS, HYPER_PARAMS

# ===== PROGRAMA PRINCIPAL =====
def main(TRAIN_MODE, SIM_PARAMS, SENS_PARAMS, TRAIN_PARAMS):
    logging.info("Starting main function...")

    # UNPACK DAS VIARIÁVEIS UTILIZADAS NESSE PROGRAMA
    episodes_config = SIM_PARAMS["episodes"]
    vehicles_config = SIM_PARAMS["vehicles"]
    top_view_config = SIM_PARAMS["top_view"]

    MAP = episodes_config["map"]
    EPISODE_RESET = episodes_config["episode_reset"]
    NUM_EPISODES = episodes_config["num_episodes"]
    RESET_INTERVAL = episodes_config["reset_interval"]
    KALMAN_FILTER = episodes_config["kalman_filter"]

    EGO_VEHICLE_NUM = vehicles_config["ego_vehicle_num"]
    NPC_VEHICLE_NUM = vehicles_config["npc_vehicle_num"]
    STATIC_PROPS_NUM = vehicles_config["static_props_num"]
    PEDESTRIAN_NUM = vehicles_config["pedestrian_num"]
    VEHICLE_AGENT = vehicles_config["vehicle_agent"]
    NUM_MIN_WAYPOINTS = vehicles_config["num_min_waypoints"]

    DEBUG = top_view_config["debug"]
    TOP_VIEW_SHOW_HUD = top_view_config["show_hud"]

    SENS_GNSS = SENS_PARAMS["gnss"]["enabled"]
    SENS_IMU = SENS_PARAMS["imu"]["enabled"]
    SENS_SPD_SAS = SENS_PARAMS["spd_sas"]["enabled"]
    SENS_OBS = SENS_PARAMS["obstacle"]["enabled"]
    SENS_COL = SENS_PARAMS["collision"]["enabled"]
    SENS_RGB = SENS_PARAMS["rgb_camera"]["enabled"]
    SENS_RGB_PREVIEW = SENS_PARAMS["rgb_camera"]["preview"]
    SENS_LIDAR = SENS_PARAMS["lidar"]["enabled"]

    PREDICTION_HUD = TRAIN_PARAMS.get("prediction_hud", False)
    HORIZON = TRAIN_PARAMS["hyperparameters"]["horizon"]
    NUM_TRAINING = TRAIN_PARAMS["hyperparameters"]["num_training"]

    logging.info("Simulation parameters and sensor configurations unpacked.")

    # INICIALIZA AS CLASSES DA SIMULAÇÃO
    sim = simulation.SimulationSetup(SIM_PARAMS, SENS_PARAMS, TRAIN_PARAMS)  # Classe com setup da simulação
    sim.simulation_status = "Loading"  # Informa que a simulação está sendo carregada
    logging.info("Simulation setup initialized.")

    sim_pause = simulation.SimPause()  # Classe que pausa/resume a simulação
    sim_pause.start(sim)
    sim_pause.pause(sim)  # pausa a simulação para configuração do primeiro episódio

    top_view = simulation.TopView(SIM_PARAMS, TRAIN_PARAMS)  # Classe que abre a janela de Top View
    if MAP == "Random" or MAP == "Gradual_Random":
        top_view.start(sim.chosen_random_map)
    else:
        top_view.start(MAP)
    #top_view.start()

    if TRAIN_MODE == "Train":  # inicializa thread p/ treinamento RL
        logging.info("Starting training mode...")
        trainer_thread = Thread(target=train_RL.train, args=(TRAIN_PARAMS, SIM_PARAMS, sim, top_view), daemon=True)
        trainer_thread.start()
    elif TRAIN_MODE == "Play":  # inicializa thread p/ preview de modelo treinado RL
        logging.info("Starting play mode...")
        trainer_thread = Thread(target=play_RL.play, args=(TRAIN_PARAMS, SIM_PARAMS, sim, top_view), daemon=True)
        # trainer_thread.start()
        sim.simulation_status = "Play_Loading"
    elif TRAIN_MODE == "Simulation":
        logging.info("Starting simulation mode...")
        sim.simulation_status = "Simulation"

    time.sleep(5)

    # PREENCHE STRING SENSORES COM OS QUE ESTÃO HABILITADOS P/ MOSTRAR NO HUD
    sensores = ""
    sensores += "GNSS " if SENS_GNSS else ""
    sensores += "IMU " if SENS_IMU else ""
    sensores += "SPD/SAS " if SENS_SPD_SAS else ""
    sensores += "OBS " if SENS_OBS else ""
    sensores += "COL " if SENS_COL else ""
    sensores += "RGB " if SENS_RGB else ""
    sensores += "LIDAR " if SENS_LIDAR else ""

    num_restarts = 0  # contabiliza número de vezes que reiniciou
    sim_start_time = time.time()

    First_episode = True  # Faz o spawn no primeiro episódio simulado
    while not sim.simulation_status == "Complete":
        logging.info(f"Starting episode {sim.episodio_atual}...")
    #for episode_num in range(EPISODE_TOTAL):

        sim.new_episode = True
        #simulation.episodio_atual +=1

        #else:
        #    top_view.start(MAP)

        # Determina se o número de episódios é infinito ou não
        if NUM_EPISODES == 0:
            num_episodes = math.inf
        else:
            num_episodes = str(NUM_EPISODES)

        # INICIALIZA O EPISÓDIO "EPISODE_NUM"
        # registra os eventos em formato de log
        print("\n======= Iniciando episódio", sim.episodio_atual, "DE", num_episodes, "=======\n")
        if MAP == "Random" or MAP == "Gradual_Random":
            print("Mapa selecionado: ", sim.chosen_random_map)

        # CARREGA PEDESTRES, VEÍCULOS E OBJETOS
        if (EPISODE_RESET and sim.episodio_atual % RESET_INTERVAL == 0 and sim.episodio_atual != 0) or First_episode == True or sim.simulation_reset == True:
            logging.info("Spawning all entities for the episode...")
            sim.spawn_all()
            sim.simulation_reset = False
            First_episode = False

        if TRAIN_MODE == "PLAY":
            top_view.tick([], sim.ego_vehicle)  # atualiza a exibição do top-view
            trainer_thread.start()

        # CONFIGURA O EXPECTADOR PARA VISÃO DE CIMA NO SERVIDOR
        spectator = sim.world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(105.462921, 96.121056, 129.900925),
                                                carla.Rotation(-59.038227, 90.226158, 0.001122)))

        #top_view.world.ground_truth()  # gera os dados de GT para o RL

        # SINALIZA SIMULAÇÃO OK P/ TREINAMENTO COMEÇAR
        if sim.simulation_status != "Play_Loading" and sim.simulation_status != "Simulation":
            sim.simulation_status = "Ready"
            # registra os eventos em formato de log
            print("\nEpisódio Iniciado - Rodando por", str(HORIZON), "horizontes")

        logging.info("Resuming simulation...")
        sim_pause.resume(sim, SIM_PARAMS)  # RESUME A SIMULAÇÃO APÓS A CONFIGURAÇÃO

        # Game loop
        ep_start_time = time.time()
        logging.info("Entering game loop...")

        # Espera treinamento começar
        while not (sim.simulation_status == "Training" or sim.simulation_status == "Play" or \
                sim.simulation_status == "Simulation"):
            time.sleep(0.01)

        # while time.time() <= ep_start_time + EPISODE_TIME:
        while sim.simulation_status == "Training" or sim.simulation_status == "Play" or \
                sim.simulation_status == "Simulation":

            # Lógica para finalizar com ESC no modo simulação
            if sim.simulation_status == "Simulation" and top_view.input_control.quit :
                logging.info("Simulation quit detected. Exiting...")
                sim.simulation_status = "Complete"
                break

            if KALMAN_FILTER: #and sim.eval:
                top_view.world.kalman_filter = True
            else:
                top_view.world.kalman_filter = False

            #top_view.world.ground_truth()  # gera os dados de GT para o RL
            hud_txt = []

            if TOP_VIEW_SHOW_HUD:  # mostra dados no HUD do modo Top-view
                sim.sim_total_time = round((time.time() - sim_start_time)+sim.sim_last_total_time)
                hud_txt.append("SIMULAÇÃO_Modo: %s;Episódio: %s / %s;Restarts: %s;Treinamento: %s / %s;Horizonte: %s / %s ;Tempo de simulação: %s;Tempo do episódio: %s s;"
                           "Número de carros EGO: %s;Número de carros NPC: %s;Número de obstáculos: %s;Número de pedestres %s;Sensores: %s"
                           % (
                               str(sim.simulation_status),
                               str(sim.episodio_atual), num_episodes,
                               num_restarts,
                               str(sim.training_atual+1), str(NUM_TRAINING),
                               sim.horizonte_atual + 1, HORIZON,
                               '{:02}:{:02}:{:02}'.format(sim.sim_total_time // 3600, sim.sim_total_time % 3600 // 60, sim.sim_total_time % 60),
                               #str(format(time.time() - sim_start_time, ".2f")),
                               str(round(time.time() - ep_start_time)),
                               str(EGO_VEHICLE_NUM), str(NPC_VEHICLE_NUM), str(STATIC_PROPS_NUM),
                               str(PEDESTRIAN_NUM), sensores))

            if PREDICTION_HUD:  # exibe informações de prediction/reward no HUD

                # Informações de rewards
                predictions = "PREDICTIONS_Reward Eval. Max.: %s;Reward Eval. Atual: %s;Reward Instantâneo: %s;" % (str('{:.2f}'.format(sim.best_reward)),
                                                                                   str('{:.2f}'.format(sim.reward_atual)), str('{:.2f}'.format(sim.reward_inst)))

                # Informações de predictions
                for idx, veh in zip(range(len(sim.ego_vehicle)), sim.ego_vehicle):
                    if veh.pred_distance is not None:
                        predictions += "Carro " + str(idx+1) + ": " + str('{:.2f}'.format(veh.pred_distance)) + ";"

                        #predictions += "CARRO " + str(idx) + ": [" + str(int(veh.prediction[0])) + ", " + \
                        #           str(int(veh.prediction[1])) + ", " + str(int(veh.prediction[2])) + "];"
                hud_txt.append(predictions)

            # ============ CONTROLE DA SIMULAÇÃO DOS EGO VEHICLES ============
            if DEBUG or sim.simulation_status == "Play":  # Exibe apenas se modo DEBUG estiver habilitado
                car_info = "SENSORES_"
            for veh in sim.ego_vehicle:
                # CONTROLE DE COMPORTAMENTO DA DIREÇÃO AUTOMÁTICA
                if VEHICLE_AGENT == "BASIC":
                    control = veh.agent.run_step()
                    control.manual_gear_shift = False
                    veh.apply_control(control)
                elif VEHICLE_AGENT == "BEHAVIOR":
                    #veh.agent.update_information()
                    #if len(veh.agent.get_local_planner().waypoints_queue) < NUM_MIN_WAYPOINTS:
                        #veh.agent.reroute(simulation.veh_spawn_points)
                    speed_limit = veh.get_speed_limit()
                    veh.agent.get_local_planner().set_speed(speed_limit)
                    control = veh.agent.run_step()
                    veh.apply_control(control)

                # MOSTRA DADOS DOS VEÍCULOS NO HUD, CASO TOP-VIEW ESTEJA HABILITADO
                if TOP_VIEW_SHOW_HUD:  # mostra dados no HUD do modo Top-view
                    if DEBUG or sim.simulation_status == "Play":  # habilita exibição das leituras dos sensores no hud (reduz FPS)
                        try:
                            car_info += 'CARRO "%s";' % (veh.attributes["role_name"])
                            car_info += "Velocidade: %13.0f km/h;" % (veh.sens_spd_sas_speed)
                            car_info += "Âng. Volante: %17.2fº;" % (veh.sens_spd_sas_angle)
                            if SENS_IMU and veh.sens_imu is not None:
                                car_info += "Direção: %17.2fº %2s;" % (
                                    veh.sens_imu.ue_compass_degrees, veh.sens_imu.ue_compass_heading)
                                car_info += "Acelerômetro:  (%4.1f,%4.1f,%4.1f);" % (veh.sens_imu.ue_accelerometer[0],
                                                                             veh.sens_imu.ue_accelerometer[1],
                                                                             veh.sens_imu.ue_accelerometer[2])
                                car_info += "Giroscópio:    (%4.1f,%4.1f,%4.1f);" % (veh.sens_imu.ue_gyroscope[0],
                                                                             veh.sens_imu.ue_gyroscope[1],
                                                                             veh.sens_imu.ue_gyroscope[2])
                        except:
                            pass

                    if SENS_RGB and veh.sens_rgb is not None:
                        try:
                            #car_info += "Detecção da câmera: %s;" % veh.sens_rgb_objid
                            if SENS_RGB_PREVIEW:
                                cv2.imshow("CARRO " + str(veh.attributes["role_name"]), veh.sens_rgb_data)
                                cv2.waitKey(1)
                        except:
                            pass

            # ============ CONTROLE DA SIMULAÇÃO DOS NPC VEHICLES ============
            for veh in sim.npc_vehicle:
                if VEHICLE_AGENT == "BASIC":
                    control = veh.agent.run_step()
                    control.manual_gear_shift = False
                    veh.apply_control(control)
                elif VEHICLE_AGENT == "BEHAVIOR":
                    veh.agent.update_information()
                    if len(veh.agent.get_local_planner().waypoints_queue) < NUM_MIN_WAYPOINTS:
                        veh.agent.reroute(sim.veh_spawn_points)
                    speed_limit = veh.get_speed_limit()
                    veh.agent.get_local_planner().set_speed(speed_limit)
                    control = veh.agent.run_step()
                    veh.apply_control(control)
            if TOP_VIEW_SHOW_HUD:  # mostra dados no HUD do modo Top-view
                if DEBUG or sim.simulation_status == "Play":
                    hud_txt.append(car_info)
            top_view.tick(hud_txt, sim.ego_vehicle)  # atualiza a exibição do top-view

        logging.info(f"Episode {sim.episodio_atual} completed.")

        sim_pause.pause(sim)  # PAUSA A SIMULAÇÃO PARA CONFIGURAR O EPISÓDIO

        if MAP == "Gradual_Random":
            sim.current_gradual_random_ep = sim.current_gradual_random_ep - 1
            print(sim.current_gradual_random_ep)
            if sim.current_gradual_random_ep <= 0:
                sim.simulation_reset = True
                #top_view.start(sim.chosen_random_map)

        if (EPISODE_RESET and sim.episodio_atual % RESET_INTERVAL == 0 and sim.episodio_atual != 0) or sim.simulation_reset == True:
            sim.reset()
            # Inicia exibição de dados do mapa selecionado na tela de top-view
            num_restarts += 1
            if MAP == "Random" or MAP == "Gradual_Random":
                top_view.start(sim.chosen_random_map)


        if sim.simulation_status != "Complete":
            logging.info("Preparing for the next episode...")
            sim.simulation_status = "Loading"  # Segura o treinamento enquanto a simulação reinicia



        # registra os eventos em formato de log
        print("Episódio Finalizado")

    # APAGA OS OBJETOS DE SIMULAÇÃO CRIADOS
    del sim
    logging.info("Simulation finalized.")
    return top_view.input_control.quit_reason

if __name__ == '__main__':
    args = parse_args()
    SIM_PARAMS, SENS_PARAMS, HYPER_PARAMS = load_config(args.simulation, args.sensors, args.training)
    SIM_PARAMS, SENS_PARAMS, HYPER_PARAMS = apply_overrides(SIM_PARAMS, SENS_PARAMS, HYPER_PARAMS, args.override)

    logging.info("Program started.")
    result = main(args.command, SIM_PARAMS, SENS_PARAMS, HYPER_PARAMS)
    logging.info(f"Program finished with result: {result}")
    print(result)
