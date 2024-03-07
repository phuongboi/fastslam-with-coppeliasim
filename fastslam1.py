from env import VrepEnvironment
import numpy as np
from sensor import Laser
import cv2
import time
from utils import relative2absolute, wrapAngle

import random
import copy
import os
import argparse
import yaml
import math

#from world import World
from robot import Robot
from world import World
from motion_model import MotionModel
from measurement_model import MeasurementModel
from utils import absolute2relative, relative2absolute, degree2radian, visualize, visualize_opencv
#import keyboard
if __name__ == "__main__":

    env = VrepEnvironment(speed=1, turn=0.5, rate=100)
    RotationMatrix = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
    scale_factor = 10
    floor_w = floor_h = 15*scale_factor

    with open("config.yaml", "r") as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    ROBOT = config['robot']
    SCENE = config['scene-1']
    NUMBER_OF_PARTICLES = 100

    # create an unknow map
    init_grid = np.ones(SCENE['grid_size']) * ROBOT['prior_prob']

    # init robot
    (x, y, theta) = SCENE['R_init']
    R = Robot(x, y, theta, init_grid, ROBOT, sense_noise=3.0)
    prev_odo = curr_odo = R.get_state()

    p = [None] * NUMBER_OF_PARTICLES
    (x, y, theta) = SCENE['p_init']

    for i in range(NUMBER_OF_PARTICLES):
        p[i] = Robot(x, y, degree2radian(theta), copy.deepcopy(init_grid), ROBOT)

    # create motion model
    motion_model = MotionModel(config['motion_model'])

    # create measurement model
    measurement_model = MeasurementModel(config['measurement_model'], ROBOT['radar_range'])
    output_path = "result/"

    idx = 0
    # create video recorder
    w = 600
    h = 300
    fps = 10
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    recorder = cv2.VideoWriter("result/map.mp4", fourcc, fps, (w, h))
    #while True:
    for i in range(100):
        #time.sleep(1)
        action = np.random.choice(2)
        #event = keyboard.read_event()
        #R.action2move(action, env.v_forward, env.v_turn, env.rate)
        curr_odo = R.get_state()
        R.update_trajectory()

        # if input("Please enter a string:\n") == "w":
        #     print("moving forward")
        #     action = 1
        # elif input("Please enter a string:\n") == "a":
        #     print("turning left")
        #     action = 0
        #
        # elif input("Please enter a string:\n") == "d":
        #     print("turning right")
        #     action = 2

        print("take action", action)
        transform, lidar_data = env.step(action=action)

        pos = transform.translation
        qua = transform.rotation
        robot_pos = np.array((pos.x, pos.y, pos.z))*scale_factor
        #robot_pos = RotationMatrix @ robot_pos + 75
        robot_pos = robot_pos + 75
        robot_pos_xy = (int(robot_pos[0]), int(robot_pos[1]))
        # robot_theta_w = 2 *np.arcsin(qua.z)
        # robot_theta = -2 *np.arcsin(qua.z) - np.pi/2
        robot_theta_w =qua.z
        #robot_theta = -2 *np.arcsin(qua.z) - np.pi/2
        R.x, R.y, R.theta = (int(robot_pos[0]), int(robot_pos[1]), robot_theta_w)

        # print("robot_pos", robot_pos_xy)
        # print("theta", robot_theta)
        # print("result from velocity")
        print(R.x, R.y, R.theta)
        robot_state = (pos.x, pos.y, robot_theta_w)

        scan = np.reshape(lidar_data, (270, -1))
        z_star, free_grid_star, occupy_grid_star = R.sense(lidar_data=scan, robot_state=robot_state)

        free_grid_offset_star = absolute2relative(free_grid_star, curr_odo)
        occupy_grid_offset_star = absolute2relative(occupy_grid_star, curr_odo)
        w = np.zeros(NUMBER_OF_PARTICLES)
        for i in range(NUMBER_OF_PARTICLES):
            prev_pose = p[i].get_state()
            x, y, theta = motion_model.sample_motion_model(prev_odo, curr_odo, prev_pose)
            p[i].set_states(x, y, theta)
            p[i].update_trajectory()

            # Calculate particle's weights depending on robot's measurement
            z, _, _ = p[i].sense()
            w[i] = measurement_model.measurement_model(z_star, z)

            # Update occupancy grid based on the true measurements
            curr_pose = p[i].get_state()
            free_grid = relative2absolute(free_grid_offset_star, curr_pose).astype(np.int32)
            occupy_grid = relative2absolute(occupy_grid_offset_star, curr_pose).astype(np.int32)
            p[i].update_occupancy_grid(free_grid, occupy_grid)

        # normalize
        w = w / np.sum(w)
        best_id = np.argsort(w)[-1]

        # select best particle
        estimated_R = copy.deepcopy(p[best_id])

        # Resample the particles with a sample probability proportional to the importance weight
        # Use low variance sampling method
        new_p = [None] * NUMBER_OF_PARTICLES
        J_inv = 1 / NUMBER_OF_PARTICLES
        r = random.random() * J_inv
        c = w[0]

        i = 0
        for j in range(NUMBER_OF_PARTICLES):
            U = r + j * J_inv
            while (U > c):
                i += 1
                c += w[i]
            new_p[j] = copy.deepcopy(p[i])

        p = new_p
        prev_odo = curr_odo
        print("vis")
        #estimated_R = copy.deepcopy(R)
        # print(p)
        # print(R)
        # print(free_grid_star)
        visualize_opencv(R, p, estimated_R, free_grid_star, idx, "FastSLAM 1.0", output_path, recorder)
        idx += 1
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
