
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
from utils import absolute2relative, relative2absolute, degree2radian, visualize
if __name__ == "__main__":

    env = VrepEnvironment(speed=1.0, turn=0.5, rate=1)
    R = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
    scale_factor = 100
    floor_w = floor_h = 15*scale_factor

    while True:
        img = np.full((floor_w, floor_h, 3), 0.5)
        action = np.random.choice(2)


        transform, lidar_data = env.step(action=3)

        p = transform.translation
        #print(p)
        qua = transform.rotation
        scan = np.reshape(lidar_data, (270, -1))

        # algo 1
        # robot_pos = np.array((-p.x, -p.y))*scale_factor
        # robot_pos =  750 + robot_pos
        # robot_pos_xy_1 = (int(robot_pos[1]), int(robot_pos[0]))
        # algo 2
        robot_pos = np.array((p.x, p.y, p.z))*scale_factor
        robot_pos = R @ robot_pos + 750
        robot_pos_xy = (int(robot_pos[0]), int(robot_pos[1]))

        #assert robot_pos_xy_1 == robot_pos_xy_2
        robot_theta_w = wrapAngle(2 *np.arcsin(qua.z))
        #robot_theta = wrapAngle(-2 *np.arcsin(qua.z) - np.pi/2)
        robot_theta = -2 *np.arcsin(qua.z) - np.pi/2
        #print(wrapAngle(2 *np.arcsin(qua.z)))
        print(-2 *np.arcsin(qua.z) - np.pi/2)
        print(robot_theta)
        print(robot_pos_xy)
        # print(robot_pos_xy)
        # print(robot_theta)

        # print("{:.2f}".format(robot_theta))
        # if math.isclose(robot_theta, -3.14, abs_tol=0.01):
        #     robot_theta = 0
        #     robot_theta_w = -np.pi/2
        # if math.isclose(robot_theta_w, -3.14, abs_tol=0.01):
        #     robot_theta_w = np.pi/2
        robot_state = (p.x, p.y, robot_theta_w)
        #print(robot_state)
        # draw robot center
        cv2.circle(img, robot_pos_xy, 30, (0, 0, 255), 2)

        # draw center map
        cv2.circle(img, (750, 750), 4, (255, 0, 0), 2)
        for i in (500, 1000):
            cv2.line(img, (0, i), (1500, i), (0, 255, 0), 2)
            cv2.line(img, (i, 0), (i, 1500), (0, 255, 0), 2)



        # robot_pos = np.array((p.x, p.y, p.z))*scale_factor
        # robot_pos = R @ robot_pos + 750
        # robot_pos_xy = (int(robot_pos[0]), int(robot_pos[1]))

        #print(robot_pos_new_coord)

        for i in range(len(scan)):
            #print(end_ray.shape)
            end_ray_wcoord = relative2absolute((scan[i][0], scan[i][1]), robot_state)

            end_ray = np.array((end_ray_wcoord[0], end_ray_wcoord[1], 0))*scale_factor
            end_ray =R @ end_ray + 750
            end_ray_xy = (int(end_ray[0]), int(end_ray[1]))
            # end_ray = np.array((scan[i][0], scan[i][1], scan[i][2]))*scale_factor
            # end_ray = R @ end_ray + 750
            # end_ray_xy = (int(end_ray[0]), int(end_ray[1]))
            if i < 10:
                cv2.line(img, robot_pos_xy, end_ray_xy, (0, 0, 255), 2)
            elif i > 260:
                cv2.line(img, robot_pos_xy, end_ray_xy, (255, 0, 0), 2)
            else:
                cv2.line(img, robot_pos_xy, end_ray_xy, (0, 255, 0), 2)
            cv2.circle(img, end_ray_xy, 4, (128, 128, 0), 2)
        # draw orientation
        x = robot_pos[0] + np.cos(robot_theta)*30
        y = robot_pos[1] + np.sin(robot_theta)*30
        cv2.line(img, robot_pos_xy,(int(x), int(y)), (0, 0, 255), 2)

        img = cv2.resize(img, (750,750))
        #img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow("slam", img)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()
#cv2.detroyAllWindow()


# transform, scan = env.step(action=3)
# while True:
#     if input("Please enter a string:\n") == "w":
#         print("moving forward")
#         action = 1
#         transform, scan = env.step(action)
#         print(len(scan))
#
#     if input("Please enter a string:\n") == "a":
#         print("turning left")
#         action = 0
#         transform, scan = env.step(action)
#
#     if input("Please enter a string:\n") == "d":
#         print("turning right")
#         action = 2
#         transform, scan = env.step(action)
#     if input("Please enter a string:\n") == "q":
#         break
