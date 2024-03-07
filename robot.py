import random
import numpy as np
import math
from utils import *


class Robot(object):
    def __init__(self, x, y, theta, grid, config, sense_noise=None):
        # initialize robot pose
        self.x = x
        self.y = y
        self.theta = theta
        self.trajectory = []

        # map that robot navigates in
        # for particles, it is a map with prior probability
        self.grid = grid
        self.grid_size = self.grid.shape

        # probability for updating occupancy map
        self.prior_prob = config['prior_prob']
        self.occupy_prob = config['occupy_prob']
        self.free_prob = config['free_prob']

        # sensing noise for trun robot measurement
        self.sense_noise = sense_noise if sense_noise is not None else 0.0

        # parameters for beam range sensor
        self.num_sensors = config['num_sensors']
        self.radar_theta = np.arange(-135*2*np.pi/360, 135*2*np.pi/360, 5*2*np.pi/360)[::-1] #np.arange(-135*2*np.pi/360, 135*2*np.pi/360, 2*np.pi/360)[::-1] # (-135, 135, 1)
        #np.arange(0, self.num_sensors) * (2 * np.pi / self.num_sensors) + np.pi / self.num_sensors
        self.radar_length = config['radar_length']
        self.radar_range = config['radar_range']
        self.d = config['w_distance']
        self.RotationMatrix = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
        self.scale_factor = 10
    def set_states(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def get_state(self):
        return (self.x, self.y, self.theta)

    def update_trajectory(self):
        self.trajectory.append([self.x, self.y])

    def move(self, turn, forward):
        self.theta = self.theta + turn
        self.theta = wrapAngle(self.theta)

        self.x = self.x + forward * np.cos(self.theta)
        self.y = self.y + forward * np.sin(self.theta)
    def action2move(self, action, v_forward, v_turn, ros_rate):
        # https://answers.ros.org/question/231942/computing-odometry-from-two-velocities/
        if action == 0:
            v_left = v_forward - v_turn
            v_right = v_forward + v_turn
        elif action == 1:
            v_left = v_forward
            v_right = v_forward
        elif action == 2:
            v_left = v_forward + v_turn
            v_right = v_forward - v_turn

        v_rx = (v_right + v_left) / 2
        v_ry = 0
        omega_r = (v_right - v_left) / self.d
        v_wx = v_rx * np.cos(self.theta) - v_ry *np.sin(self.theta)
        v_wy = v_rx * np.sin(self.theta) + v_ry * np.cos(self.theta)
        thetadot = omega_r
        self.x = self.x + v_wx * 1
        self.y = self.y + v_wy * 1
        self.theta = self.theta + thetadot * 1
        self.theta = wrapAngle(self.theta) + np.pi

    def sense(self, lidar_data=None, robot_state=None):
        if lidar_data is None:
            measurements, free_grid, occupy_grid = self.ray_casting(lidar_data)
            measurements = np.clip(measurements + np.random.normal(0.0, self.sense_noise, self.num_sensors), 0.0, self.radar_range)
        else:
            measurements, free_grid, occupy_grid = self.ray_casting_realdata(lidar_data, robot_state)

        return measurements, free_grid, occupy_grid

    def build_radar_beams(self):
        radar_src = np.array([[self.x] * self.num_sensors, [self.y] * self.num_sensors])
        radar_theta = self.radar_theta + self.theta
        radar_rel_dest = np.stack(
            (
                np.cos(radar_theta) * self.radar_length,
                np.sin(radar_theta) * self.radar_length
            ), axis=0
        )

        radar_dest = radar_rel_dest + radar_src

        beams = [None] * self.num_sensors
        for i in range(self.num_sensors):
            x1, y1 = radar_src[:, i]
            x2, y2 = radar_dest[:, i]
            beams[i] = bresenham(x1, y1, x2, y2, self.grid_size[0], self.grid_size[1])

        return beams

    def build_radar_beams_realdata(self, lidar_data=None, robot_state=None):
        radar_src = np.array([[self.x] * self.num_sensors, [self.y] * self.num_sensors])
        radar_theta = self.radar_theta + self.theta
        # radar_rel_dest = np.stack(
        #     (
        #         np.cos(radar_theta) * self.radar_length,
        #         np.sin(radar_theta) * self.radar_length
        #     ), axis=0
        # )
        #
        # radar_dest = radar_rel_dest + radar_src

        beams = [None] * self.num_sensors
        for i in range(self.num_sensors):
            x1, y1 = radar_src[:, i]
            end_ray_wcoord = relative2absolute((lidar_data[5*i][0], lidar_data[5*i][1]), robot_state)
            end_ray = np.array((end_ray_wcoord[0], end_ray_wcoord[1], 0))*self.scale_factor
            #end_ray =self.RotationMatrix @ end_ray + 75
            end_ray = end_ray + 75
            x2, y2 = (int(end_ray[0]), int(end_ray[1]))
            #x2, y2 = radar_dest[:, i]
            beams[i] = bresenham(x1, y1, x2, y2, self.grid_size[0], self.grid_size[1])

        return beams


    def ray_casting(self, lidar_data=None):

        beams = self.build_radar_beams()

        loc = np.array([self.x, self.y])
        measurements = [self.radar_range] * self.num_sensors
        free_grid, occupy_grid = [], []

        for i, beam in enumerate(beams):
            dist = np.linalg.norm(beam - loc, axis=1)
            beam = np.array(beam)

            obstacle_position = np.where(self.grid[beam[:, 1], beam[:, 0]] >= 0.9)[0]
            if len(obstacle_position) > 0:
                idx = obstacle_position[0]
                occupy_grid.append(list(beam[idx]))
                free_grid.extend(list(beam[:idx]))
                measurements[i] = dist[idx]
            else:
                free_grid.extend(list(beam))

        return measurements, free_grid, occupy_grid

    def ray_casting_realdata(self, lidar_data=None, robot_state=None):

        beams = self.build_radar_beams_realdata(lidar_data, robot_state)

        loc = np.array([self.x, self.y])
        measurements = [self.radar_range] * self.num_sensors
        free_grid, occupy_grid = [], []
        for i, beam in enumerate(beams):
            dist = np.linalg.norm(beam - loc, axis=1)
            beam = np.array(beam)
            # robot_pos_w = ((np.array((self.x, self.y, 0)) - 75) / self.scale_factor
            # robot_theta_w = - (self.theta + np.pi/2)
            # robot_state_1 = np.array((robot_pos_w[0], robot_pos_w[1], robot_theta_w))
            # print("robot state", robot_state)
            # print("robot state reverse", robot_state_1)
            #print(robot_state)
            end_ray_wcoord = relative2absolute((lidar_data[i][0], lidar_data[i][1]), robot_state)
            end_ray = np.array((end_ray_wcoord[0], end_ray_wcoord[1], 0))*self.scale_factor
            end_ray =end_ray + 75
            #end_ray_xy = (int(end_ray[0]), int(end_ray[1]))
            x2, y2 = (int(end_ray[0]), int(end_ray[1]))

            #print("end_ray", end_ray_xy)
            #print("beam shape",beam.shape)
            #x2, y2 = end_ray_wcoord[0], end_ray_wcoord[1]

            ray_length = np.linalg.norm(np.array([x2, y2]) - loc)
            if ray_length < self.radar_range:
                free_grid.extend(list(beam))
                occupy_grid.append(list([x2, y2]))
                measurements[i] = ray_length
            else:
                free_grid.extend(list(beam))
            # obstacle_position = np.where(beam[:, (1, 0)] == end_ray_xy)
            # print(obstacle_position)
            # idx = obstacle_position[1][0]
            # print("beam", beam[idx, (1, 0)])
            #print(np.where(beam[:, (1, 0)] == end_ray_xy))
            # print(idx)
            # if False: #len(obstacle_position) > 0:
            #     idx = obstacle_position[0]
            #     occupy_grid.append(list(beam[idx]))
            #     free_grid.extend(list(beam[:idx]))
            #     measurements[i] = dist[idx]
            # else:
            #     free_grid.extend(list(beam))

        return measurements, free_grid, occupy_grid

    def update_occupancy_grid(self, free_grid, occupy_grid):
        mask1 = np.logical_and(0 < free_grid[:, 0], free_grid[:, 0] < self.grid_size[1])
        mask2 = np.logical_and(0 < free_grid[:, 1], free_grid[:, 1] < self.grid_size[0])
        free_grid = free_grid[np.logical_and(mask1, mask2)]

        inverse_prob = self.inverse_sensing_model(False)
        l = prob2logodds(self.grid[free_grid[:, 1], free_grid[:, 0]]) + prob2logodds(inverse_prob) - prob2logodds(self.prior_prob)
        self.grid[free_grid[:, 1], free_grid[:, 0]] = logodds2prob(l)

        mask1 = np.logical_and(0 < occupy_grid[:, 0], occupy_grid[:, 0] < self.grid_size[1])
        mask2 = np.logical_and(0 < occupy_grid[:, 1], occupy_grid[:, 1] < self.grid_size[0])
        occupy_grid = occupy_grid[np.logical_and(mask1, mask2)]

        inverse_prob = self.inverse_sensing_model(True)
        l = prob2logodds(self.grid[occupy_grid[:, 1], occupy_grid[:, 0]]) + prob2logodds(inverse_prob) - prob2logodds(self.prior_prob)
        self.grid[occupy_grid[:, 1], occupy_grid[:, 0]] = logodds2prob(l)

    def inverse_sensing_model(self, occupy):
        if occupy:
            return self.occupy_prob
        else:
            return self.free_prob
