import random

import config


class Odometry:
    def __init__(self, robot, noise_std=config.ODOMETRY_NOISE_STD):
        self.robot = robot
        self.noise_std = noise_std

    def read(self, dt):
        vL, vR = self.robot.get_wheel_velocities()

        d_left = vL * dt + random.gauss(0.0, self.noise_std)
        d_right = vR * dt + random.gauss(0.0, self.noise_std)

        return d_left, d_right
