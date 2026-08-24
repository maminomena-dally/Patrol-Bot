import math

import config
from robot.kinematics import Pose, normalize_angle


class Localizer:
    def __init__(self, initial_pose, wheel_base=config.WHEEL_BASE,
                 process_noise=config.LOCALIZATION_PROCESS_NOISE,
                 measurement_noise=config.LOCALIZATION_MEASUREMENT_NOISE):
        x, y, theta = initial_pose
        self.estimated_pose = Pose(x, y, theta)
        self.wheel_base = wheel_base
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.uncertainty = 0.0  # m — grandit avec predict(), diminue avec correct()

    def predict(self, d_left, d_right):
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        mid_theta = self.estimated_pose.theta + d_theta / 2.0
        new_x = self.estimated_pose.x + d_center * math.cos(mid_theta)
        new_y = self.estimated_pose.y + d_center * math.sin(mid_theta)
        new_theta = normalize_angle(self.estimated_pose.theta + d_theta)

        self.estimated_pose = Pose(new_x, new_y, new_theta)
        self.uncertainty += self.process_noise * (abs(d_center) + abs(d_theta))

        return self.estimated_pose

    def correct(self, landmark_measurements):
        if not landmark_measurements:
            return self.estimated_pose

        x_estimates = []
        y_estimates = []
        for m in landmark_measurements:
            bearing_world = self.estimated_pose.theta + m["angle"]
            x_estimates.append(m["x"] - m["distance"] * math.cos(bearing_world))
            y_estimates.append(m["y"] - m["distance"] * math.sin(bearing_world))

        x_from_landmarks = sum(x_estimates) / len(x_estimates)
        y_from_landmarks = sum(y_estimates) / len(y_estimates)

        weight = self.uncertainty / (self.uncertainty + self.measurement_noise)
        weight = max(0.0, min(1.0, weight))

        new_x = (1 - weight) * self.estimated_pose.x + weight * x_from_landmarks
        new_y = (1 - weight) * self.estimated_pose.y + weight * y_from_landmarks

        self.estimated_pose = Pose(new_x, new_y, self.estimated_pose.theta)
        self.uncertainty = (1 - weight) * self.uncertainty

        return self.estimated_pose
