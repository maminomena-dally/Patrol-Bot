import math
import random

import config
from robot.kinematics import normalize_angle


class LandmarkDetector:
    def __init__(self, robot, landmarks,
                 detection_radius=config.LANDMARK_DETECTION_RADIUS,
                 noise_std_distance=config.LANDMARK_NOISE_STD_DISTANCE,
                 noise_std_angle=config.LANDMARK_NOISE_STD_ANGLE):
        self.robot = robot
        self.landmarks = landmarks  # ex: [{"id": 0, "x": 2.0, "y": 3.0}, ...]
        self.detection_radius = detection_radius
        self.noise_std_distance = noise_std_distance
        self.noise_std_angle = noise_std_angle

    def detect(self):
        x, y, theta = self.robot.get_true_pose()
        detections = []

        for landmark in self.landmarks:
            dx = landmark["x"] - x
            dy = landmark["y"] - y
            true_distance = math.hypot(dx, dy)

            if true_distance > self.detection_radius:
                continue

            true_angle = normalize_angle(math.atan2(dy, dx) - theta)

            measured_distance = max(0.0, true_distance + random.gauss(0.0, self.noise_std_distance))
            measured_angle = normalize_angle(true_angle + random.gauss(0.0, self.noise_std_angle))

            detections.append({
                "id": landmark["id"],
                "x": landmark["x"],
                "y": landmark["y"],
                "distance": measured_distance,
                "angle": measured_angle,
            })

        return detections
