"""
tests/test_sensors.py — Tests des capteurs sensors/lidar.py et
sensors/cameras.py (Role 5 - Tino).

Lancer avec :
    python -m unittest discover -s tests -v
"""

import sys
import os
import math
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from robot.robot import Robot
from sensors.lidar import LidarSensor
from sensors.cameras import Camera, creer_cameras_robot


class TestLidarSensor(unittest.TestCase):
    def test_aucun_obstacle_renvoie_max_range_partout(self):
        robot = Robot(initial_pose=(1.0, 1.0, 0.0))
        lidar = LidarSensor(robot, obstacles=[], max_range=5.0, num_rays=8)

        scan = lidar.scan()

        self.assertEqual(len(scan), 8)
        self.assertTrue(all(d == 5.0 for d in scan))

    def test_obstacle_a_distance_connue_est_detecte(self):
        robot = Robot(initial_pose=(5.0, 5.0, 0.0))
        obstacle = {"type": "rect", "x": 7.0, "y": 4.0, "w": 1.0, "h": 2.0}
        lidar = LidarSensor(robot, obstacles=[obstacle], max_range=5.0, num_rays=36)

        scan = lidar.scan()

        self.assertAlmostEqual(scan[0], 2.0, places=3)  # rayon vers l'Est (angle 0)

    def test_min_distance_renvoie_le_plus_proche(self):
        robot = Robot(initial_pose=(5.0, 5.0, 0.0))
        obstacle_proche = {"type": "rect", "x": 6.0, "y": 4.9, "w": 0.2, "h": 0.2}
        lidar = LidarSensor(robot, obstacles=[obstacle_proche], max_range=5.0, num_rays=36)

        self.assertLess(lidar.min_distance(), 2.0)

    def test_update_obstacles_change_bien_le_scan(self):
        robot = Robot(initial_pose=(1.0, 1.0, 0.0))
        lidar = LidarSensor(robot, obstacles=[], max_range=5.0, num_rays=8)
        self.assertEqual(lidar.min_distance(), 5.0)

        lidar.update_obstacles([{"type": "rect", "x": 2.0, "y": 0.5, "w": 0.5, "h": 1.0}])
        self.assertLess(lidar.min_distance(), 5.0)


class TestCamera(unittest.TestCase):
    def test_cible_devant_dans_le_fov_est_visible(self):
        robot = Robot(initial_pose=(5.0, 5.0, 0.0))
        cam = Camera(robot, mount_angle_deg=0.0, fov_deg=90, max_range=5.0)

        visibles = cam.observe([(7.0, 5.0)])

        self.assertEqual(len(visibles), 1)
        self.assertAlmostEqual(visibles[0]["distance"], 2.0, places=3)

    def test_cible_hors_champ_de_vision_non_detectee(self):
        robot = Robot(initial_pose=(5.0, 5.0, 0.0))
        cam = Camera(robot, mount_angle_deg=0.0, fov_deg=60, max_range=5.0)

        # Cible plein Nord (90deg), hors d'un FOV de 60deg (demi-fov=30deg)
        visibles = cam.observe([(5.0, 7.0)])

        self.assertEqual(len(visibles), 0)

    def test_cible_hors_portee_non_detectee(self):
        robot = Robot(initial_pose=(5.0, 5.0, 0.0))
        cam = Camera(robot, mount_angle_deg=0.0, fov_deg=90, max_range=3.0)

        visibles = cam.observe([(10.0, 5.0)])  # a 5m, portee max = 3m

        self.assertEqual(len(visibles), 0)

    def test_observation_triee_par_distance_croissante(self):
        robot = Robot(initial_pose=(0.0, 0.0, 0.0))
        cam = Camera(robot, mount_angle_deg=0.0, fov_deg=180, max_range=10.0)

        visibles = cam.observe([(5.0, 0.0), (1.0, 0.0), (3.0, 0.0)])

        distances = [c["distance"] for c in visibles]
        self.assertEqual(distances, sorted(distances))

    def test_creer_cameras_robot_frontale_et_surveillance_opposees(self):
        robot = Robot(initial_pose=(5.0, 5.0, 0.0))
        frontale, surveillance = creer_cameras_robot(robot)

        cible_devant = [(7.0, 5.0)]
        cible_derriere = [(3.0, 5.0)]

        self.assertEqual(len(frontale.observe(cible_devant)), 1)
        self.assertEqual(len(frontale.observe(cible_derriere)), 0)
        self.assertEqual(len(surveillance.observe(cible_derriere)), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
