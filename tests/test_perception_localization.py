"""
tests/test_perception_localization.py — Tests du binôme Perception / Localisation.

Couvre sensors/odometry.py, sensors/landmarks.py et
localization/localization.py.

Lancer avec :
    python -m unittest discover -s tests -v
"""

import math
import sys
import os
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import config
from robot.robot import Robot
from robot.kinematics import normalize_angle
from sensors.odometry import Odometry
from sensors.landmarks import LandmarkDetector
from localization.localization import Localizer


class TestOdometry(unittest.TestCase):
    def test_lecture_sans_bruit_ligne_droite(self):
        """Sans bruit, d_left = d_right = v * dt en ligne droite."""
        robot = Robot()
        robot.set_velocity(0.4, 0.0)
        odom = Odometry(robot, noise_std=0.0)

        d_left, d_right = odom.read(dt=0.5)

        self.assertAlmostEqual(d_left, 0.2, places=6)
        self.assertAlmostEqual(d_right, 0.2, places=6)

    def test_lecture_sans_bruit_rotation(self):
        """Sans bruit, d_left != d_right en rotation (roues opposées)."""
        robot = Robot()
        robot.set_velocity(0.0, 1.0)
        odom = Odometry(robot, noise_std=0.0)

        d_left, d_right = odom.read(dt=0.5)

        self.assertLess(d_left, 0.0)
        self.assertGreater(d_right, 0.0)
        self.assertAlmostEqual(d_left, -d_right, places=6)

    def test_bruit_gaussien_non_nul(self):
        """Avec noise_std > 0, deux lectures successives diffèrent (bruit appliqué)."""
        robot = Robot()
        robot.set_velocity(0.3, 0.0)
        odom = Odometry(robot, noise_std=0.05)

        lectures = {odom.read(dt=0.1) for _ in range(20)}
        self.assertGreater(len(lectures), 1)


class TestLandmarkDetector(unittest.TestCase):
    def test_balise_hors_de_portee_non_detectee(self):
        robot = Robot()  # pose (0, 0, 0)
        landmarks = [{"id": 0, "x": 10.0, "y": 10.0}]
        detector = LandmarkDetector(robot, landmarks, detection_radius=2.0)

        self.assertEqual(detector.detect(), [])

    def test_balise_a_portee_detectee_sans_bruit(self):
        robot = Robot()  # pose (0, 0, 0)
        landmarks = [{"id": 0, "x": 1.0, "y": 0.0}]
        detector = LandmarkDetector(robot, landmarks, detection_radius=2.0,
                                     noise_std_distance=0.0, noise_std_angle=0.0)

        detections = detector.detect()

        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0]["id"], 0)
        self.assertAlmostEqual(detections[0]["distance"], 1.0, places=6)
        self.assertAlmostEqual(detections[0]["angle"], 0.0, places=6)

    def test_angle_relatif_au_cap_du_robot(self):
        """Balise droit devant un robot orienté à 90° -> angle mesuré ~ 0."""
        robot = Robot(initial_pose=(0.0, 0.0, math.pi / 2))
        landmarks = [{"id": 0, "x": 0.0, "y": 1.0}]
        detector = LandmarkDetector(robot, landmarks, detection_radius=2.0,
                                     noise_std_distance=0.0, noise_std_angle=0.0)

        detections = detector.detect()

        self.assertAlmostEqual(detections[0]["angle"], 0.0, places=6)


class TestLocalizer(unittest.TestCase):
    def test_predict_ligne_droite(self):
        localizer = Localizer(initial_pose=(0.0, 0.0, 0.0))
        localizer.predict(d_left=1.0, d_right=1.0)

        self.assertAlmostEqual(localizer.estimated_pose.x, 1.0, places=6)
        self.assertAlmostEqual(localizer.estimated_pose.y, 0.0, places=6)
        self.assertAlmostEqual(localizer.estimated_pose.theta, 0.0, places=6)

    def test_predict_rotation_pure(self):
        localizer = Localizer(initial_pose=(0.0, 0.0, 0.0), wheel_base=config.WHEEL_BASE)
        localizer.predict(d_left=-0.1, d_right=0.1)

        expected_theta = normalize_angle(0.2 / config.WHEEL_BASE)
        self.assertAlmostEqual(localizer.estimated_pose.theta, expected_theta, places=6)
        # rotation pure -> ne doit quasiment pas se déplacer en x/y
        self.assertAlmostEqual(localizer.estimated_pose.x, 0.0, places=6)

    def test_predict_augmente_incertitude(self):
        localizer = Localizer(initial_pose=(0.0, 0.0, 0.0))
        self.assertEqual(localizer.uncertainty, 0.0)

        localizer.predict(d_left=0.5, d_right=0.5)

        self.assertGreater(localizer.uncertainty, 0.0)

    def test_correct_sans_balise_ne_change_rien(self):
        localizer = Localizer(initial_pose=(0.0, 0.0, 0.0))
        localizer.predict(d_left=0.5, d_right=0.5)
        pose_avant = localizer.estimated_pose
        incertitude_avant = localizer.uncertainty

        localizer.correct([])

        self.assertEqual(localizer.estimated_pose, pose_avant)
        self.assertEqual(localizer.uncertainty, incertitude_avant)

    def test_correct_reduit_incertitude_et_rapproche_la_pose(self):
        """
        Le robot 'vrai' est en (1.0, 0.0), l'odométrie a dérivé vers
        (1.3, 0.0). Une balise en (2.0, 0.0) mesurée sans bruit depuis la
        vraie position doit rapprocher l'estimation de la vérité.
        """
        localizer = Localizer(initial_pose=(0.0, 0.0, 0.0))
        localizer.predict(d_left=1.3, d_right=1.3)  # dérive : estime x=1.3 au lieu de 1.0
        incertitude_avant = localizer.uncertainty

        # mesure prise depuis la vraie position (1.0, 0.0) vers la balise (2.0, 0.0)
        mesure = [{"id": 0, "x": 2.0, "y": 0.0, "distance": 1.0, "angle": 0.0}]
        localizer.correct(mesure)

        self.assertLess(localizer.uncertainty, incertitude_avant)
        # l'estimation doit se rapprocher de x=1.0 (vérité), donc s'éloigner de 1.3
        self.assertLess(localizer.estimated_pose.x, 1.3)
        self.assertGreater(localizer.estimated_pose.x, 1.0 - 1e-6)


class TestIntegrationPerceptionLocalisation(unittest.TestCase):
    def test_boucle_predict_correct_sans_bruit_suit_la_verite_terrain(self):
        """
        Sans bruit sur les capteurs, la pose estimée doit rester très
        proche de la vérité terrain sur plusieurs pas de simulation.
        """
        robot = Robot()
        robot.set_velocity(0.3, 0.2)

        odom = Odometry(robot, noise_std=0.0)
        landmarks = [{"id": 0, "x": 5.0, "y": 5.0}]
        detector = LandmarkDetector(robot, landmarks, detection_radius=50.0,
                                     noise_std_distance=0.0, noise_std_angle=0.0)
        localizer = Localizer(initial_pose=robot.get_true_pose())

        dt = config.DT
        for _ in range(20):
            d_left, d_right = odom.read(dt)
            localizer.predict(d_left, d_right)
            localizer.correct(detector.detect())
            robot.step(dt)

        true_x, true_y, _ = robot.get_true_pose()
        self.assertAlmostEqual(localizer.estimated_pose.x, true_x, places=1)
        self.assertAlmostEqual(localizer.estimated_pose.y, true_y, places=1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
