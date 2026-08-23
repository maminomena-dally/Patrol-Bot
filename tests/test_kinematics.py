"""
tests/test_kinematics.py — Tests unitaires du module Système/Cinématique.

Couvre les critères de la section 19 du cahier des charges :
    Avancer, Tourner, Roues différentielles, Limitation vitesse,
    Limitation rotation.

Lancer avec :
    python -m unittest discover -s tests -v
ou :
    python -m pytest tests/ -v
"""

import math
import sys
import os
import unittest

# Permet de lancer ce fichier directement (python tests/test_kinematics.py)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import config
from robot.robot import Robot
from robot.kinematics import (
    Pose, integrate_euler, saturate_command,
    wheel_speeds_to_body, body_to_wheel_speeds, normalize_angle,
)


class TestKinematicsPures(unittest.TestCase):
    """Tests des fonctions pures de robot/kinematics.py (sans classe Robot)."""

    def test_avancer_ligne_droite(self):
        """En ligne droite (omega=0), la pose évolue conformément au modèle."""
        pose = Pose(0.0, 0.0, 0.0)
        new_pose = integrate_euler(pose, v=1.0, omega=0.0, dt=1.0)
        self.assertAlmostEqual(new_pose.x, 1.0, places=6)
        self.assertAlmostEqual(new_pose.y, 0.0, places=6)
        self.assertAlmostEqual(new_pose.theta, 0.0, places=6)

    def test_tourner_sur_place(self):
        """theta évolue dans le sens demandé (v=0, omega>0 -> theta augmente)."""
        pose = Pose(0.0, 0.0, 0.0)
        new_pose = integrate_euler(pose, v=0.0, omega=1.0, dt=1.0)
        self.assertAlmostEqual(new_pose.x, 0.0, places=6)
        self.assertAlmostEqual(new_pose.y, 0.0, places=6)
        self.assertAlmostEqual(new_pose.theta, 1.0, places=6)

    def test_tourner_sens_negatif(self):
        pose = Pose(0.0, 0.0, 0.0)
        new_pose = integrate_euler(pose, v=0.0, omega=-1.0, dt=1.0)
        self.assertLess(new_pose.theta, 0.0)

    def test_roues_differentielles_ligne_droite(self):
        """vL = vR -> omega = 0 (le robot va droit)."""
        v, omega = wheel_speeds_to_body(vL=0.4, vR=0.4, wheel_base=config.WHEEL_BASE)
        self.assertAlmostEqual(v, 0.4, places=6)
        self.assertAlmostEqual(omega, 0.0, places=6)

    def test_roues_differentielles_rotation_sur_place(self):
        """vL = -vR -> v = 0 (rotation approx. sur place)."""
        v, omega = wheel_speeds_to_body(vL=-0.2, vR=0.2, wheel_base=config.WHEEL_BASE)
        self.assertAlmostEqual(v, 0.0, places=6)
        self.assertNotAlmostEqual(omega, 0.0, places=6)

    def test_conversion_aller_retour_roues(self):
        """body -> roues -> body doit redonner la même commande."""
        v0, omega0 = 0.3, 0.7
        vL, vR = body_to_wheel_speeds(v0, omega0, config.WHEEL_BASE)
        v1, omega1 = wheel_speeds_to_body(vL, vR, config.WHEEL_BASE)
        self.assertAlmostEqual(v0, v1, places=6)
        self.assertAlmostEqual(omega0, omega1, places=6)

    def test_limitation_vitesse(self):
        """|v| <= v_max après saturation, même pour une commande excessive."""
        v, omega = saturate_command(v=10.0, omega=0.0,
                                     v_max=config.V_MAX, omega_max=config.OMEGA_MAX)
        self.assertLessEqual(abs(v), config.V_MAX)

    def test_limitation_rotation(self):
        """|omega| <= omega_max après saturation, même pour une commande excessive."""
        v, omega = saturate_command(v=0.0, omega=10.0,
                                     v_max=config.V_MAX, omega_max=config.OMEGA_MAX)
        self.assertLessEqual(abs(omega), config.OMEGA_MAX)

    def test_normalisation_angle(self):
        # 3*pi et pi (ou -pi) sont le même angle a 2*pi pres.
        result = normalize_angle(3 * math.pi)
        self.assertTrue(-math.pi - 1e-6 <= result <= math.pi + 1e-6)
        self.assertAlmostEqual(abs(result), math.pi, places=6)

        result2 = normalize_angle(-3 * math.pi)
        self.assertTrue(-math.pi - 1e-6 <= result2 <= math.pi + 1e-6)
        self.assertAlmostEqual(abs(result2), math.pi, places=6)


class TestRobot(unittest.TestCase):
    """Tests d'intégration sur la classe Robot (section 19)."""

    def setUp(self):
        self.robot = Robot()

    def test_avancer(self):
        self.robot.set_velocity(0.5, 0.0)
        self.robot.step(dt=1.0)
        self.assertAlmostEqual(self.robot.pose.x, 0.5, places=6)
        self.assertAlmostEqual(self.robot.pose.y, 0.0, places=6)

    def test_tourner(self):
        self.robot.set_velocity(0.0, 1.0)
        self.robot.step(dt=1.0)
        self.assertAlmostEqual(self.robot.pose.theta, 1.0, places=6)

    def test_limitation_vitesse_appliquee_par_le_robot(self):
        self.robot.set_velocity(v=100.0, omega=0.0)
        self.assertLessEqual(abs(self.robot.v), config.V_MAX)

    def test_limitation_rotation_appliquee_par_le_robot(self):
        self.robot.set_velocity(v=0.0, omega=100.0)
        self.assertLessEqual(abs(self.robot.omega), config.OMEGA_MAX)

    def test_arret_sur(self):
        """emergency_stop() doit immobiliser le robot immédiatement."""
        self.robot.set_velocity(0.4, 0.5)
        self.robot.emergency_stop()
        self.assertEqual(self.robot.v, 0.0)
        self.assertEqual(self.robot.omega, 0.0)
        # une nouvelle commande ne doit pas relancer le robot tant qu'on
        # n'a pas appelé resume()
        self.robot.set_velocity(0.4, 0.5)
        self.assertEqual(self.robot.v, 0.0)
        self.assertEqual(self.robot.omega, 0.0)

    def test_reprise_apres_arret(self):
        self.robot.emergency_stop()
        self.robot.resume()
        self.robot.set_velocity(0.3, 0.0)
        self.assertAlmostEqual(self.robot.v, 0.3, places=6)

    def test_footprint_et_collision(self):
        fp = self.robot.get_footprint()
        self.assertEqual(fp["radius"], config.ROBOT_RADIUS)
        # un point au centre du robot doit être en collision
        self.assertTrue(self.robot.collides_with_point(0.0, 0.0))
        # un point très éloigné ne doit pas être en collision
        self.assertFalse(self.robot.collides_with_point(100.0, 100.0))

    def test_rejeu_export_log(self):
        """L'état et les événements doivent être reconstructibles depuis les logs."""
        self.robot.set_velocity(0.2, 0.1)
        for _ in range(5):
            self.robot.step(dt=0.1)
        self.assertEqual(len(self.robot.history), 5)
        path = self.robot.export_log(path="results/test_log.csv")
        self.assertTrue(os.path.exists(path))
        os.remove(path)


if __name__ == "__main__":
    unittest.main(verbosity=2)
