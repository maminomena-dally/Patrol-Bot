"""
tests/test_safety.py — Tests du rôle Sûreté (safety/safety_manager.py).

Lancer avec :
    python -m unittest discover -s tests -v
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from robot.robot import Robot
from safety.safety_manager import SafetyManager, EtatSurete


class TestSafetyManagerNominal(unittest.TestCase):
    def test_situation_nominale_ne_declenche_rien(self):
        robot = Robot()
        robot.set_velocity(0.3, 0.1)
        sm = SafetyManager()

        etat = sm.check(robot, localization_uncertainty=0.05,
                         obstacle_distance=2.0, path_found=True)

        self.assertEqual(etat, EtatSurete.NOMINAL)
        self.assertFalse(robot.stopped)


class TestSafetyManagerLocalisation(unittest.TestCase):
    def test_incertitude_sous_le_seuil_ne_declenche_pas_arret(self):
        robot = Robot()
        sm = SafetyManager(localization_uncertainty_max=0.5)

        etat = sm.check(robot, localization_uncertainty=0.3,
                         obstacle_distance=2.0, path_found=True)

        self.assertEqual(etat, EtatSurete.NOMINAL)
        self.assertFalse(robot.stopped)

    def test_incertitude_au_dessus_du_seuil_declenche_arret_sur(self):
        robot = Robot()
        sm = SafetyManager(localization_uncertainty_max=0.5)

        etat = sm.check(robot, localization_uncertainty=0.6,
                         obstacle_distance=2.0, path_found=True)

        self.assertEqual(etat, EtatSurete.ARRET_SUR)
        self.assertTrue(robot.stopped)
        self.assertEqual(robot.v, 0.0)
        self.assertEqual(robot.omega, 0.0)


class TestSafetyManagerPlanification(unittest.TestCase):
    def test_un_seul_echec_de_chemin_reste_en_alerte(self):
        robot = Robot()
        sm = SafetyManager(tentatives_max_replanification=3)

        etat = sm.check(robot, localization_uncertainty=0.05,
                         obstacle_distance=2.0, path_found=False)

        self.assertEqual(etat, EtatSurete.ALERTE)
        self.assertFalse(robot.stopped)

    def test_echecs_consecutifs_au_seuil_declenchent_arret_sur(self):
        robot = Robot()
        sm = SafetyManager(tentatives_max_replanification=3)

        for _ in range(3):
            etat = sm.check(robot, localization_uncertainty=0.05,
                             obstacle_distance=2.0, path_found=False)

        self.assertEqual(etat, EtatSurete.ARRET_SUR)
        self.assertTrue(robot.stopped)

    def test_un_chemin_retrouve_reinitialise_le_compteur_d_echecs(self):
        robot = Robot()
        sm = SafetyManager(tentatives_max_replanification=3)

        sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0, path_found=False)
        sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0, path_found=True)  # reset
        etat = sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0, path_found=False)

        self.assertEqual(etat, EtatSurete.ALERTE)  # pas encore ARRET_SUR, compteur reparti à 1
        self.assertFalse(robot.stopped)


class TestSafetyManagerCapteurIndisponible(unittest.TestCase):
    def test_aucune_mesure_disponible_declenche_arret_sur(self):
        robot = Robot()
        sm = SafetyManager()

        etat = sm.check(robot, localization_uncertainty=None,
                         obstacle_distance=None, path_found=True)

        self.assertEqual(etat, EtatSurete.ARRET_SUR)
        self.assertTrue(robot.stopped)


class TestSafetyManagerJournal(unittest.TestCase):
    def test_transition_est_journalisee(self):
        robot = Robot()
        sm = SafetyManager(localization_uncertainty_max=0.5)

        sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0, path_found=True)
        sm.check(robot, localization_uncertainty=0.6, obstacle_distance=2.0, path_found=True)

        self.assertEqual(len(sm.journal), 1)
        self.assertEqual(sm.journal[0].transition, "NOMINAL -> ARRET_SUR")
        self.assertEqual(sm.journal[0].raison, "localisation_trop_incertaine")

    def test_etat_stable_ne_journalise_rien_de_nouveau(self):
        robot = Robot()
        sm = SafetyManager()

        sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0, path_found=True)
        sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0, path_found=True)

        self.assertEqual(len(sm.journal), 0)


class TestSafetyManagerReprise(unittest.TestCase):
    def test_resume_si_possible_ne_leve_pas_arret_si_toujours_critique(self):
        robot = Robot()
        sm = SafetyManager(localization_uncertainty_max=0.5)
        sm.check(robot, localization_uncertainty=0.6, obstacle_distance=2.0, path_found=True)

        resultat = sm.resume_si_possible(robot)

        self.assertFalse(resultat)
        self.assertTrue(robot.stopped)

    def test_resume_si_possible_leve_arret_si_redevenu_nominal(self):
        robot = Robot()
        sm = SafetyManager(localization_uncertainty_max=0.5)
        sm.check(robot, localization_uncertainty=0.6, obstacle_distance=2.0, path_found=True)
        sm.check(robot, localization_uncertainty=0.1, obstacle_distance=2.0, path_found=True)

        resultat = sm.resume_si_possible(robot)

        self.assertTrue(resultat)
        self.assertFalse(robot.stopped)


class TestSafetyManagerIntrusion(unittest.TestCase):
    def test_intrusion_confirmee_seule_passe_en_alerte_sans_arret(self):
        robot = Robot()
        sm = SafetyManager()

        etat = sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0,
                         path_found=True, intrusion_confirmed=True)

        self.assertEqual(etat, EtatSurete.ALERTE)
        self.assertFalse(robot.stopped)

    def test_intrusion_danger_declenche_arret_urgence(self):
        robot = Robot()
        sm = SafetyManager()

        etat = sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0,
                         path_found=True, intrusion_confirmed=True, intrusion_danger=True)

        self.assertEqual(etat, EtatSurete.ARRET_SUR)
        self.assertTrue(robot.stopped)

    def test_intrusion_danger_journalisee_avec_la_bonne_raison(self):
        robot = Robot()
        sm = SafetyManager()

        sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0,
                  path_found=True, intrusion_confirmed=True, intrusion_danger=True)

        self.assertEqual(sm.journal[-1].raison, "intrusion_danger")

    def test_aucune_intrusion_reste_nominal(self):
        robot = Robot()
        sm = SafetyManager()

        etat = sm.check(robot, localization_uncertainty=0.05, obstacle_distance=2.0,
                         path_found=True, intrusion_confirmed=False, intrusion_danger=False)

        self.assertEqual(etat, EtatSurete.NOMINAL)


if __name__ == "__main__":
    unittest.main(verbosity=2)
