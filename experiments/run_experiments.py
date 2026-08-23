"""
experiments/run_experiments.py — À COMPLÉTER par toute l'équipe au fur et
à mesure que les modules sont intégrés.

Rôle attendu (section 19 du cahier des charges) :
    Lancer une série de scénarios de test reproductibles (patrouille
    normale, obstacle imprévu, intrusion, perte de localisation...) et
    sauvegarder les résultats dans config.RESULTS_DIR pour analyse.

Pour l'instant, ce fichier ne contient qu'un exemple minimal basé sur le
module Système/Cinématique déjà fonctionnel (voir main.py à la racine
pour une démonstration plus complète et commentée).
"""

import config
from robot.robot import Robot
from simulation.simulator import Simulator


def scenario_avancer_puis_tourner():
    """Scénario simple : le robot avance puis tourne, comme au section 19."""
    robot = Robot()
    sim = Simulator(robot)

    # 3 secondes en ligne droite
    sim.run(duration=3.0, command_fn=lambda r, t: r.set_velocity(0.3, 0.0))
    # puis 2 secondes de rotation
    sim.run(duration=2.0, command_fn=lambda r, t: r.set_velocity(0.0, 0.5))

    path = robot.export_log(f"{config.RESULTS_DIR}/scenario_avancer_tourner.csv")
    print(f"Scénario terminé. État final : {robot.get_state()}")
    print(f"Log exporté : {path}")
    return robot


if __name__ == "__main__":
    scenario_avancer_puis_tourner()
