"""
main.py — Démonstration du module "Système et Cinématique".

Ce script montre que le coeur cinématique fonctionne de bout en bout :
création du robot, commande, avance, virage, respect des limites,
export d'un log rejouable. Il sert aussi de point de départ pour les
autres binômes : chaque module peut être branché ici au fur et à mesure
qu'il est développé (voir les commentaires "BRANCHEMENT FUTUR").

Lancer avec :
    python main.py
"""

import config
from robot.robot import Robot
from simulation.simulator import Simulator


def demo_ligne_droite(robot: Robot, sim: Simulator):
    print("\n--- Scénario 1 : avancer en ligne droite (v=0.3 m/s, 3 s) ---")
    sim.run(duration=3.0, command_fn=lambda r, t: r.set_velocity(0.3, 0.0), verbose=False)
    print("État final :", robot.get_state())


def demo_virage(robot: Robot, sim: Simulator):
    print("\n--- Scénario 2 : virage (v=0.2 m/s, omega=0.4 rad/s, 2 s) ---")
    sim.run(duration=2.0, command_fn=lambda r, t: r.set_velocity(0.2, 0.4), verbose=False)
    print("État final :", robot.get_state())


def demo_limite_vitesse(robot: Robot, sim: Simulator):
    print("\n--- Scénario 3 : commande excessive -> vérifie la saturation ---")
    robot.set_velocity(v=99.0, omega=99.0)
    print(f"Commande demandée : v=99.0, omega=99.0")
    print(f"Commande réellement appliquée (saturée) : v={robot.v}, omega={robot.omega}")
    assert abs(robot.v) <= config.V_MAX
    assert abs(robot.omega) <= config.OMEGA_MAX
    print("OK : les limites v_max / omega_max sont respectées.")


def demo_arret_sur(robot: Robot):
    print("\n--- Scénario 4 : arrêt sûr (emergency_stop) ---")
    robot.set_velocity(0.4, 0.2)
    robot.emergency_stop()
    print(f"Après emergency_stop() : v={robot.v}, omega={robot.omega}, stopped={robot.stopped}")
    robot.resume()
    print(f"Après resume() : stopped={robot.stopped}")


def main():
    robot = Robot()
    sim = Simulator(robot)

    print("Robot créé :", robot)
    print(f"Paramètres : rayon={robot.radius} m, entraxe={robot.wheel_base} m, "
          f"v_max={robot.v_max} m/s, omega_max={robot.omega_max} rad/s")

    # ------------------------------------------------------------------
    # BRANCHEMENT FUTUR : une fois les autres modules développés, on
    # pourra les connecter ici, par exemple :
    #
    #   from sensors.lidar import LidarSensor
    #   from security.intrusion_detector import IntrusionDetector
    #   from security.alert_manager import AlertManager
    #   from safety.safety_manager import SafetyManager
    #
    #   robot.sensors["lidar"] = LidarSensor(robot, obstacles=[...])
    #   safety_mgr = SafetyManager()
    #   sim.on_safety = lambda r, t: safety_mgr.check(r, ...)
    # ------------------------------------------------------------------

    demo_ligne_droite(robot, sim)
    demo_virage(robot, sim)
    demo_limite_vitesse(robot, sim)
    demo_arret_sur(robot)

    log_path = robot.export_log()
    print(f"\nLog exporté dans : {log_path} ({len(robot.history)} pas enregistrés)")
    print("\nDémonstration du module Système/Cinématique terminée avec succès.")


if __name__ == "__main__":
    main()
