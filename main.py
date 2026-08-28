"""
main.py — Démonstration du module "Système et Cinématique" + aperçu du
système complet.

Ce script montre d'abord que le coeur cinématique fonctionne de bout en
bout : création du robot, commande, avance, virage, respect des limites,
export d'un log rejouable. Il enchaîne ensuite avec un aperçu rapide du
système complet (perception, localisation EKF, sécurité, sûreté,
planification, commande), réellement assemblé dans
experiments/integration_finale.py (voir SIMULATION_INTEGRATION.md pour le
détail).

Lancer avec :
    python main.py

Pour le détail complet du système (graphiques, logs, comparaison A*/RRT) :
    python -m experiments.integration_finale
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


def demo_systeme_complet():
    print("\n--- Scénario 5 : système complet (patrouille entrepôt, A*) ---")
    from experiments.integration_finale import _run_patrol
    metrics, robot, loop = _run_patrol("astar", verbose=False)
    print(f"Perception + Localisation EKF + Sécurité + Sûreté + Planification + Commande")
    print(f"Succès              : {metrics['success']}")
    print(f"Waypoints atteints  : {metrics['waypoints_reached']}/{metrics['waypoints_target']}")
    print(f"Erreur loc. max     : {metrics.get('max_loc_error')} m")
    print(f"Intrusions détectées: {metrics['intrusions_detected']} pas")
    print(f"État de sûreté final: {metrics['safety_final_state']}")
    print("Pour le détail complet (RRT, graphiques, logs de rejeu) : "
          "python -m experiments.integration_finale")


def main():
    robot = Robot()
    sim = Simulator(robot)

    print("Robot créé :", robot)
    print(f"Paramètres : rayon={robot.radius} m, entraxe={robot.wheel_base} m, "
          f"v_max={robot.v_max} m/s, omega_max={robot.omega_max} rad/s")

    demo_ligne_droite(robot, sim)
    demo_virage(robot, sim)
    demo_limite_vitesse(robot, sim)
    demo_arret_sur(robot)

    log_path = robot.export_log()
    print(f"\nLog exporté dans : {log_path} ({len(robot.history)} pas enregistrés)")
    print("\nDémonstration du module Système/Cinématique terminée avec succès.")

    demo_systeme_complet()
    print("\nAperçu du système complet terminé.")


if __name__ == "__main__":
    main()
