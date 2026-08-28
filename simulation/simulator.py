"""
simulation/simulator.py — Boucle temporelle générique de simulation.

Ce module fait tourner la boucle "état -> capteurs -> ... -> commande ->
cinématique -> mise à jour" (section 11 du cahier des charges). Dans cette
version, seul le bloc "Système et Cinématique" est implémenté : les
étapes localisation, détection, planification, sécurité sont de simples
points d'extension (callbacks) que les autres binômes viendront brancher,
sans avoir à modifier ce fichier.

Utilisation minimale (voir main.py) :

    from robot.robot import Robot
    from simulation.simulator import Simulator

    robot = Robot()
    sim = Simulator(robot)
    sim.run(duration=5.0, command_fn=lambda r, t: r.set_velocity(0.3, 0.2))
"""

import config
from robot.robot import Robot


class Simulator:
    def __init__(self, robot: Robot, dt: float = config.DT):
        self.robot = robot
        self.dt = dt

        # --- Points d'extension pour les autres modules ---
        # Chaque callback reçoit (robot, t) et peut être branché par les
        # autres binômes sans modifier ce fichier :
        #   sim.on_perceive   = mon_module_capteurs.lire
        #   sim.on_localize   = mon_module_localisation.estimer
        #   sim.on_plan       = mon_module_planification.replanifier
        #   sim.on_detect     = mon_module_securite.detecter_intrusion
        #   sim.on_safety     = mon_module_securite.verifier_arret_sur
        self.on_perceive = None
        self.on_localize = None
        self.on_plan = None
        self.on_detect = None
        self.on_safety = None

    def run(self, duration: float, command_fn=None, verbose: bool = False, stop_fn=None):
        """
        Exécute la boucle de simulation pendant `duration` secondes.

        command_fn(robot, t) -> None : fonction appelée à chaque pas pour
            fixer la commande (v, omega) du robot, typiquement via
            robot.set_velocity(...). Si absent, le robot garde sa dernière
            commande (utile pour tester la persistance d'une commande).

        Les callbacks on_perceive / on_localize / on_plan / on_detect /
        on_safety sont appelés dans cet ordre AVANT command_fn, pour
        reproduire la boucle système complète (section 11) une fois que
        les autres modules seront implémentés. Ils sont ignorés (no-op)
        tant qu'ils ne sont pas branchés.

        stop_fn(robot, t) -> bool, optionnel : vérifié après chaque pas ;
            si True, arrête la simulation avant d'avoir consommé toute la
            `duration` (ex. mission terminée, arrêt sûr déclenché). Ignoré
            si absent (comportement inchangé : la boucle va jusqu'au bout
            de `duration`).
        """
        n_steps = int(round(duration / self.dt))
        for i in range(n_steps):
            t = self.robot.time

            if self.on_perceive:
                self.on_perceive(self.robot, t)
            if self.on_localize:
                self.on_localize(self.robot, t)
            if self.on_detect:
                self.on_detect(self.robot, t)
            if self.on_plan:
                self.on_plan(self.robot, t)
            if self.on_safety:
                self.on_safety(self.robot, t)

            if command_fn:
                command_fn(self.robot, t)

            self.robot.step(self.dt)

            if verbose:
                print(self.robot.get_state())

            if stop_fn and stop_fn(self.robot, t):
                break

        return self.robot.get_state()
