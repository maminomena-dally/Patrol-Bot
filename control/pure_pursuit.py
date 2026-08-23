"""
control/pure_pursuit.py — À IMPLÉMENTER par le binôme Contrôle.

Rôle attendu (slide 26 du support de cours) :
    Suivre un chemin (liste de points, issue de planning/astar.py ou
    planning/rrt.py) en visant à chaque instant un point "en avant" sur
    le chemin (lookahead), et en calculant une commande (v, omega) à
    envoyer via robot.set_velocity(v, omega).

Interface attendue :
    - `compute_command(robot, path, lookahead_distance) -> (v, omega)`
    - Doit utiliser robot.get_true_pose() en test, puis à terme la pose
      ESTIMÉE renvoyée par localization/localization.py (jamais la
      vérité terrain en conditions réelles).
    - Respecter nativement les limites : robot.set_velocity() sature déjà
      selon config.V_MAX / config.OMEGA_MAX, donc pas besoin de le refaire.

Exemple de squelette :

    class PurePursuitController:
        def __init__(self, lookahead_distance=0.5, v_cruise=0.3):
            self.lookahead_distance = lookahead_distance
            self.v_cruise = v_cruise

        def compute_command(self, current_pose, path):
            # TODO: trouver le point cible sur `path` à lookahead_distance,
            # calculer l'angle vers ce point, en déduire omega.
            raise NotImplementedError
"""

# TODO(binôme contrôle) : implémenter la classe PurePursuitController.
