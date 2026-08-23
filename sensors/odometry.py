"""
sensors/odometry.py — À IMPLÉMENTER par le binôme Perception / Localisation.

Rôle attendu (section 5 du cahier des charges) :
    Simuler les encodeurs de roues et produire un delta de déplacement
    (∆roue gauche, ∆roue droite) à chaque pas de temps, avec un bruit
    réaliste (dérive en cas de glissement, biais de calibration...).

Interface à respecter pour rester compatible avec robot/robot.py :
    - Ne JAMAIS lire robot.pose directement pour "tricher" : ne t'appuie
      que sur robot.get_true_pose() et robot.get_wheel_velocities(), en
      ajoutant du bruit, comme le ferait un vrai capteur.
    - Le résultat de ce module alimente localization/localization.py, qui
      lui, n'a pas accès à la vérité terrain.

Exemple de squelette :

    import random

    class Odometry:
        def __init__(self, robot, noise_std=0.01):
            self.robot = robot
            self.noise_std = noise_std
            self._last_wheel_pos = (0.0, 0.0)  # à intégrer depuis vL, vR

        def read(self, dt):
            vL, vR = self.robot.get_wheel_velocities()
            # TODO: intégrer vL, vR en distance parcourue par roue,
            # ajouter du bruit gaussien, retourner (d_left, d_right)
            raise NotImplementedError
"""

# TODO(binôme perception/localisation) : implémenter la classe Odometry.
