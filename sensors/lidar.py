"""
sensors/lidar.py — À IMPLÉMENTER par le binôme Perception / Navigation.

Rôle attendu (sections 5, 12 du cahier des charges) :
    Simuler un capteur de distance type LiDAR simplifié : renvoyer, à
    chaque appel, une liste de distances mesurées le long de plusieurs
    rayons répartis autour du robot (voir config.LIDAR_NUM_RAYS,
    config.LIDAR_MAX_RANGE), en tenant compte des obstacles de la carte.

Interface à respecter :
    - Le capteur doit utiliser robot.get_true_pose() (vérité terrain) et
      une représentation de la carte / des obstacles (à définir par ce
      binôme, par exemple une liste de polygones ou une grille).
    - Le résultat alimente planning/ (évitement, replanification, section 12)
      et security/intrusion_detector.py peut aussi s'en servir en complément
      des caméras.

Exemple de squelette :

    import config

    class LidarSensor:
        def __init__(self, robot, obstacles, max_range=config.LIDAR_MAX_RANGE,
                     num_rays=config.LIDAR_NUM_RAYS):
            self.robot = robot
            self.obstacles = obstacles
            self.max_range = max_range
            self.num_rays = num_rays

        def scan(self):
            x, y, theta = self.robot.get_true_pose()
            # TODO: pour chaque rayon (angle réparti sur 360°), calculer la
            # distance à la première intersection avec un obstacle, ou
            # max_range si rien n'est détecté.
            raise NotImplementedError
"""

# TODO(binôme perception/navigation) : implémenter la classe LidarSensor.
