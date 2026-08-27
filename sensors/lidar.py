"""
sensors/lidar.py — Capteur de distance type LiDAR simplifie (Role 5 - Tino,
sensor tombe sur ce role apres le depart de Kojy, faute de responsable).

Simule un capteur de distance : renvoie, a chaque appel, une liste de
distances mesurees le long de plusieurs rayons repartis a 360 deg autour
du robot (config.LIDAR_NUM_RAYS rayons, portee config.LIDAR_MAX_RANGE),
en tenant compte des obstacles rectangulaires de la carte (meme format
que partout ailleurs dans le projet : {"type":"rect","x","y","w","h"}).

LIAISON avec le reste du role Surete/Experimentation :
    LidarSensor.min_distance() alimente directement
    SafetyManager.check(obstacle_distance=...) -- jusqu'ici cette valeur
    etait simulee "en dur" (2.0 ou 0.3) dans demo_safety.py et les
    campagnes d'essais. Elle vient maintenant d'un vrai capteur.

LIAISON avec le reste de l'equipe :
    - planning/ peut l'utiliser pour de l'evitement local (hors perimetre
      de ce role, laisse en l'etat)
    - security/intrusion_detector.py peut s'en servir en complement des
      cameras (section 13 du cahier des charges) : min_distance() /
      scan() sont directement appelables depuis ce module, une fois
      implemente
"""

import math

import config


class LidarSensor:
    def __init__(self, robot, obstacles=None, max_range=config.LIDAR_MAX_RANGE,
                 num_rays=config.LIDAR_NUM_RAYS):
        self.robot = robot
        self.obstacles = obstacles if obstacles is not None else []
        self.max_range = max_range
        self.num_rays = num_rays
        self._last_scan = [max_range] * num_rays

    def update_obstacles(self, obstacles):
        """
        Remplace la liste d'obstacles connus du capteur. A appeler quand
        un obstacle imprevu apparait en cours de simulation (le lidar,
        contrairement a la carte de planification, doit voir le monde
        "tel qu'il est reellement" a chaque instant).
        """
        self.obstacles = obstacles

    def scan(self):
        """
        Renvoie la liste des `num_rays` distances mesurees (m), une par
        rayon, repartis uniformement sur 360 deg autour du robot, dans le
        referentiel du monde (angle absolu, pas relatif a theta -- un
        LiDAR tourne independamment de l'orientation du chassis).
        """
        x, y, _theta = self.robot.get_true_pose()
        distances = []
        for i in range(self.num_rays):
            angle = 2 * math.pi * i / self.num_rays
            dx, dy = math.cos(angle), math.sin(angle)
            d = self._distance_au_premier_obstacle(x, y, dx, dy)
            distances.append(round(d, 4))
        self._last_scan = distances
        return distances

    def min_distance(self):
        """
        Distance au point le plus proche detecte par le dernier scan.
        C'est cette valeur qui alimente SafetyManager.check(obstacle_distance=...).
        Relance un scan si aucun n'a encore ete fait.
        """
        if self._last_scan is None:
            self.scan()
        return min(self.scan())

    # ------------------------------------------------------------------
    # Geometrie : intersection rayon / obstacles rectangulaires
    # ------------------------------------------------------------------

    def _distance_au_premier_obstacle(self, ox, oy, dx, dy):
        """Distance a la premiere intersection rayon-obstacle, ou max_range."""
        meilleure = self.max_range
        for obs in self.obstacles:
            if obs.get("type") != "rect":
                continue  # seule la geometrie rectangulaire est geree ici
            t = self._intersection_rayon_rect(ox, oy, dx, dy, obs)
            if t is not None and t < meilleure:
                meilleure = t
        return meilleure

    def _intersection_rayon_rect(self, ox, oy, dx, dy, rect):
        """
        Test d'intersection rayon/AABB par la methode des "slabs".
        Renvoie la distance au point d'entree du rayon dans le
        rectangle, ou None si aucune intersection dans [0, max_range].
        """
        x_min, y_min = rect["x"], rect["y"]
        x_max, y_max = rect["x"] + rect["w"], rect["y"] + rect["h"]

        t_min, t_max = 0.0, self.max_range

        for (o, d, lo, hi) in [(ox, dx, x_min, x_max), (oy, dy, y_min, y_max)]:
            if abs(d) < 1e-9:
                if o < lo or o > hi:
                    return None  # rayon parallele a cet axe, hors du rectangle
                continue
            t1 = (lo - o) / d
            t2 = (hi - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            t_min = max(t_min, t1)
            t_max = min(t_max, t2)
            if t_min > t_max:
                return None

        return t_min if t_min > 1e-9 else None
