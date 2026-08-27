"""
sensors/cameras.py — Cameras frontale et de surveillance (Role 5 - Tino,
sensor tombe sur ce role apres le depart de Kojy, faute de responsable).

Modelise la camera frontale et la camera de surveillance comme des champs
de vision (angle, portee) attaches au robot. Produit une "observation
simulee" au format attendu par security/intrusion_detector.py (section 13
du cahier des charges : camera -> observation -> detection -> validation
-> alerte / faux positif), meme si ce module de detection reste a
implementer par un autre binome.

Le projet etant hors ligne, l'observation est simulee via des "cibles"
(x, y) injectees dans la scene (ex : une personne simulee qui traverse le
champ de la camera de surveillance) -- conforme au squelette d'origine.

LIAISON avec security/ (a venir, non implemente) :
    Camera.observe(cibles) renvoie une liste de dicts
    {"x","y","distance","angle_deg","camera"} -- c'est exactement la
    forme d'"observation" que security/intrusion_detector.py attend en
    entree de sa methode detect(observation) (voir son docstring).
    Aucune dependance requise dans l'autre sens : ce module ne connait
    pas security/, il produit juste un contrat de donnees stable.
"""

import math

import config


class Camera:
    def __init__(self, robot, mount_angle_deg=0.0,
                 fov_deg=config.CAMERA_FRONT_FOV_DEG, max_range=5.0,
                 nom="frontale"):
        self.robot = robot
        self.mount_angle_deg = mount_angle_deg
        self.fov_deg = fov_deg
        self.max_range = max_range
        self.nom = nom

    def observe(self, targets):
        """
        Args:
            targets: liste de points (x, y) representant personnes/objets
                simules dans la scene.

        Returns:
            Liste de dicts pour chaque cible visible (dans le champ de
            vision ET a portee) :
                {"x", "y", "distance", "angle_deg", "camera"}
            triee par distance croissante (cible la plus proche en premier).
        """
        x, y, theta = self.robot.get_true_pose()
        cap_absolu = theta + math.radians(self.mount_angle_deg)
        demi_fov = math.radians(self.fov_deg) / 2.0

        visibles = []
        for (tx, ty) in targets:
            dx, dy = tx - x, ty - y
            distance = math.hypot(dx, dy)
            if distance > self.max_range or distance < 1e-9:
                continue

            angle_cible = math.atan2(dy, dx)
            ecart = self._normaliser_angle(angle_cible - cap_absolu)

            if abs(ecart) <= demi_fov:
                visibles.append({
                    "x": round(tx, 3),
                    "y": round(ty, 3),
                    "distance": round(distance, 3),
                    "angle_deg": round(math.degrees(ecart), 2),
                    "camera": self.nom,
                })

        visibles.sort(key=lambda c: c["distance"])
        return visibles

    @staticmethod
    def _normaliser_angle(angle):
        """Ramene un angle dans [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def creer_cameras_robot(robot):
    """
    Cree les 2 cameras prevues par le cahier des charges (section 13) :
    une frontale (dans l'axe du robot) et une de surveillance (montee a
    180 deg, champ de vision plus large, pour couvrir l'arriere pendant
    la patrouille).
    """
    frontale = Camera(robot, mount_angle_deg=0.0,
                       fov_deg=config.CAMERA_FRONT_FOV_DEG, nom="frontale")
    surveillance = Camera(robot, mount_angle_deg=180.0,
                           fov_deg=config.CAMERA_SURV_FOV_DEG, nom="surveillance")
    return frontale, surveillance
