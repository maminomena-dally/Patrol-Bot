"""
robot/robot.py — Classe Robot : coeur "Système et Cinématique" du projet.

Cette classe est LE point d'intégration du projet : tous les autres modules
(capteurs, sécurité, localisation, planification, contrôle, simulation)
doivent lire ou piloter le robot UNIQUEMENT à travers l'interface publique
définie ici (set_velocity, get_true_pose, get_footprint, ...), et ne jamais
modifier `robot.pose` directement. Cela garde le projet maintenable : la
cinématique peut être améliorée sans casser les autres modules.

Responsabilités couvertes (section 18 du cahier des charges) :
  - Maintenir l'état réel simulé du robot : x, y, theta.
  - Appliquer le modèle cinématique différentiel.
  - Gérer les deux roues motrices et leurs limites.
  - Appliquer les limites de vitesse linéaire et angulaire.
  - Représenter la géométrie du robot et ses zones de collision.
  - Fournir la vérité terrain aux capteurs et à la localisation.
  - Faire fonctionner la boucle temporelle de simulation (voir simulation/simulator.py).
  - Exposer les interfaces de commande aux modules de planification et de contrôle.
  - Permettre l'intégration des capteurs, caméras et du système d'alerte.
  - Enregistrer les états utiles aux essais et au rejeu.
"""

import csv
import os

from .kinematics import (
    Pose,
    saturate_command,
    integrate_euler,
    body_to_wheel_speeds,
    wheel_speeds_to_body,
)

import config


class Robot:
    def __init__(self,
                 initial_pose=config.INITIAL_POSE,
                 radius=config.ROBOT_RADIUS,
                 wheel_base=config.WHEEL_BASE,
                 v_max=config.V_MAX,
                 omega_max=config.OMEGA_MAX):
        x, y, theta = initial_pose
        self.pose = Pose(x, y, theta)

        # géométrie (section 4, 9)
        self.radius = radius
        self.wheel_base = wheel_base

        # limites cinématiques (section 9)
        self.v_max = v_max
        self.omega_max = omega_max

        # commande courante appliquée (après saturation)
        self.v = 0.0
        self.omega = 0.0

        # état système
        self.time = 0.0
        self.stopped = False   # arrêt sûr (déclenché par safety/safety_manager.py)

        # historique des états, pour rejeu / debug / logs (section 18, 19)
        self.history = []

        # Points d'extension : chaque binôme branche son module ici sans
        # toucher au coeur cinématique. Exemple :
        #   robot.sensors["lidar"] = LidarSensor(robot, config.LIDAR_MAX_RANGE)
        #   robot.security["speaker"] = Speaker()
        self.sensors = {}
        self.security = {}

    # ------------------------------------------------------------------
    # Commande — interface utilisée par planning/ et control/
    # ------------------------------------------------------------------
    def set_velocity(self, v, omega):
        """
        Commande le robot en (v, omega). Sature automatiquement selon les
        limites du robot (section 9). C'est l'interface principale à
        utiliser depuis control/pure_pursuit.py ou tout autre contrôleur.
        Si un arrêt sûr est actif, la commande est ignorée (v = omega = 0).
        """
        if self.stopped:
            self.v, self.omega = 0.0, 0.0
            return
        self.v, self.omega = saturate_command(v, omega, self.v_max, self.omega_max)

    def set_wheel_velocity(self, vL, vR):
        """Alternative : commander directement les deux roues (vL, vR en m/s)."""
        v, omega = wheel_speeds_to_body(vL, vR, self.wheel_base)
        self.set_velocity(v, omega)

    def get_wheel_velocities(self):
        """Retourne (vL, vR) équivalentes à la commande (v, omega) actuelle."""
        return body_to_wheel_speeds(self.v, self.omega, self.wheel_base)

    def emergency_stop(self):
        """Arrêt sûr immédiat. À appeler depuis safety/safety_manager.py."""
        self.stopped = True
        self.v, self.omega = 0.0, 0.0

    def resume(self):
        """Lève l'arrêt sûr (reprise de mission après contrôle)."""
        self.stopped = False

    # ------------------------------------------------------------------
    # Simulation — boucle temporelle
    # ------------------------------------------------------------------
    def step(self, dt=config.DT):
        """
        Avance la simulation d'un pas `dt` en appliquant le modèle
        cinématique différentiel à la commande courante. À appeler à
        chaque itération de la boucle principale (voir simulation/simulator.py).
        """
        self.pose = integrate_euler(self.pose, self.v, self.omega, dt)
        self.time += dt
        self._record_state()
        return self.pose

    # ------------------------------------------------------------------
    # Géométrie / collision — utilisé par planning/ et sensors/
    # ------------------------------------------------------------------
    def get_footprint(self):
        """Disque de collision du robot : {x, y, radius} (section 9, 18)."""
        return {"x": self.pose.x, "y": self.pose.y, "radius": self.radius}

    def distance_to(self, x, y):
        return ((self.pose.x - x) ** 2 + (self.pose.y - y) ** 2) ** 0.5

    def collides_with_point(self, x, y, obstacle_radius=0.0):
        """Test de collision simple robot / point (obstacle ponctuel)."""
        return self.distance_to(x, y) <= (self.radius + obstacle_radius)

    # ------------------------------------------------------------------
    # État / vérité terrain — utilisé par sensors/ et localization/
    # ------------------------------------------------------------------
    def get_true_pose(self):
        """
        Vérité terrain (x, y, theta), à utiliser UNIQUEMENT par les capteurs
        simulés (sensors/*.py) pour générer leurs mesures bruitées.
        Le module localization/ ne doit JAMAIS lire cette pose directement :
        il doit estimer sa propre pose à partir des mesures des capteurs.
        """
        return self.pose.x, self.pose.y, self.pose.theta

    def get_state(self):
        """Snapshot complet de l'état du robot (utilisé pour les logs et l'UI)."""
        return {
            "time": round(self.time, 3),
            "x": round(self.pose.x, 4),
            "y": round(self.pose.y, 4),
            "theta": round(self.pose.theta, 4),
            "v": round(self.v, 4),
            "omega": round(self.omega, 4),
            "stopped": self.stopped,
        }

    # ------------------------------------------------------------------
    # Journalisation / rejeu (section 18, 19 "Rejeu")
    # ------------------------------------------------------------------
    def _record_state(self):
        self.history.append(self.get_state())

    def export_log(self, path=None):
        """
        Écrit l'historique des états dans un fichier CSV rejouable
        (une ligne par pas de temps). Retourne le chemin du fichier écrit.
        """
        path = path or os.path.join(config.LOG_DIR, config.LOG_FILE)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        if not self.history:
            return path
        fieldnames = list(self.history[0].keys())
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.history)
        return path

    def reset(self, initial_pose=config.INITIAL_POSE):
        """Réinitialise complètement le robot (utile entre deux essais)."""
        x, y, theta = initial_pose
        self.pose = Pose(x, y, theta)
        self.v = 0.0
        self.omega = 0.0
        self.time = 0.0
        self.stopped = False
        self.history = []

    def __repr__(self):
        return (f"Robot(x={self.pose.x:.2f}, y={self.pose.y:.2f}, "
                f"theta={self.pose.theta:.2f}, v={self.v:.2f}, omega={self.omega:.2f})")
