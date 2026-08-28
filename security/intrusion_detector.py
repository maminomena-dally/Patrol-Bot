"""
security/intrusion_detector.py — Detection d'intrusion dans l'entrepot.

Module du Role 1 (Securite) — repris par Koja (Role 3) apres le depart de Kojy.

Utilise les cameras du robot (sensors/cameras.py) pour detecter des cibles
potentielles (personnes, objets non reperes) et les distinguer des
obstacles connus (racks, murs).

En cas d'intrusion confirmee :
  - Transmet l'information au SafetyManager (intrusion_confirmed=True)
  - Fournit la position de l'intrus pour le replanification (Role 3)

Interface avec Camera.observe() (Tino, Role 5) :
  - Entree : targets = [(x, y), ...]
  - Sortie : [{"x", "y", "distance", "angle_deg", "camera"}, ...]
"""

import math
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
from sensors.cameras import Camera


@dataclass
class IntrusionAlert:
    """Alerte d'intrusion detectee par une camera."""
    x: float
    y: float
    distance: float
    camera_name: str
    timestamp: float
    is_close: bool = False

    def to_dict(self) -> Dict:
        return {
            "x": self.x,
            "y": self.y,
            "distance": self.distance,
            "camera": self.camera_name,
            "time": self.timestamp,
            "is_close": self.is_close,
        }

    def __repr__(self):
        tag = " [PROCHE]" if self.is_close else ""
        return (f"IntrusionAlert(x={self.x:.2f}, y={self.y:.2f}, "
                f"d={self.distance:.2f}m, cam={self.camera_name}, "
                f"t={self.timestamp:.2f}s){tag}")


class IntrusionDetector:
    """
    Detecteur d'intrusion utilisant les cameras du robot.

    Pipeline :
        targets -> Camera.observe() -> filtrage obstacles connus
                -> deduplication -> IntrusionAlert(s)

    Utilisation typique dans la boucle de simulation :
        detector = IntrusionDetector(robot, known_obstacles=racks)
        confirmed, alerts = detector.check(all_targets, current_time)
        # confirmed -> bool pour SafetyManager.check(intrusion_confirmed=confirmed)
        # alerts -> positions pour replanification dynamique
    """

    def __init__(
        self,
        robot,
        known_obstacles: List[Tuple[float, float, float, float]] = None,
        detection_cooldown: float = 2.0,
        alert_distance_threshold: float = 3.0,
        obstacle_tolerance: float = 0.5,
    ):
        """
        Args:
            robot: instance du robot (robot.x, robot.y, robot.theta)
            known_obstacles: obstacles statiques [(x, y, w, h), ...]
            detection_cooldown: temps min (s) entre deux alertes
            alert_distance_threshold: distance (m) seuil "intrusion proche"
            obstacle_tolerance: marge (m) autour des obstacles pour le filtrage
        """
        self.robot = robot
        self.known_obstacles: List[Tuple[float, float, float, float]] = known_obstacles or []
        self.detection_cooldown = detection_cooldown
        self.alert_distance_threshold = alert_distance_threshold
        self.obstacle_tolerance = obstacle_tolerance

        # Cameras : frontale (90 deg) + surveillance arriere (120 deg)
        self.cameras: List[Camera] = [
            Camera(robot, mount_angle_deg=0, fov_deg=90,
                   max_range=5.0, nom="frontale"),
            Camera(robot, mount_angle_deg=180, fov_deg=120,
                   max_range=5.0, nom="surveillance"),
        ]

        # Etat interne
        self._last_alert_time: float = -999.0
        self._active_alerts: List[IntrusionAlert] = []
        self._history: List[Dict] = []
        self._total_detections: int = 0

    # ------------------------------------------------------------------
    # Methodes publiques
    # ------------------------------------------------------------------

    def check(
        self,
        targets: List[Tuple[float, float]],
        current_time: float,
    ) -> Tuple[bool, List[IntrusionAlert]]:
        """
        Verifie la presence d'intrus dans la scene.

        Args:
            targets: positions (x, y) de toutes les entites detectables
            current_time: temps de simulation (s)

        Returns:
            (intrusion_confirmed, alerts)
        """
        self._active_alerts = []

        # 1) Observer avec chaque camera
        all_observations: List[Dict] = []
        for cam in self.cameras:
            all_observations.extend(cam.observe(targets))

        # 2) Filtrer les obstacles connus
        intruders = self._filter_known_obstacles(all_observations)

        # 3) Dedupliquer (meme cible vue par les 2 cameras)
        unique = self._deduplicate(intruders)

        # 4) Intrusion confirmee ?
        intrusion_confirmed = len(unique) > 0

        # 5) Creer les alertes (si pas en cooldown)
        if intrusion_confirmed:
            self._total_detections += 1
            if current_time - self._last_alert_time >= self.detection_cooldown:
                for obs in unique:
                    is_close = obs["distance"] <= self.alert_distance_threshold
                    alert = IntrusionAlert(
                        x=obs["x"],
                        y=obs["y"],
                        distance=obs["distance"],
                        camera_name=obs["camera"],
                        timestamp=current_time,
                        is_close=is_close,
                    )
                    self._active_alerts.append(alert)
                self._last_alert_time = current_time

        # 6) Journaliser
        robot_x, robot_y, robot_theta = self.robot.get_true_pose()
        self._history.append({
            "time": current_time,
            "robot_x": robot_x,
            "robot_y": robot_y,
            "robot_theta": robot_theta,
            "intrusion_detected": intrusion_confirmed,
            "num_intruders": len(unique),
            "num_alerts": len(self._active_alerts),
            "closest_dist": min((o["distance"] for o in unique), default=None),
        })

        return intrusion_confirmed, self._active_alerts

    def get_intruder_positions(self) -> List[Tuple[float, float]]:
        """Positions des intrus pour le replanification (Role 3)."""
        return [(a.x, a.y) for a in self._active_alerts]

    def get_closest_intruder_distance(self) -> Optional[float]:
        """Distance de l'intrus le plus proche (None si aucun)."""
        if not self._active_alerts:
            return None
        return min(a.distance for a in self._active_alerts)

    def is_close_intrusion(self) -> bool:
        """True si un intrus est a distance critique."""
        d = self.get_closest_intruder_distance()
        return d is not None and d <= self.alert_distance_threshold

    @property
    def history(self) -> List[Dict]:
        """Historique complet des verifications."""
        return self._history

    @property
    def total_detections(self) -> int:
        """Nombre total de pas avec au moins 1 intrus."""
        return self._total_detections

    # ------------------------------------------------------------------
    # Methodes privees
    # ------------------------------------------------------------------

    def _filter_known_obstacles(self, observations: List[Dict]) -> List[Dict]:
        """Exclut les observations correspondant a des obstacles connus."""
        return [obs for obs in observations
                if not self._is_known_obstacle(obs["x"], obs["y"])]

    def _is_known_obstacle(self, x: float, y: float) -> bool:
        """Verifie si (x, y) est a l'interieur (ou pres) d'un obstacle."""
        tol = self.obstacle_tolerance
        for (ox, oy, ow, oh) in self.known_obstacles:
            if (ox - tol <= x <= ox + ow + tol and
                    oy - tol <= y <= oy + oh + tol):
                return True
        return False

    def _deduplicate(
        self, observations: List[Dict], tolerance: float = 0.3
    ) -> List[Dict]:
        """Supprime les doublons (meme cible vue par 2 cameras)."""
        unique: List[Dict] = []
        for obs in observations:
            if not any(
                math.hypot(obs["x"] - u["x"], obs["y"] - u["y"]) < tolerance
                for u in unique
            ):
                unique.append(obs)
        return unique
