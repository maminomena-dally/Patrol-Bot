"""
security/alert_manager.py — Gestion des alertes de securite.

Module du Role 1 (Securite) — repris par Koja (Role 3).

Recoit les alertes de IntrusionDetector, les classe par niveau de
gravite (INFO / WARNING / DANGER) et declenche les actions appropriees.

Pipeline :
    IntrusionDetector.check() -> AlertManager.update() -> Speaker / SafetyManager

Interface avec SafetyManager (Tino, Role 5) :
    am.get_intrusion_confirmed()  -> bool  (pour sm.check(intrusion_confirmed=...))
    am.is_danger()                -> bool  (arret urgence)
"""

from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple
from security.intrusion_detector import IntrusionAlert


class AlertLevel(Enum):
    """Niveaux d'alerte de securite."""
    NOMINAL = "nominal"
    INFO = "info"
    WARNING = "warning"
    DANGER = "danger"


@dataclass
class AlertEvent:
    """Evenement d'alerte avec niveau et contexte."""
    timestamp: float
    level: AlertLevel
    num_intruders: int
    closest_distance: Optional[float]
    intruder_positions: List[Tuple[float, float]]
    message: str

    def to_dict(self) -> Dict:
        return {
            "time": self.timestamp,
            "level": self.level.value,
            "num_intruders": self.num_intruders,
            "closest_distance": self.closest_distance,
            "intruder_positions": self.intruder_positions,
            "message": self.message,
        }

    def __repr__(self):
        d = f", d_min={self.closest_distance:.2f}m" if self.closest_distance is not None else ""
        return (f"AlertEvent(t={self.timestamp:.2f}s, "
                f"{self.level.value.upper()}, "
                f"{self.num_intruders} intrus{d})")


class AlertManager:
    """
    Gestionnaire d'alertes de securite.

    Classe les intrusions par niveau et fournit les interfaces pour :
    - SafetyManager : intrusion_confirmed, is_danger
    - Speaker       : should_alarm
    - Journal       : history, last_event

    Utilisation typique :
        am = AlertManager()
        confirmed, alerts = detector.check(targets, t)
        event = am.update(alerts, t)
        # event.level -> AlertLevel
        # am.get_intrusion_confirmed() -> bool pour SafetyManager
        # am.should_alarm()           -> bool pour Speaker
    """

    def __init__(
        self,
        warning_distance: float = 4.0,
        danger_distance: float = 2.0,
        resolution_delay: float = 1.0,
    ):
        """
        Args:
            warning_distance: distance (m) seuil WARNING
            danger_distance: distance (m) seuil DANGER
            resolution_delay: temps (s) sans intrusion avant retour NOMINAL
        """
        self.warning_distance = warning_distance
        self.danger_distance = danger_distance
        self.resolution_delay = resolution_delay

        self._current_level: AlertLevel = AlertLevel.NOMINAL
        self._last_intrusion_time: float = -999.0
        self._total_danger_events: int = 0
        self._history: List[Dict] = []

    def update(
        self,
        alerts: List[IntrusionAlert],
        current_time: float,
    ) -> AlertEvent:
        """
        Traite les alertes et retourne l'etat courant.

        Args:
            alerts: liste d'IntrusionAlert (vide si pas d'intrusion)
            current_time: temps de simulation (s)

        Returns:
            AlertEvent avec le niveau et le contexte
        """
        if not alerts:
            # Pas d'alerte -> verifier retour au nominal
            if (current_time - self._last_intrusion_time >= self.resolution_delay
                    and self._current_level != AlertLevel.NOMINAL):
                self._current_level = AlertLevel.NOMINAL

            event = AlertEvent(
                timestamp=current_time,
                level=self._current_level,
                num_intruders=0,
                closest_distance=None,
                intruder_positions=[],
                message="Aucune intrusion" if self._current_level == AlertLevel.NOMINAL
                        else "Intrusion resolue (attente confirmation)",
            )
            self._history.append(event.to_dict())
            return event

        # Il y a des alertes
        self._last_intrusion_time = current_time
        positions = [(a.x, a.y) for a in alerts]
        n_intruders = len(alerts)
        closest = min(a.distance for a in alerts)

        if closest <= self.danger_distance:
            level = AlertLevel.DANGER
            self._total_danger_events += 1
            message = (f"INTRUSION CRITIQUE — {n_intruders} intrus, "
                       f"le plus proche a {closest:.2f}m")
        elif closest <= self.warning_distance:
            level = AlertLevel.WARNING
            message = (f"Intrusion proche — {n_intruders} intrus, "
                       f"le plus proche a {closest:.2f}m")
        else:
            level = AlertLevel.INFO
            message = (f"Intrusion detectee — {n_intruders} intrus, "
                       f"le plus proche a {closest:.2f}m")

        self._current_level = level

        event = AlertEvent(
            timestamp=current_time,
            level=level,
            num_intruders=n_intruders,
            closest_distance=closest,
            intruder_positions=positions,
            message=message,
        )
        self._history.append(event.to_dict())
        return event

    def get_intrusion_confirmed(self) -> bool:
        """True si intrusion active (WARNING ou DANGER) — pour SafetyManager."""
        return self._current_level in (AlertLevel.WARNING, AlertLevel.DANGER)

    def is_danger(self) -> bool:
        """True si niveau DANGER."""
        return self._current_level == AlertLevel.DANGER

    def should_alarm(self) -> bool:
        """True si l'alarme sonore doit retentir — pour Speaker."""
        return self._current_level == AlertLevel.DANGER

    @property
    def current_level(self) -> AlertLevel:
        """Niveau d'alerte actuel."""
        return self._current_level

    @property
    def history(self) -> List[Dict]:
        """Historique complet des mises a jour."""
        return self._history

    @property
    def total_danger_events(self) -> int:
        """Nombre d'evenements au niveau DANGER."""
        return self._total_danger_events
