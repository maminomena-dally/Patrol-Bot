"""
security/speaker.py — Alarme sonore du robot de patrouille.

Module du Role 1 (Securite) — repris par Koja (Role 3).

Recoit les ordres de l'AlertManager et declenche l'alarme sonore
en cas de niveau DANGER. En simulation, l'alarme est journalisee
(pret pour le branchement materiel).

Pipeline :
    AlertManager.should_alarm() -> Speaker.update() -> [alarme hardware]

Note : en simulation, l'alarme est representee par un journal d'evenements.
En production, remplacer _trigger_hardware_alarm() par l'appel reel
au haut-parleur du robot.
"""

from enum import Enum
from typing import List, Dict, Optional


class AlarmPattern(Enum):
    """Modes d'alarme."""
    CONTINUOUS = "continuous"          # son continu
    INTERMITTENT = "intermittent"      # bip regulier


class Speaker:
    """
    Gestionnaire d'alarme sonore.

    En simulation : journalise les evenements (alarm_on, alarm_off, alarm_active).
    En production : brancher _trigger_hardware_alarm() au vrai haut-parleur.

    Utilisation typique :
        speaker = Speaker()
        event = speaker.update(alert_manager.should_alarm(), current_time)
        # event["is_alarming"] -> True si l'alarme retentit
    """

    def __init__(
        self,
        pattern: str = "continuous",
    ):
        """
        Args:
            pattern: mode d'alarme ("continuous" ou "intermittent")
        """
        self.pattern = AlarmPattern(pattern)
        self._is_alarming: bool = False
        self._alarm_start_time: Optional[float] = None
        self._total_alarm_time: float = 0.0
        self._total_alarms_triggered: int = 0
        self._history: List[Dict] = []

    def update(
        self,
        should_alarm: bool,
        current_time: float,
    ) -> Dict:
        """
        Met a jour l'etat de l'alarme a chaque pas de temps.

        Args:
            should_alarm: True si AlertManager declenche DANGER
            current_time: temps de simulation (s)

        Returns:
            {"event": str, "is_alarming": bool}
        """
        was_alarming = self._is_alarming

        if should_alarm and not was_alarming:
            # Debut de l'alarme
            self._is_alarming = True
            self._alarm_start_time = current_time
            self._total_alarms_triggered += 1
            event_type = "alarm_on"
            self._trigger_hardware_alarm(True)

        elif not should_alarm and was_alarming:
            # Fin de l'alarme
            duration = current_time - self._alarm_start_time
            self._total_alarm_time += duration
            self._is_alarming = False
            event_type = "alarm_off"
            self._alarm_start_time = None
            self._trigger_hardware_alarm(False)

        elif should_alarm and was_alarming:
            event_type = "alarm_active"
        else:
            event_type = "silent"

        self._history.append({
            "time": current_time,
            "event": event_type,
            "is_alarming": self._is_alarming,
        })

        return {"event": event_type, "is_alarming": self._is_alarming}

    def is_alarming(self) -> bool:
        """True si l'alarme retentit actuellement."""
        return self._is_alarming

    @property
    def total_alarm_time(self) -> float:
        """Temps total d'alarme active (s)."""
        t = self._total_alarm_time
        if self._is_alarming and self._alarm_start_time is not None:
            t += self._history[-1]["time"] - self._alarm_start_time
        return t

    @property
    def total_alarms_triggered(self) -> int:
        """Nombre de fois que l'alarme a ete declenchee."""
        return self._total_alarms_triggered

    @property
    def history(self) -> List[Dict]:
        """Historique des evenements d'alarme."""
        return self._history

    def _trigger_hardware_alarm(self, on: bool):
        """
        Branchement materiel (a implementer en production).

        En simulation, ne fait rien. En production, connecter au
        haut-parleur du robot (GPIO, PWM, etc.).
        """
        pass  # Simulation : no-op
