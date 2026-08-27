"""
tests/test_alert_manager.py — Tests du module AlertManager.

Role 1 (Securite) / Koja (Role 3)

Tests unitaires :
  1. Nominal quand pas d'alerte
  2. INFO pour intrus eloigne
  3. WARNING pour intrus proche
  4. DANGER pour intrus critique
  5. Plusieurs intrus -> utilise le plus proche
  6. Retour au nominal apres delai
  7. intrusion_confirmed pour WARNING et DANGER
  8. should_alarm uniquement pour DANGER
  9. Escalade de niveaux (loin -> proche -> critique)
 10. Historique complet

Test d'integration avec visualisation :
  - Robot patrouille avec intrus a distances variables
  - Timeline des niveaux d'alerte + distances

Resultats : results/intrusion_detection/
"""

import math
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import csv
import unittest
from datetime import datetime

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from security.alert_manager import AlertManager, AlertLevel, AlertEvent
from security.intrusion_detector import IntrusionAlert


# ======================================================================
# Tests unitaires
# ======================================================================
class TestAlertManager(unittest.TestCase):
    """Tests unitaires du gestionnaire d'alertes."""

    def setUp(self):
        self.am = AlertManager(
            warning_distance=4.0, danger_distance=2.0, resolution_delay=1.0
        )

    def _make_alert(self, distance, x=7.0, y=5.0, t=0.0, cam="frontale"):
        return IntrusionAlert(
            x=x, y=y, distance=distance,
            camera_name=cam, timestamp=t,
        )

    def test_nominal_when_no_alerts(self):
        """Pas d'alerte -> niveau NOMINAL."""
        event = self.am.update([], 0.0)
        self.assertEqual(event.level, AlertLevel.NOMINAL)
        self.assertFalse(self.am.get_intrusion_confirmed())

    def test_info_level_for_distant_intruder(self):
        """Intrus a 5m (> warning 4m) -> INFO."""
        alert = self._make_alert(distance=5.0)
        event = self.am.update([alert], 0.0)
        self.assertEqual(event.level, AlertLevel.INFO)
        self.assertFalse(self.am.get_intrusion_confirmed())
        self.assertFalse(self.am.is_danger())

    def test_warning_level_for_close_intruder(self):
        """Intrus a 3m (danger < 3 < warning) -> WARNING."""
        alert = self._make_alert(distance=3.0)
        event = self.am.update([alert], 0.0)
        self.assertEqual(event.level, AlertLevel.WARNING)
        self.assertTrue(self.am.get_intrusion_confirmed())
        self.assertFalse(self.am.is_danger())

    def test_danger_level_for_very_close_intruder(self):
        """Intrus a 1m (< danger 2m) -> DANGER."""
        alert = self._make_alert(distance=1.0)
        event = self.am.update([alert], 0.0)
        self.assertEqual(event.level, AlertLevel.DANGER)
        self.assertTrue(self.am.get_intrusion_confirmed())
        self.assertTrue(self.am.is_danger())
        self.assertTrue(self.am.should_alarm())

    def test_multiple_intruders_uses_closest(self):
        """Plusieurs intrus -> niveau base sur le plus proche."""
        alerts = [
            self._make_alert(distance=4.5, x=10, y=5),
            self._make_alert(distance=1.5, x=6, y=5),
            self._make_alert(distance=3.0, x=8, y=3),
        ]
        event = self.am.update(alerts, 0.0)
        self.assertEqual(event.level, AlertLevel.DANGER)  # 1.5m < 2m
        self.assertEqual(event.closest_distance, 1.5)

    def test_returns_to_nominal_after_delay(self):
        """Retour au nominal apres resolution_delay sans alerte."""
        alert = self._make_alert(distance=3.0)
        self.am.update([alert], 0.0)
        self.assertEqual(self.am.current_level, AlertLevel.WARNING)

        # Avant le delai -> reste WARNING
        event = self.am.update([], 0.5)
        self.assertEqual(event.level, AlertLevel.WARNING)

        # Apres le delai -> NOMINAL
        event = self.am.update([], 2.0)
        self.assertEqual(event.level, AlertLevel.NOMINAL)

    def test_intrusion_confirmed_for_warning_and_danger(self):
        """intrusion_confirmed=True uniquement pour WARNING et DANGER."""
        # INFO
        self.am.update([self._make_alert(5.0)], 0.0)
        self.assertFalse(self.am.get_intrusion_confirmed())

        # WARNING
        self.am.update([self._make_alert(3.0)], 1.0)
        self.assertTrue(self.am.get_intrusion_confirmed())

        # DANGER
        self.am.update([self._make_alert(1.0)], 2.0)
        self.assertTrue(self.am.get_intrusion_confirmed())

        # Retour NOMINAL
        self.am.update([], 5.0)
        self.assertFalse(self.am.get_intrusion_confirmed())

    def test_should_alarm_only_for_danger(self):
        """should_alarm=True uniquement pour DANGER."""
        self.am.update([self._make_alert(4.5)], 0.0)  # INFO
        self.assertFalse(self.am.should_alarm())

        self.am.update([self._make_alert(3.0)], 1.0)  # WARNING
        self.assertFalse(self.am.should_alarm())

        self.am.update([self._make_alert(1.0)], 2.0)  # DANGER
        self.assertTrue(self.am.should_alarm())

    def test_level_escalation(self):
        """Escalade : INFO -> WARNING -> DANGER."""
        # Intrus s'approche progressivement
        self.am.update([self._make_alert(5.0)], 0.0)  # INFO
        self.assertEqual(self.am.current_level, AlertLevel.INFO)

        self.am.update([self._make_alert(3.0)], 1.0)  # WARNING
        self.assertEqual(self.am.current_level, AlertLevel.WARNING)

        self.am.update([self._make_alert(1.5)], 2.0)  # DANGER
        self.assertEqual(self.am.current_level, AlertLevel.DANGER)

        # De-escalade
        self.am.update([self._make_alert(3.5)], 3.0)  # WARNING
        self.assertEqual(self.am.current_level, AlertLevel.WARNING)

        self.am.update([self._make_alert(5.0)], 4.0)  # INFO
        self.assertEqual(self.am.current_level, AlertLevel.INFO)

    def test_history_recorded(self):
        """Chaque update() est enregistre dans l'historique."""
        self.am.update([], 0.0)
        self.am.update([self._make_alert(3.0)], 1.0)
        self.am.update([], 5.0)
        self.assertEqual(len(self.am.history), 3)

    def test_total_danger_counter(self):
        """Compteur d'evenements DANGER."""
        self.assertEqual(self.am.total_danger_events, 0)
        self.am.update([self._make_alert(1.0)], 0.0)
        self.assertEqual(self.am.total_danger_events, 1)
        self.am.update([self._make_alert(1.5)], 1.0)
        self.assertEqual(self.am.total_danger_events, 2)
        self.am.update([self._make_alert(3.0)], 2.0)
        self.assertEqual(self.am.total_danger_events, 2)  # pas incremente


# ======================================================================
# Test d'integration avec visualisation
# ======================================================================
def run_alert_timeline_test(output_dir="results/intrusion_detection"):
    """
    Scenario d'integration : intrus a distance variable.

    Un robot fixe observe un intrus qui s'approche puis s'eloigne.
    Montre l'escalade et la de-escalade des niveaux d'alerte.

    Genere :
      - alert_timeline.png  : distance + niveaux d'alerte sur le temps
      - alert_log.csv       : log complet
      - alert_report.txt    : rapport resume
    """
    os.makedirs(output_dir, exist_ok=True)

    dt = 0.1
    am = AlertManager(warning_distance=4.0, danger_distance=2.0,
                      resolution_delay=1.0)

    # Scenario : intrus s'approche de 8m a 1m puis s'eloigne
    # Phase 1 (t=0-5s)   : intrus a 8m -> 3m (s'approche)
    # Phase 2 (t=5-8s)   : intrus a 3m -> 1m (tres proche)
    # Phase 3 (t=8-10s)  : intrus a 1m (critique)
    # Phase 4 (t=10-15s) : intrus s'eloigne 1m -> 5m
    # Phase 5 (t=15-18s) : intrus a 5m -> 10m (disparait)
    # Phase 6 (t=18-22s) : plus d'intrus

    def intruder_distance(t):
        if t < 5:
            return 8.0 - t          # 8 -> 3
        elif t < 8:
            return 3.0 - (t - 5) * 2 / 3  # 3 -> 1
        elif t < 10:
            return 1.0               # reste a 1m
        elif t < 15:
            return 1.0 + (t - 10) * 4 / 5  # 1 -> 5
        elif t < 18:
            return 5.0 + (t - 15) * 5 / 3  # 5 -> 10
        return None  # disparu

    times = []
    distances = []
    levels = []
    level_names = []

    t = 0.0
    while t < 22.0:
        d = intruder_distance(t)

        if d is not None:
            alert = IntrusionAlert(
                x=7.0, y=5.0, distance=d,
                camera_name="frontale", timestamp=t,
            )
            event = am.update([alert], t)
        else:
            event = am.update([], t)

        times.append(t)
        distances.append(d if d is not None else np.nan)
        levels.append(event.level.value)
        level_names.append(event.level.name)

        t += dt

    # ----- Export CSV -----
    csv_path = os.path.join(output_dir, "alert_log.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "level", "closest_distance", "message"])
        for h in am.history:
            d_str = f'{h["closest_distance"]:.3f}' if h["closest_distance"] is not None else ""
            writer.writerow([
                f'{h["time"]:.3f}', h["level"], d_str, h["message"]
            ])

    # ----- Metriques -----
    total = len(am.history)
    nominal_count = sum(1 for l in levels if l == "nominal")
    info_count = sum(1 for l in levels if l == "info")
    warning_count = sum(1 for l in levels if l == "warning")
    danger_count = sum(1 for l in levels if l == "danger")
    valid_dists = [d for d in distances if not np.isnan(d)]
    min_dist = min(valid_dists) if valid_dists else None

    # ----- Rapport texte -----
    report_path = os.path.join(output_dir, "alert_report.txt")
    with open(report_path, "w") as f:
        f.write("=" * 60 + "\n")
        f.write("  RAPPORT — GESTION DES ALERTES\n")
        f.write("=" * 60 + "\n\n")
        f.write(f"Date      : {datetime.now().strftime('%Y-%m-%d %H:%M')}\n")
        f.write(f"Scenario  : Intrus a distance variable\n")
        f.write(f"Durée     : {t:.1f} s\n\n")
        f.write("--- Seuils de classification ---\n")
        f.write(f"  WARNING : d <= {am.warning_distance} m\n")
        f.write(f"  DANGER  : d <= {am.danger_distance} m\n")
        f.write(f"  Resolution delay : {am.resolution_delay} s\n\n")
        f.write("--- Distribution des niveaux ---\n")
        f.write(f"  NOMINAL : {nominal_count:4d} ({nominal_count/total*100:.1f}%)\n")
        f.write(f"  INFO    : {info_count:4d} ({info_count/total*100:.1f}%)\n")
        f.write(f"  WARNING : {warning_count:4d} ({warning_count/total*100:.1f}%)\n")
        f.write(f"  DANGER  : {danger_count:4d} ({danger_count/total*100:.1f}%)\n")
        f.write(f"  Total   : {total:4d}\n\n")
        if min_dist is not None:
            f.write(f"  Distance minimale atteinte : {min_dist:.2f} m\n")
        f.write(f"  Evenements DANGER (uniques)  : {am.total_danger_events}\n")

    # ----- Visualisation -----
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True,
                                   gridspec_kw={"height_ratios": [1, 1]})

    # Couleurs des niveaux
    level_colors = {
        "nominal": "#4CAF50",
        "info": "#2196F3",
        "warning": "#FF9800",
        "danger": "#F44336",
    }

    # --- Subplot 1 : Distance ---
    ax1.plot(times, distances, "-", color="#333333", linewidth=1, label="Distance intrus")
    ax1.axhline(y=am.warning_distance, color="#FF9800", linestyle="--",
                linewidth=1, alpha=0.7, label=f"Seuil WARNING ({am.warning_distance}m)")
    ax1.axhline(y=am.danger_distance, color="#F44336", linestyle="--",
                linewidth=1, alpha=0.7, label=f"Seuil DANGER ({am.danger_distance}m)")

    # Zones colorees pour les niveaux
    prev_level = levels[0]
    seg_start = times[0]
    for i in range(1, len(times)):
        if levels[i] != prev_level or i == len(times) - 1:
            color = level_colors[prev_level]
            ax1.axvspan(seg_start, times[i], alpha=0.08, color=color)
            seg_start = times[i]
            prev_level = levels[i]

    ax1.set_ylabel("Distance (m)")
    ax1.set_ylim(0, 10)
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_title("Gestion des Alertes — Intrusion a Distance Variable",
                  fontsize=13, fontweight="bold")
    ax1.grid(True, alpha=0.3)

    # --- Subplot 2 : Niveau d'alerte ---
    # Convertir niveaux en valeurs numeriques pour le plot
    level_values = {"nominal": 0, "info": 1, "warning": 2, "danger": 3}
    y_values = [level_values[l] for l in levels]

    ax2.fill_between(times, 0, y_values, step="post", alpha=0.3, color="#FF9800")
    ax2.step(times, y_values, where="post", color="#333333", linewidth=1)

    # Colorier les marches par niveau
    for i in range(len(times) - 1):
        color = level_colors[levels[i]]
        ax2.fill_between(times[i:i+2], level_values[levels[i]] - 0.4,
                         level_values[levels[i]] + 0.4,
                         step="post", alpha=0.5, color=color)

    ax2.set_yticks([0, 1, 2, 3])
    ax2.set_yticklabels(["NOMINAL", "INFO", "WARNING", "DANGER"], fontsize=9)
    ax2.set_ylim(-0.5, 3.5)
    ax2.set_xlabel("Temps (s)")
    ax2.set_ylabel("Niveau d'alerte")
    ax2.grid(True, alpha=0.3, axis="x")

    # Legende pour niveaux
    legend_patches = [
        mpatches.Patch(color=level_colors["nominal"], alpha=0.5, label="NOMINAL"),
        mpatches.Patch(color=level_colors["info"], alpha=0.5, label="INFO"),
        mpatches.Patch(color=level_colors["warning"], alpha=0.5, label="WARNING"),
        mpatches.Patch(color=level_colors["danger"], alpha=0.5, label="DANGER"),
    ]
    ax2.legend(handles=legend_patches, loc="upper right", fontsize=8,
               ncol=4)

    # Annotations de phase
    phase_labels = [
        (2.5, "S'approche\n(8m->3m)"),
        (6.5, "Tres proche\n(3m->1m)"),
        (9.0, "Critique\n(1m)"),
        (12.5, "S'eloigne\n(1m->5m)"),
        (16.5, "Disparait\n(5m->10m)"),
        (20.0, "Plus\nd'intrus"),
    ]
    for (tx, label) in phase_labels:
        ax1.annotate(label, (tx, 9), fontsize=7, ha="center",
                     color="#555555", style="italic")

    plt.tight_layout()
    png_path = os.path.join(output_dir, "alert_timeline.png")
    fig.savefig(png_path, dpi=150)
    plt.close(fig)

    # ----- Resume console -----
    print("\n" + "=" * 60)
    print("  RAPPORT — GESTION DES ALERTES")
    print("=" * 60)
    print(f"  Verifications  : {total}")
    print(f"  NOMINAL        : {nominal_count} ({nominal_count/total*100:.1f}%)")
    print(f"  INFO           : {info_count} ({info_count/total*100:.1f}%)")
    print(f"  WARNING        : {warning_count} ({warning_count/total*100:.1f}%)")
    print(f"  DANGER         : {danger_count} ({danger_count/total*100:.1f}%)")
    if min_dist is not None:
        print(f"  Dist. min      : {min_dist:.2f} m")
    print(f"  Evts DANGER    : {am.total_danger_events}")
    print(f"\n  Fichiers generes :")
    print(f"    {png_path}")
    print(f"    {csv_path}")
    print(f"    {report_path}")
    print("=" * 60 + "\n")

    return {
        "total": total, "danger_count": danger_count,
        "min_dist": min_dist, "danger_events": am.total_danger_events,
    }


# ======================================================================
# Point d'entree
# ======================================================================
if __name__ == "__main__":
    print("=== Tests unitaires — AlertManager ===\n")
    unittest.main(argv=[""], exit=False, verbosity=2)

    print("\n\n=== Test d'integration — Timeline des alertes ===\n")
    metrics = run_alert_timeline_test()
