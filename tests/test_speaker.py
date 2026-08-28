"""
tests/test_speaker.py — Tests du module Speaker + pipeline complet.

Role 1 (Securite) / Koja (Role 3)

Tests unitaires (Speaker) :
  1. Alarme inactive par defaut
  2. Alarme declenchee quand should_alarm=True
  3. Alarme arretee quand should_alarm=False
  4. Compteur d'alarmes
  5. Temps total d'alarme
  6. Historique des evenements

Test d'integration (pipeline complet) :
  IntrusionDetector -> AlertManager -> Speaker
  Patrouille avec intrus -> visualisation du pipeline complet

Resultats : results/intrusion_detection/


"""



import math
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import csv
import unittest
from datetime import datetime
from typing import List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from security.speaker import Speaker, AlarmPattern
from security.alert_manager import AlertManager, AlertLevel
from security.intrusion_detector import IntrusionDetector, IntrusionAlert


# ======================================================================
# Mock Robot
# ======================================================================
class MockRobot:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.time = 0.0

    def get_true_pose(self):
        return (self.x, self.y, self.theta)


# ======================================================================
# Tests unitaires — Speaker
# ======================================================================
class TestSpeaker(unittest.TestCase):

    def setUp(self):
        self.speaker = Speaker()

    def test_alarm_inactive_by_default(self):
        """Alarme inactive au depart."""
        self.assertFalse(self.speaker.is_alarming())
        result = self.speaker.update(should_alarm=False, current_time=0.0)
        self.assertFalse(result["is_alarming"])
        self.assertEqual(result["event"], "silent")

    def test_alarm_triggers_on(self):
        """Alarme declenchee quand should_alarm=True."""
        result = self.speaker.update(should_alarm=True, current_time=0.0)
        self.assertTrue(self.speaker.is_alarming())
        self.assertEqual(result["event"], "alarm_on")

    def test_alarm_stops(self):
        """Alarme arretee quand should_alarm=False."""
        self.speaker.update(should_alarm=True, current_time=0.0)
        result = self.speaker.update(should_alarm=False, current_time=2.0)
        self.assertFalse(self.speaker.is_alarming())
        self.assertEqual(result["event"], "alarm_off")

    def test_alarm_counter(self):
        """Compteur d'alarmes s'incremente a chaque declenchement."""
        self.assertEqual(self.speaker.total_alarms_triggered, 0)
        self.speaker.update(should_alarm=True, current_time=0.0)
        self.assertEqual(self.speaker.total_alarms_triggered, 1)
        self.speaker.update(should_alarm=False, current_time=2.0)
        self.speaker.update(should_alarm=True, current_time=3.0)
        self.assertEqual(self.speaker.total_alarms_triggered, 2)

    def test_total_alarm_time(self):
        """Temps total d'alarme calcule correctement."""
        self.speaker.update(should_alarm=True, current_time=0.0)
        self.speaker.update(should_alarm=True, current_time=2.0)
        self.speaker.update(should_alarm=False, current_time=5.0)
        self.assertAlmostEqual(self.speaker.total_alarm_time, 5.0, places=1)

        # 2eme alarme
        self.speaker.update(should_alarm=True, current_time=6.0)
        self.speaker.update(should_alarm=False, current_time=8.0)
        self.assertAlmostEqual(self.speaker.total_alarm_time, 7.0, places=1)

    def test_history_recorded(self):
        """Historique enregistre chaque update."""
        self.speaker.update(should_alarm=False, current_time=0.0)
        self.speaker.update(should_alarm=True, current_time=1.0)
        self.speaker.update(should_alarm=False, current_time=3.0)
        self.assertEqual(len(self.speaker.history), 3)
        self.assertEqual(self.speaker.history[0]["event"], "silent")
        self.assertEqual(self.speaker.history[1]["event"], "alarm_on")
        self.assertEqual(self.speaker.history[2]["event"], "alarm_off")


# ======================================================================
# Test d'integration — Pipeline complet
# ======================================================================
def run_full_pipeline_test(output_dir="results/intrusion_detection"):
    """
    Pipeline complet : IntrusionDetector -> AlertManager -> Speaker

    Genere :
      - full_pipeline.png  : carte + niveaux d'alerte + etat alarme
      - pipeline_log.csv   : log complet du pipeline
      - pipeline_report.txt : rapport
    """
    os.makedirs(output_dir, exist_ok=True)

    # ----- Entrepot -----
    racks = [
        (4.0, 2.0, 2.0, 0.4), (8.0, 2.0, 2.0, 0.4), (12.0, 2.0, 2.0, 0.4),
        (4.0, 6.0, 2.0, 0.4), (8.0, 6.0, 2.0, 0.4), (12.0, 6.0, 2.0, 0.4),
    ]
    walls = [
        (0.0, 0.0, 16.0, 0.1), (0.0, 9.9, 16.0, 0.1),
        (0.0, 0.0, 0.1, 10.0), (15.9, 0.0, 0.1, 10.0),
    ]
    waypoints = [
        (1.5, 1.5), (7.0, 1.5), (13.0, 1.5),
        (13.0, 8.0), (7.0, 8.0), (1.5, 8.0),
    ]
    intruder_schedule = [
        (3.0, (6.5, 4.0)),    # Intrus 1 : entre les racks
        (12.0, (11.0, 7.5)),  # Intrus 2 : pres du mur
    ]

    # ----- Initialisation pipeline -----
    dt = 0.05
    robot = MockRobot(x=waypoints[0][0], y=waypoints[0][1], theta=0.0)

    detector = IntrusionDetector(
        robot, known_obstacles=racks + walls,
        detection_cooldown=1.0, alert_distance_threshold=3.0,
    )
    am = AlertManager(warning_distance=4.0, danger_distance=2.0,
                      resolution_delay=1.0)
    speaker = Speaker()

    # ----- Simulation -----
    trajectory = [(robot.x, robot.y, robot.theta)]
    pipeline_log = []
    detection_events = []
    current_wp_idx = 0
    t = 0.0

    while current_wp_idx < len(waypoints) and t < 60.0:
        wx, wy = waypoints[current_wp_idx]
        dx = wx - robot.x
        dy = wy - robot.y

        if math.hypot(dx, dy) < 0.3:
            current_wp_idx += 1
            if current_wp_idx >= len(waypoints):
                break
            continue

        robot.theta = math.atan2(dy, dx)
        speed = 0.3
        robot.x += speed * math.cos(robot.theta) * dt
        robot.y += speed * math.sin(robot.theta) * dt
        robot.time = t

        trajectory.append((robot.x, robot.y, robot.theta))

        # Pipeline
        targets = [pos for (t_app, pos) in intruder_schedule if t >= t_app]
        confirmed, alerts = detector.check(targets, t)
        event = am.update(alerts, t)
        spk = speaker.update(am.should_alarm(), t)

        # Log pipeline
        pipeline_log.append({
            "time": t,
            "robot_x": robot.x,
            "robot_y": robot.y,
            "intrusion_detected": confirmed,
            "num_intruders": event.num_intruders,
            "closest_dist": event.closest_distance,
            "alert_level": event.level.value,
            "is_alarming": spk["is_alarming"],
            "alarm_event": spk["event"],
        })

        for a in alerts:
            detection_events.append(
                (t, (robot.x, robot.y), (a.x, a.y), a.is_close)
            )

        t += dt

    # ----- Export CSV -----
    csv_path = os.path.join(output_dir, "pipeline_log.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time", "robot_x", "robot_y",
            "intrusion_detected", "num_intruders", "closest_dist",
            "alert_level", "is_alarming", "alarm_event",
        ])
        for row in pipeline_log:
            writer.writerow([
                f'{row["time"]:.3f}',
                f'{row["robot_x"]:.3f}',
                f'{row["robot_y"]:.3f}',
                row["intrusion_detected"],
                row["num_intruders"],
                f'{row["closest_dist"]:.3f}' if row["closest_dist"] is not None else "",
                row["alert_level"],
                row["is_alarming"],
                row["alarm_event"],
            ])

    # ----- Metriques -----
    total = len(pipeline_log)
    det_count = sum(1 for r in pipeline_log if r["intrusion_detected"])
    alarm_count = sum(1 for r in pipeline_log if r["is_alarming"])
    danger_count = sum(1 for r in pipeline_log if r["alert_level"] == "danger")
    valid_dists = [r["closest_dist"] for r in pipeline_log
                   if r["closest_dist"] is not None]
    min_dist = min(valid_dists) if valid_dists else None
    alarm_pct = alarm_count / total * 100 if total else 0

    # ----- Rapport -----
    report_path = os.path.join(output_dir, "pipeline_report.txt")
    with open(report_path, "w") as f:
        f.write("=" * 60 + "\n")
        f.write("  RAPPORT — PIPELINE SECURITE COMPLET\n")
        f.write("=" * 60 + "\n\n")
        f.write(f"Date      : {datetime.now().strftime('%Y-%m-%d %H:%M')}\n")
        f.write(f"Scenario  : Patrouille + 2 intrus (pipeline 3 modules)\n")
        f.write(f"  Module 1 : IntrusionDetector (cameras)\n")
        f.write(f"  Module 2 : AlertManager (classification)\n")
        f.write(f"  Module 3 : Speaker (alarme)\n\n")
        f.write("--- Pipeline ---\n")
        f.write(f"  Verifications totales  : {total}\n")
        f.write(f"  Intrusion detectee     : {det_count} ({det_count/total*100:.1f}%)\n")
        f.write(f"  Niveau DANGER          : {danger_count} ({danger_count/total*100:.1f}%)\n")
        f.write(f"  Alarme activee         : {alarm_count} ({alarm_pct:.1f}%)\n")
        if min_dist is not None:
            f.write(f"  Distance min intrus    : {min_dist:.2f} m\n")
        f.write(f"  Alarmes declenchees     : {speaker.total_alarms_triggered}\n")
        f.write(f"  Temps total d'alarme    : {speaker.total_alarm_time:.1f} s\n")
        f.write(f"\n--- Niveaux d'alerte ---\n")
        for lvl in ["nominal", "info", "warning", "danger"]:
            c = sum(1 for r in pipeline_log if r["alert_level"] == lvl)
            f.write(f"  {lvl:10s} : {c:4d} ({c/total*100:.1f}%)\n")

    # ----- Visualisation (3 subplots) -----
    times = [r["time"] for r in pipeline_log]
    levels = [r["alert_level"] for r in pipeline_log]
    alarming = [r["is_alarming"] for r in pipeline_log]
    dists = [r["closest_dist"] if r["closest_dist"] is not None
             else np.nan for r in pipeline_log]

    level_colors = {
        "nominal": "#4CAF50", "info": "#2196F3",
        "warning": "#FF9800", "danger": "#F44336",
    }
    level_vals = {"nominal": 0, "info": 1, "warning": 2, "danger": 3}

    fig = plt.figure(figsize=(14, 12))
    gs = fig.add_gridspec(3, 1, height_ratios=[1.5, 1, 0.8], hspace=0.25)

    # --- Subplot 1 : Carte entrepot ---
    ax1 = fig.add_subplot(gs[0])

    for (wx, wy, ww, wh) in walls:
        ax1.add_patch(mpatches.Rectangle(
            (wx, wy), ww, wh, linewidth=1,
            edgecolor="black", facecolor="#555555", alpha=0.7))
    for (rx, ry, rw, rh) in racks:
        ax1.add_patch(mpatches.Rectangle(
            (rx, ry), rw, rh, linewidth=1,
            edgecolor="#333333", facecolor="#AAAAAA", alpha=0.8))

    for i, (wx, wy) in enumerate(waypoints):
        ax1.plot(wx, wy, "o", color="#2196F3", markersize=7, zorder=5)

    traj_x = [p[0] for p in trajectory]
    traj_y = [p[1] for p in trajectory]
    ax1.plot(traj_x, traj_y, "-", color="#4CAF50", linewidth=1.5,
             alpha=0.6, label="Trajectoire")
    ax1.plot(trajectory[0][0], trajectory[0][1], "^", color="#4CAF50",
             markersize=12, markeredgecolor="black", zorder=6, label="Depart")
    ax1.plot(trajectory[-1][0], trajectory[-1][1], "s", color="#F44336",
             markersize=12, markeredgecolor="black", zorder=6, label="Arrivee")

    for (ev_t, rob_pos, alert_pos, is_close) in detection_events:
        color = "#FF0000" if is_close else "#FF9800"
        marker = "X" if is_close else "D"
        ax1.plot(alert_pos[0], alert_pos[1], marker, color=color,
                 markersize=10, markeredgecolor="black", zorder=7)
        ax1.plot([rob_pos[0], alert_pos[0]], [rob_pos[1], alert_pos[1]],
                 "--", color=color, alpha=0.3, linewidth=1)

    for (t_app, pos) in intruder_schedule:
        ax1.plot(pos[0], pos[1], "*", color="#FF0000", markersize=15,
                 markeredgecolor="black", zorder=8)
        ax1.annotate(f"Intrus t={t_app:.0f}s", pos,
                     textcoords="offset points", xytext=(8, 8),
                     fontsize=9, color="#FF0000", fontweight="bold")

    ax1.set_xlim(-0.5, 17)
    ax1.set_ylim(-0.5, 11)
    ax1.set_aspect("equal")
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.set_title("Pipeline Securite — Patrouille Entrepot",
                  fontsize=13, fontweight="bold")
    ax1.legend(loc="upper right", fontsize=7)
    ax1.grid(True, alpha=0.3)

    # --- Subplot 2 : Niveaux d'alerte + distance ---
    ax2 = fig.add_subplot(gs[1])

    # Zones colorees
    for i in range(len(times) - 1):
        color = level_colors[levels[i]]
        ax2.axvspan(times[i], times[i + 1], alpha=0.2, color=color)

    y_vals = [level_vals[l] for l in levels]
    ax2.step(times, y_vals, where="post", color="#333333", linewidth=0.8)

    # Distance sur axe secondaire
    ax2b = ax2.twinx()
    ax2b.plot(times, dists, "-", color="#666666", linewidth=1, alpha=0.7,
              label="Distance intrus")
    ax2b.axhline(y=4.0, color="#FF9800", linestyle="--", linewidth=1,
                 alpha=0.5, label="Seuil WARNING (4m)")
    ax2b.axhline(y=2.0, color="#F44336", linestyle="--", linewidth=1,
                 alpha=0.5, label="Seuil DANGER (2m)")
    ax2b.set_ylabel("Distance (m)")
    ax2b.set_ylim(0, 8)
    ax2b.legend(loc="upper left", fontsize=7)

    ax2.set_yticks([0, 1, 2, 3])
    ax2.set_yticklabels(["NOMINAL", "INFO", "WARNING", "DANGER"], fontsize=9)
    ax2.set_ylim(-0.3, 3.5)
    ax2.set_ylabel("Niveau d'alerte")
    ax2.set_title("Niveaux d'alerte + distance intrus", fontsize=11)

    legend_patches = [
        mpatches.Patch(color=level_colors["nominal"], alpha=0.5, label="NOMINAL"),
        mpatches.Patch(color=level_colors["info"], alpha=0.5, label="INFO"),
        mpatches.Patch(color=level_colors["warning"], alpha=0.5, label="WARNING"),
        mpatches.Patch(color=level_colors["danger"], alpha=0.5, label="DANGER"),
    ]
    ax2.legend(handles=legend_patches, loc="upper right", fontsize=7, ncol=4)
    ax2.grid(True, alpha=0.3, axis="x")

    # --- Subplot 3 : Etat alarme ---
    ax3 = fig.add_subplot(gs[2])

    alarm_arr = np.array(alarming, dtype=float)
    for i in range(len(times) - 1):
        color = "#F44336" if alarming[i] else "#E0E0E0"
        ax3.fill_between(times[i:i + 2], 0, 1, step="post",
                         color=color, alpha=0.6)

    ax3.step(times, alarm_arr, where="post", color="#333333", linewidth=0.5)
    ax3.set_yticks([0, 1])
    ax3.set_yticklabels(["Silence", "ALARME"], fontsize=10, fontweight="bold")
    for i, c in enumerate(["#4CAF50", "#F44336"]):
        ax3.get_yticklabels()[i].set_color(c)
    ax3.set_ylim(-0.2, 1.3)
    ax3.set_xlabel("Temps (s)")
    ax3.set_title(f"Alarme sonore — {speaker.total_alarms_triggered} declenchements, "
                  f"{speaker.total_alarm_time:.1f}s total", fontsize=11)
    ax3.grid(True, alpha=0.3, axis="x")

    plt.tight_layout()
    png_path = os.path.join(output_dir, "full_pipeline.png")
    fig.savefig(png_path, dpi=150)
    plt.close(fig)

    # ----- Resume console -----
    print("\n" + "=" * 60)
    print("  RAPPORT — PIPELINE SECURITE COMPLET")
    print("=" * 60)
    print(f"  Verifications  : {total}")
    print(f"  Intrusion      : {det_count}/{total} ({det_count/total*100:.1f}%)")
    print(f"  DANGER         : {danger_count}/{total} ({danger_count/total*100:.1f}%)")
    print(f"  Alarme activee : {alarm_count}/{total} ({alarm_pct:.1f}%)")
    if min_dist is not None:
        print(f"  Dist. min      : {min_dist:.2f} m")
    print(f"  Declenchements : {speaker.total_alarms_triggered}")
    print(f"  Temps alarme   : {speaker.total_alarm_time:.1f} s")
    print(f"\n  Fichiers generes :")
    print(f"    {png_path}")
    print(f"    {csv_path}")
    print(f"    {report_path}")
    print("=" * 60 + "\n")

    return {
        "total": total, "danger_count": danger_count,
        "alarm_pct": alarm_pct, "min_dist": min_dist,
    }


# ======================================================================
# Point d'entree
# ======================================================================
if __name__ == "__main__":
    print("=== Tests unitaires — Speaker ===\n")
    unittest.main(argv=[""], exit=False, verbosity=2)

    print("\n\n=== Test d'integration — Pipeline complet ===\n")
    metrics = run_full_pipeline_test()
