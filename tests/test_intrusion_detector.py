"""
tests/test_intrusion_detector.py — Tests du module IntrusionDetector.

Role 1 (Securite) / Koja (Role 3)

Scenarios testes (unitaires) :
  1. Aucun intrus -> pas de detection
  2. Intrus dans le FOV -> detection confirmee
  3. Intrus derriere le robot -> detection par camera surveillance
  4. Intrus hors FOV -> pas de detection
  5. Obstacle connu (rack) -> pas flague comme intrus
  6. Cooldown respecte entre les alertes
  7. Deduplication entre cameras frontale/surveillance
  8. Intrusion proche (distance critique)
  9. Positions pour replanification
 10. Historique et compteurs

Test d'integration avec visualisation :
  - Patrouille dans l'entrepot avec apparition d'intrus
  - Generation : PNG, CSV, rapport texte

Resultats : results/intrusion_detection/
"""

import math
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import csv
import unittest
from datetime import datetime
from typing import List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from security.intrusion_detector import IntrusionDetector, IntrusionAlert
from config import OBSTACLE_SAFE_DISTANCE, LOCALIZATION_UNCERTAINTY_MAX


# ======================================================================
# Mock Robot (pour les tests unitaires sans Simulator)
# ======================================================================
class MockRobot:
    """Robot minimal pour les tests unitaires."""
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.time = 0.0

    def get_true_pose(self):
        return (self.x, self.y, self.theta)




# ======================================================================
# Tests unitaires
# ======================================================================
class TestIntrusionDetector(unittest.TestCase):
    """Tests unitaires du detecteur d'intrusion."""

    def setUp(self):
        self.robot = MockRobot(x=5.0, y=5.0, theta=0.0)
        self.racks = [
            (4.0, 2.0, 2.0, 0.4),
            (8.0, 2.0, 2.0, 0.4),
            (4.0, 5.0, 2.0, 0.4),
            (8.0, 5.0, 2.0, 0.4),
        ]
        self.detector = IntrusionDetector(
            self.robot, known_obstacles=self.racks,
            detection_cooldown=1.0, alert_distance_threshold=3.0,
        )

    def test_no_intruder_when_no_targets(self):
        """Aucune cible -> pas de detection."""
        confirmed, alerts = self.detector.check([], 0.0)
        self.assertFalse(confirmed)
        self.assertEqual(len(alerts), 0)

    def test_intruder_detected_in_fov(self):
        """Intrus dans le FOV frontal -> detection."""
        # Robot en (5,5), theta=0 -> regarde vers +x
        # Intrus en (7, 5) -> droit devant, a 2m
        confirmed, alerts = self.detector.check([(7.0, 5.0)], 0.0)
        self.assertTrue(confirmed)
        self.assertGreaterEqual(len(alerts), 1)

    def test_intruder_behind_detected_by_surveillance(self):
        """Intrus derriere le robot -> detecte par camera surveillance."""
        # Robot en (5,5), theta=0 -> regarde vers +x
        # Intrus en (2, 5) -> derriere, dans FOV 120° arriere
        confirmed, alerts = self.detector.check([(2.0, 5.0)], 0.0)
        self.assertTrue(confirmed)

    def test_intruder_outside_all_fov(self):
        """Intrus a 90 deg (lateral) -> hors des deux FOV."""
        # Robot en (5,5), theta=0
        # Intrus en (5, 10) -> lateral pur
        confirmed, alerts = self.detector.check([(5.0, 10.0)], 0.0)
        self.assertFalse(confirmed)

    def test_known_obstacle_not_flagged(self):
        """Un rack connu n'est pas signale comme intrus."""
        confirmed, alerts = self.detector.check([(5.0, 2.2)], 0.0)
        self.assertFalse(confirmed)
        self.assertEqual(len(alerts), 0)

    def test_cooldown_respected(self):
        """Pas de nouvelle alerte pendant le cooldown."""
        intruder = (7.0, 5.0)
        # 1ere detection
        c1, a1 = self.detector.check([intruder], 0.0)
        self.assertTrue(c1)
        self.assertGreater(len(a1), 0)
        # Dans le cooldown -> detection mais pas d'alerte
        c2, a2 = self.detector.check([intruder], 0.5)
        self.assertTrue(c2)
        self.assertEqual(len(a2), 0)
        # Apres cooldown -> nouvelle alerte
        c3, a3 = self.detector.check([intruder], 1.5)
        self.assertTrue(c3)
        self.assertGreater(len(a3), 0)

    def test_deduplication_across_cameras(self):
        """Meme intrus vu par 2 cameras -> 1 seule alerte."""
        confirmed, alerts = self.detector.check([(7.0, 5.1)], 0.0)
        self.assertLessEqual(len(alerts), 1)

    def test_close_intrusion_flag(self):
        """Intrus proche -> is_close=True."""
        # Robot en (5,5), intrus a 1m en (6, 5)
        confirmed, alerts = self.detector.check([(7.0, 5.0)], 0.0)
        self.assertTrue(confirmed)
        if alerts:
            self.assertTrue(alerts[0].is_close)

    def test_get_intruder_positions_for_replanning(self):
        """get_intruder_positions() retourne les positions pour le replan."""
        self.detector.check([(7.0, 5.0), (3.0, 5.0)], 0.0)
        positions = self.detector.get_intruder_positions()
        self.assertGreater(len(positions), 0)
        for (x, y) in positions:
            self.assertIsInstance(x, float)
            self.assertIsInstance(y, float)

    def test_get_closest_distance_no_intruder(self):
        """Pas d'intrus -> closest_distance = None."""
        self.detector.check([], 0.0)
        self.assertIsNone(self.detector.get_closest_intruder_distance())

    def test_history_recorded(self):
        """Chaque check() est enregistre dans l'historique."""
        self.detector.check([], 0.0)
        self.detector.check([(7.0, 5.0)], 1.0)
        self.assertEqual(len(self.detector.history), 2)
        self.assertEqual(self.detector.history[0]["time"], 0.0)

    def test_total_detections_counter(self):
        """Le compteur de detections s'incremente correctement."""
        self.detector.check([], 0.0)
        self.assertEqual(self.detector.total_detections, 0)
        self.detector.check([(7.0, 5.0)], 1.0)
        self.assertEqual(self.detector.total_detections, 1)
        self.detector.check([(7.0, 5.0)], 2.0)
        self.assertEqual(self.detector.total_detections, 2)


# ======================================================================
# Test d'integration avec visualisation
# ======================================================================
def run_intrusion_patrol_test(output_dir="results/intrusion_detection"):
    """
    Scenario d'integration : patrouille + intrusion.

    Genere :
      - intrusion_patrol.png   : trajectoire + FOV + detections
      - intrusion_log.csv      : log complet des verifications
      - intrusion_report.txt   : resume
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

    # Waypoints de patrouille
    waypoints = [
        (1.5, 1.5), (7.0, 1.5), (13.0, 1.5),
        (13.0, 8.0), (7.0, 8.0), (1.5, 8.0),
    ]

    # Intrus : (temps_apparition, position)
    intruder_schedule = [
        (3.0, (6.5, 4.0)),   # Intrus 1 : entre les racks
        (8.0, (11.0, 7.5)),  # Intrus 2 : pres du mur nord
    ]

    # ----- Simulation -----
    dt = 0.05
    robot = MockRobot(x=waypoints[0][0], y=waypoints[0][1], theta=0.0)
    detector = IntrusionDetector(
        robot, known_obstacles=racks + walls,
        detection_cooldown=1.5, alert_distance_threshold=3.0,
    )

    trajectory = [(robot.x, robot.y, robot.theta)]
    detection_events = []  # (time, robot_pos, alert_pos, is_close)
    all_alerts = []
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

        # Cibles actives
        targets = [pos for (t_app, pos) in intruder_schedule if t >= t_app]

        # Verification intrusion
        confirmed, alerts = detector.check(targets, t)

        for a in alerts:
            detection_events.append(
                (t, (robot.x, robot.y), (a.x, a.y), a.is_close)
            )
            all_alerts.append(a)

        t += dt

    # ----- Export CSV -----
    csv_path = os.path.join(output_dir, "intrusion_log.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time", "robot_x", "robot_y", "robot_theta",
            "intrusion_detected", "num_intruders", "num_alerts",
            "closest_dist",
        ])
        for h in detector.history:
            writer.writerow([
                f'{h["time"]:.3f}',
                f'{h["robot_x"]:.3f}',
                f'{h["robot_y"]:.3f}',
                f'{h["robot_theta"]:.3f}',
                h["intrusion_detected"],
                h["num_intruders"],
                h["num_alerts"],
                f'{h["closest_dist"]:.3f}' if h["closest_dist"] is not None else "",
            ])

    # ----- Rapport texte -----
    total_checks = len(detector.history)
    detection_steps = sum(1 for h in detector.history if h["intrusion_detected"])
    unique_alert_times = len(set(a.timestamp for a in all_alerts))
    closest = min((a.distance for a in all_alerts), default=None)
    first_det = min(
        (h["time"] for h in detector.history if h["intrusion_detected"]),
        default=None,
    )
    close_alerts = sum(1 for a in all_alerts if a.is_close)

    report_path = os.path.join(output_dir, "intrusion_report.txt")
    with open(report_path, "w") as f:
        f.write("=" * 60 + "\n")
        f.write("  RAPPORT — DETECTION D'INTRUSION\n")
        f.write("=" * 60 + "\n\n")
        f.write(f"Date      : {datetime.now().strftime('%Y-%m-%d %H:%M')}\n")
        f.write(f"Scenario  : Patrouille entrepot + 2 intrus programmas\n")
        f.write(f"Durée     : {t:.1f} s\n\n")
        f.write("--- Métriques ---\n")
        f.write(f"Vérifications totales    : {total_checks}\n")
        f.write(f"Pas avec intrus détecté  : {detection_steps}\n")
        f.write(f"Taux de détection        : {detection_steps/total_checks*100:.1f}%\n")
        f.write(f"Nombre d'alertes émises  : {unique_alert_times}\n")
        f.write(f"  dont alertes proches    : {close_alerts}\n")
        if closest is not None:
            f.write(f"Distance min intrus      : {closest:.2f} m\n")
        if first_det is not None:
            f.write(f"Première détection à     : {first_det:.2f} s\n")
        f.write(f"\n--- Intrus programmés ---\n")
        for (t_app, pos) in intruder_schedule:
            f.write(f"  t={t_app:.1f}s  position=({pos[0]}, {pos[1]})\n")
        f.write(f"\n--- Détecteur ---\n")
        f.write(f"  Caméras       : frontale (90°) + surveillance (180°, 120°)\n")
        f.write(f"  Cooldown      : {detector.detection_cooldown} s\n")
        f.write(f"  Seuil proche  : {detector.alert_distance_threshold} m\n")
        f.write(f"  Obstacles connus : {len(detector.known_obstacles)}\n")

    # ----- Visualisation -----
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))

    # Murs
    for (wx, wy, ww, wh) in walls:
        ax.add_patch(mpatches.Rectangle(
            (wx, wy), ww, wh, linewidth=1,
            edgecolor="black", facecolor="#555555", alpha=0.7))

    # Racks
    for (rx, ry, rw, rh) in racks:
        ax.add_patch(mpatches.Rectangle(
            (rx, ry), rw, rh, linewidth=1,
            edgecolor="#333333", facecolor="#AAAAAA", alpha=0.8))

    # Waypoints
    for i, (wx, wy) in enumerate(waypoints):
        ax.plot(wx, wy, "o", color="#2196F3", markersize=8, zorder=5)
        ax.annotate(f"WP{i}", (wx, wy), textcoords="offset points",
                    xytext=(5, 5), fontsize=8, color="#2196F3")

    # Trajectoire
    traj_x = [p[0] for p in trajectory]
    traj_y = [p[1] for p in trajectory]
    ax.plot(traj_x, traj_y, "-", color="#4CAF50", linewidth=1.5,
            alpha=0.6, label="Trajectoire")

    # Depart / Arrivee
    ax.plot(trajectory[0][0], trajectory[0][1], "^", color="#4CAF50",
            markersize=12, markeredgecolor="black", zorder=6)
    ax.plot(trajectory[-1][0], trajectory[-1][1], "s", color="#F44336",
            markersize=12, markeredgecolor="black", zorder=6)

    # FOV cones (premiere detection)
    if detection_events:
        ev_t, (rx_ev, ry_ev), _, _ = detection_events[0]
        idx = min(int(ev_t / dt), len(trajectory) - 1)
        _, _, theta_ev = trajectory[idx]
        colors_fov = ["#4CAF50", "#FF9800"]
        for cam, col in zip(detector.cameras, colors_fov):
            fov_rad = math.radians(cam.fov_deg / 2)
            mount_rad = math.radians(cam.mount_angle_deg)
            center = theta_ev + mount_rad
            ax.add_patch(mpatches.Wedge(
                (rx_ev, ry_ev), cam.max_range,
                math.degrees(center - fov_rad),
                math.degrees(center + fov_rad),
                alpha=0.08, facecolor=col,
                edgecolor=col, linewidth=0.5))

    # Evenements de detection
    for (ev_t, rob_pos, alert_pos, is_close) in detection_events:
        color = "#FF0000" if is_close else "#FF9800"
        marker = "X" if is_close else "D"
        size = 14 if is_close else 10
        ax.plot(alert_pos[0], alert_pos[1], marker, color=color,
                markersize=size, markeredgecolor="black", zorder=7)
        ax.plot([rob_pos[0], alert_pos[0]], [rob_pos[1], alert_pos[1]],
                "--", color=color, alpha=0.4, linewidth=1)

    # Intrus programmas
    for (t_app, pos) in intruder_schedule:
        ax.plot(pos[0], pos[1], "*", color="#FF0000", markersize=15,
                markeredgecolor="black", zorder=8)
        ax.annotate(f"Intrus t={t_app:.0f}s", pos,
                    textcoords="offset points", xytext=(8, 8),
                    fontsize=9, color="#FF0000", fontweight="bold")

    # Legende
    legend_elements = [
        mpatches.Patch(facecolor="#AAAAAA", edgecolor="#333333", label="Racks"),
        mpatches.Patch(facecolor="#555555", edgecolor="black", label="Murs"),
        plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="#2196F3",
                   markersize=8, label="Waypoints"),
        plt.Line2D([0], [0], marker="^", color="w", markerfacecolor="#4CAF50",
                   markeredgecolor="black", markersize=10, label="Départ"),
        plt.Line2D([0], [0], marker="s", color="w", markerfacecolor="#F44336",
                   markeredgecolor="black", markersize=10, label="Arrivée"),
        plt.Line2D([0], [0], marker="*", color="w", markerfacecolor="#FF0000",
                   markeredgecolor="black", markersize=12, label="Intrus"),
        plt.Line2D([0], [0], marker="D", color="w", markerfacecolor="#FF9800",
                   markeredgecolor="black", markersize=8, label="Détection"),
        plt.Line2D([0], [0], marker="X", color="w", markerfacecolor="#FF0000",
                   markeredgecolor="black", markersize=10, label="Détection proche"),
        plt.Line2D([0], [0], color="#4CAF50", linewidth=1.5, alpha=0.6,
                   label="Trajectoire"),
    ]
    ax.legend(handles=legend_elements, loc="upper right", fontsize=8,
              framealpha=0.9)

    ax.set_xlim(-0.5, 17)
    ax.set_ylim(-0.5, 11)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Détection d'Intrusion — Patrouille Entrepôt",
                 fontsize=13, fontweight="bold")
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    png_path = os.path.join(output_dir, "intrusion_patrol.png")
    fig.savefig(png_path, dpi=150)
    plt.close(fig)

    # ----- Resume console -----
    print("\n" + "=" * 60)
    print("  RAPPORT — DETECTION D'INTRUSION")
    print("=" * 60)
    print(f"  Vérifications  : {total_checks}")
    print(f"  Détecté        : {detection_steps}/{total_checks} "
          f"({detection_steps/total_checks*100:.1f}%)")
    print(f"  Alertes émises : {unique_alert_times} (dont {close_alerts} proches)")
    if closest is not None:
        print(f"  Dist. min intrus: {closest:.2f} m")
    if first_det is not None:
        print(f"  1ère détection : {first_det:.2f} s")
    print(f"\n  Fichiers générés :")
    print(f"    {png_path}")
    print(f"    {csv_path}")
    print(f"    {report_path}")
    print("=" * 60 + "\n")

    return {
        "total_checks": total_checks,
        "detection_steps": detection_steps,
        "unique_alerts": unique_alert_times,
        "closest_dist": closest,
        "first_detection": first_det,
    }


# ======================================================================
# Point d'entree
# ======================================================================
if __name__ == "__main__":
    print("=== Tests unitaires — IntrusionDetector ===\n")
    unittest.main(argv=[""], exit=False, verbosity=2)

    print("\n\n=== Test d'intégration — Patrouille avec intrusion ===\n")
    metrics = run_intrusion_patrol_test()
