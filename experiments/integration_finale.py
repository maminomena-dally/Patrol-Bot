import math
import os
import random
import time

import config
from robot.robot import Robot
from sensors.odometry import Odometry
from sensors.landmarks import LandmarkDetector
from sensors.lidar import LidarSensor
from localization.localization import Localizer
from security.intrusion_detector import IntrusionDetector
from security.alert_manager import AlertManager
from security.speaker import Speaker
from safety.safety_manager import SafetyManager, EtatSurete
from control.pure_pursuit import PurePursuitController
from experiments.run_experiments import _make_planner, _resample_path, _ensure_dir
from experiments.entrepot_patrouille import (
    WAREHOUSE_OBSTACLES, WAREHOUSE_WAYPOINTS, WAREHOUSE_LANDMARKS,
)
from planning.astar import create_test_grid


INTRUDER_SCHEDULE = [
    (15.0, (10.0, 3.0)),   # intrus 1 : entre les rangees 1-2
    (70.0, (17.5, 10.5)),  # intrus 2 : pres de la zone de stockage reservee
]


def _run_patrol(planner_name="astar", verbose=False, max_time=400.0,
                 replan_every_n_steps=100):
    """
    Fait patrouiller le robot sur tous les waypoints de l'entrepot en
    branchant reellement les six modules. Retourne un dict de metriques.
    """
    dt = config.DT

    grid = create_test_grid(config.WORLD_WIDTH, config.WORLD_HEIGHT,
                             config.GRID_RESOLUTION, WAREHOUSE_OBSTACLES)
    planner = _make_planner(planner_name, grid, config.GRID_RESOLUTION,
                             config.ROBOT_RADIUS)
    controller = PurePursuitController(
        lookahead_distance=config.LOOKAHEAD_DISTANCE,
        v_cruise=config.V_CRUISE,
        goal_tolerance=config.GOAL_TOLERANCE,
    )

    start = WAREHOUSE_WAYPOINTS[0]
    robot = Robot(initial_pose=(start[0], start[1], 0.0))
    waypoints_to_visit = WAREHOUSE_WAYPOINTS[1:]

    # --- Perception / Localisation (Role 2) ---
    odometry = Odometry(robot)
    landmark_detector = LandmarkDetector(robot, WAREHOUSE_LANDMARKS)
    localizer = Localizer(initial_pose=robot.get_true_pose())

    random.seed(42 if planner_name == "astar" else 43)

    # --- Securite : intrusion (Role 1) ---
    known_obstacle_rects = [(o["x"], o["y"], o["w"], o["h"]) for o in WAREHOUSE_OBSTACLES]
    intrusion_detector = IntrusionDetector(robot, known_obstacles=known_obstacle_rects)
    alert_manager = AlertManager()
    speaker = Speaker()

    # --- Surete (Role 5) ---
    lidar = LidarSensor(robot, obstacles=WAREHOUSE_OBSTACLES)
    safety_manager = SafetyManager(tentatives_max_replanification=3)

    metrics = {
        "planner": planner_name,
        "waypoints_target": len(waypoints_to_visit),
        "waypoints_reached": 0,
        "success": False,
        "mission_time": 0.0,
        "safety_final_state": EtatSurete.NOMINAL.name,
        "safety_triggered": False,
        "safety_journal": [],
        "max_loc_error": 0.0,
        "intrusions_detected": 0,
        "max_alert_level": "nominal",
        "alarms_triggered": 0,
    }

    max_steps = int(max_time / dt)
    current_wp_idx = 0
    current_path = []
    path_found = True
    loc_errors = []

    if verbose:
        print(f"\n{'='*60}")
        print(f"  INTEGRATION FINALE — {planner_name.upper()} — entrepot")
        print(f"  {len(WAREHOUSE_WAYPOINTS)} waypoints, {len(WAREHOUSE_LANDMARKS)} balises, "
              f"{len(WAREHOUSE_OBSTACLES)} obstacles")
        print(f"{'='*60}")

    for step_i in range(max_steps):
        t = robot.time

        # === 1. Perception + Localisation EKF (Role 2) ===
        d_l, d_r = odometry.read(dt)
        localizer.predict(d_l, d_r)
        detections = landmark_detector.detect()
        localizer.correct(detections)
        est = localizer.estimated_pose

        true_x, true_y, _ = robot.get_true_pose()
        loc_errors.append(math.hypot(est.x - true_x, est.y - true_y))

        # === 2. Securite : detection d'intrusion (Role 1) ===
        cibles_visibles = [pos for (t_app, pos) in INTRUDER_SCHEDULE if t >= t_app]
        intrusion_confirmee, alertes = intrusion_detector.check(cibles_visibles, t)
        if intrusion_confirmee:
            metrics["intrusions_detected"] += 1
        alert_event = alert_manager.update(alertes, t)
        if alert_event.level.value != "nominal":
            niveaux = ["nominal", "info", "warning", "danger"]
            if niveaux.index(alert_event.level.value) > niveaux.index(metrics["max_alert_level"]):
                metrics["max_alert_level"] = alert_event.level.value
        speaker_event = speaker.update(alert_manager.should_alarm(), t)
        if speaker_event["event"] == "alarm_on":
            metrics["alarms_triggered"] += 1

        # === 3. Surete (Role 5) ===
        obstacle_distance = lidar.min_distance()
        etat = safety_manager.check(
            robot,
            localization_uncertainty=localizer.uncertainty,
            obstacle_distance=obstacle_distance,
            path_found=path_found,
            intrusion_confirmed=alert_manager.get_intrusion_confirmed(),
            intrusion_danger=alert_manager.is_danger(),
        )

        if etat == EtatSurete.ARRET_SUR:
            metrics["safety_triggered"] = True
            if verbose:
                print(f"  [t={t:.1f}s] ARRET_SUR declenche par SafetyManager "
                      f"(incertitude={localizer.uncertainty:.3f}m, "
                      f"obstacle={obstacle_distance:.3f}m)")
            break

        # === 4. Planification + replanification (Role 3) ===
        if current_wp_idx < len(waypoints_to_visit):
            goal = waypoints_to_visit[current_wp_idx]
            dist_to_wp = math.hypot(est.x - goal[0], est.y - goal[1])
            wp_reached = dist_to_wp <= config.GOAL_TOLERANCE and localizer.uncertainty < 0.3

            if wp_reached:
                metrics["waypoints_reached"] += 1
                if verbose:
                    print(f"  [t={t:.1f}s] WP{current_wp_idx + 2}/{len(WAREHOUSE_WAYPOINTS)} atteint "
                          f"(incertitude={localizer.uncertainty:.3f}m)")
                current_wp_idx += 1
                current_path = []
                if current_wp_idx >= len(waypoints_to_visit):
                    robot.set_velocity(0.0, 0.0)
                    robot.step(dt)
                    metrics["success"] = True
                    metrics["mission_time"] = round(t, 2)
                    break

            replan_interval = replan_every_n_steps
            if localizer.uncertainty > 0.2:
                replan_interval = max(20, replan_every_n_steps // 3)

            need_replan = (
                not current_path
                or controller.goal_reached((est.x, est.y), current_path)
                or (step_i > 0 and step_i % replan_interval == 0)
            )

            if need_replan:
                raw_path = planner.plan(start=(est.x, est.y), goal=goal)
                path_found = bool(raw_path)
                if not path_found:
                    if verbose:
                        print(f"  [t={t:.1f}s] PAS DE CHEMIN vers WP{current_wp_idx + 2} ! "
                              f"(depart estime=({est.x:.3f},{est.y:.3f}), "
                              f"vrai=({true_x:.3f},{true_y:.3f}))")
                    robot.set_velocity(0.0, 0.0)
                    robot.step(dt)
                    continue
                current_path = _resample_path(raw_path, max_segment=1.0)
                controller.reset()

        # === 5. Commande (Role 3) ===
        v, omega = controller.compute_command(pose=(est.x, est.y, est.theta), path=current_path)
        omega_lim = 1.5
        if abs(omega) > omega_lim:
            scale = omega_lim / abs(omega)
            omega *= scale
            v *= max(0.1, scale)

        robot.set_velocity(v, omega)
        robot.step(dt)

    metrics["safety_final_state"] = safety_manager.etat.name
    metrics["safety_journal"] = [
        (ev.t, ev.transition, ev.raison) for ev in safety_manager.journal
    ]
    if not metrics["success"]:
        metrics["mission_time"] = robot.time
    if loc_errors:
        metrics["max_loc_error"] = round(max(loc_errors), 4)
        metrics["mean_loc_error"] = round(sum(loc_errors) / len(loc_errors), 4)

    return metrics, robot


def _save_resume(all_metrics, filepath):
    _ensure_dir(filepath)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("INTEGRATION FINALE -- Role 4 (Dally)\n")
        f.write("Boucle complete : perception + localisation EKF + securite + surete "
                "+ planification + commande\n")
        f.write("=" * 65 + "\n\n")
        for name, m in all_metrics.items():
            f.write(f"--- {name.upper()} ---\n")
            for k, v in m.items():
                if k == "safety_journal":
                    continue
                f.write(f"  {k:<24}: {v}\n")
            if m["safety_journal"]:
                f.write("  journal_surete:\n")
                for t, transition, raison in m["safety_journal"]:
                    f.write(f"    t={t:.2f}s  {transition}  ({raison})\n")
            f.write("\n")
    print(f"  Resume : {filepath}")


def main(verbose=True):
    print("Integration finale -- perception + localisation + securite + surete "
          "+ planification + commande\n")
    results_dir = os.path.join(config.RESULTS_DIR, "features_integration_finale")

    all_metrics = {}
    for planner_name in ["astar", "rrt"]:
        m, robot = _run_patrol(planner_name, verbose=verbose)
        all_metrics[planner_name] = m
        print(f"\n--- {planner_name.upper()} ---")
        print(f"  Succes                : {m['success']}")
        print(f"  Waypoints atteints     : {m['waypoints_reached']}/{m['waypoints_target']}")
        print(f"  Temps mission (s)      : {m['mission_time']:.1f}")
        print(f"  Erreur loc. max (m)    : {m['max_loc_error']}")
        print(f"  Intrusions detectees   : {m['intrusions_detected']} pas")
        print(f"  Niveau d'alerte max    : {m['max_alert_level']}")
        print(f"  Alarmes declenchees    : {m['alarms_triggered']}")
        print(f"  Etat surete final      : {m['safety_final_state']}")
        print()

    _save_resume(all_metrics, os.path.join(results_dir, "resume_integration_finale.txt"))
    return all_metrics


if __name__ == "__main__":
    main()
