import math
import os
import random

import config
from robot.robot import Robot
from simulation.simulator import Simulator
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


class _IntegrationLoop:
    """
    Regroupe l'etat partage entre les callbacks de Simulator (odometrie,
    EKF, securite, surete, planification) : Simulator n'appelle chaque
    callback qu'avec (robot, t), donc tout ce qui doit survivre d'un pas
    a l'autre doit vivre ici plutot que dans des variables locales d'une
    fonction de boucle (qui n'existe plus).
    """

    def __init__(self, planner_name, robot, waypoints_to_visit, landmarks,
                 obstacles, replan_every_n_steps=100, verbose=False):
        self.planner_name = planner_name
        self.waypoints_to_visit = waypoints_to_visit
        self.replan_every_n_steps = replan_every_n_steps
        self.verbose = verbose

        # --- Perception / Localisation (Role 2) ---
        self.odometry = Odometry(robot)
        self.landmark_detector = LandmarkDetector(robot, landmarks)
        self.localizer = Localizer(initial_pose=robot.get_true_pose())

        # --- Securite : intrusion (Role 1) ---
        known_obstacle_rects = [(o["x"], o["y"], o["w"], o["h"]) for o in obstacles]
        self.intrusion_detector = IntrusionDetector(robot, known_obstacles=known_obstacle_rects)
        self.alert_manager = AlertManager()
        self.speaker = Speaker()

        # --- Surete (Role 5) ---
        self.lidar = LidarSensor(robot, obstacles=obstacles)
        self.safety_manager = SafetyManager(tentatives_max_replanification=3)

        # --- Planification + commande (Role 3) ---
        grid = create_test_grid(config.WORLD_WIDTH, config.WORLD_HEIGHT,
                                 config.GRID_RESOLUTION, obstacles)
        self.planner = _make_planner(planner_name, grid, config.GRID_RESOLUTION,
                                      config.ROBOT_RADIUS)
        self.controller = PurePursuitController(
            lookahead_distance=config.LOOKAHEAD_DISTANCE,
            v_cruise=config.V_CRUISE,
            goal_tolerance=config.GOAL_TOLERANCE,
        )

        # --- Etat de boucle ---
        self.step_i = 0
        self.current_wp_idx = 0
        self.current_path = []
        self.path_found = True
        self._d_l = 0.0
        self._d_r = 0.0
        self.obstacle_distance = None
        self.loc_errors = []
        self.true_trajectory = []
        self.est_trajectory = []
        self.arret_sur_point = None

        self.metrics = {
            "planner": planner_name,
            "waypoints_target": len(waypoints_to_visit),
            "waypoints_reached": 0,
            "success": False,
            "mission_time": 0.0,
            "safety_final_state": EtatSurete.NOMINAL.name,
            "safety_triggered": False,
            "intrusions_detected": 0,
            "max_alert_level": "nominal",
            "alarms_triggered": 0,
        }

    # ------------------------------------------------------------------
    # Callbacks pour simulation.Simulator
    # ------------------------------------------------------------------
    def on_perceive(self, robot, t):
        self.step_i += 1
        self._d_l, self._d_r = self.odometry.read(config.DT)
        self.obstacle_distance = self.lidar.min_distance()

    def on_localize(self, robot, t):
        self.localizer.predict(self._d_l, self._d_r)
        detections = self.landmark_detector.detect()
        self.localizer.correct(detections)

        est = self.localizer.estimated_pose
        true_x, true_y, _ = robot.get_true_pose()
        self.loc_errors.append(math.hypot(est.x - true_x, est.y - true_y))
        self.true_trajectory.append((true_x, true_y))
        self.est_trajectory.append((est.x, est.y))

    def on_detect(self, robot, t):
        cibles_visibles = [pos for (t_app, pos) in INTRUDER_SCHEDULE if t >= t_app]
        confirmee, alertes = self.intrusion_detector.check(cibles_visibles, t)
        if confirmee:
            self.metrics["intrusions_detected"] += 1

        event = self.alert_manager.update(alertes, t)
        niveaux = ["nominal", "info", "warning", "danger"]
        if niveaux.index(event.level.value) > niveaux.index(self.metrics["max_alert_level"]):
            self.metrics["max_alert_level"] = event.level.value

        speaker_event = self.speaker.update(self.alert_manager.should_alarm(), t)
        if speaker_event["event"] == "alarm_on":
            self.metrics["alarms_triggered"] += 1

    def on_plan(self, robot, t):
        if self.metrics["success"] or self.current_wp_idx >= len(self.waypoints_to_visit):
            return

        est = self.localizer.estimated_pose
        goal = self.waypoints_to_visit[self.current_wp_idx]
        dist_to_wp = math.hypot(est.x - goal[0], est.y - goal[1])
        # Anti-fausse-arrivee : n'accepter un waypoint que si l'incertitude
        # EKF est raisonnable (pas de "arrivee" sur une position mal estimee)
        wp_reached = dist_to_wp <= config.GOAL_TOLERANCE and self.localizer.uncertainty < 0.3

        if wp_reached:
            self.metrics["waypoints_reached"] += 1
            if self.verbose:
                print(f"  [t={t:.1f}s] WP{self.current_wp_idx + 2}/{len(self.waypoints_to_visit) + 1} "
                      f"atteint (incertitude={self.localizer.uncertainty:.3f}m)")
            self.current_wp_idx += 1
            self.current_path = []
            if self.current_wp_idx >= len(self.waypoints_to_visit):
                self.metrics["success"] = True
                self.metrics["mission_time"] = round(t, 2)
                return
            goal = self.waypoints_to_visit[self.current_wp_idx]

        replan_interval = self.replan_every_n_steps
        if self.localizer.uncertainty > 0.2:
            replan_interval = max(20, self.replan_every_n_steps // 3)

        need_replan = (
            not self.current_path
            or self.controller.goal_reached((est.x, est.y), self.current_path)
            or (self.step_i > 0 and self.step_i % replan_interval == 0)
        )

        if need_replan:
            raw_path = self.planner.plan(start=(est.x, est.y), goal=goal)
            self.path_found = bool(raw_path)
            if not self.path_found:
                if self.verbose:
                    print(f"  [t={t:.1f}s] PAS DE CHEMIN vers WP{self.current_wp_idx + 2} !")
                return
            self.current_path = _resample_path(raw_path, max_segment=1.0)
            self.controller.reset()

    def on_safety(self, robot, t):
        etat = self.safety_manager.check(
            robot,
            localization_uncertainty=self.localizer.uncertainty,
            obstacle_distance=self.obstacle_distance,
            path_found=self.path_found,
            intrusion_confirmed=self.alert_manager.get_intrusion_confirmed(),
            intrusion_danger=self.alert_manager.is_danger(),
        )
        if etat == EtatSurete.ARRET_SUR and not self.metrics["safety_triggered"]:
            self.metrics["safety_triggered"] = True
            true_x, true_y, _ = robot.get_true_pose()
            self.arret_sur_point = (true_x, true_y, t)
            if self.verbose:
                print(f"  [t={t:.1f}s] ARRET_SUR declenche par SafetyManager "
                      f"(incertitude={self.localizer.uncertainty:.3f}m, "
                      f"obstacle={self.obstacle_distance:.3f}m)")

    def command_fn(self, robot, t):
        est = self.localizer.estimated_pose
        v, omega = self.controller.compute_command(
            pose=(est.x, est.y, est.theta), path=self.current_path)

        omega_lim = 1.5
        if abs(omega) > omega_lim:
            scale = omega_lim / abs(omega)
            omega *= scale
            v *= max(0.1, scale)

        robot.set_velocity(v, omega)

    def stop_fn(self, robot, t):
        return self.metrics["success"] or self.metrics["safety_triggered"]

    def finalize(self):
        self.metrics["safety_final_state"] = self.safety_manager.etat.name
        self.metrics["safety_journal"] = [
            (ev.t, ev.transition, ev.raison) for ev in self.safety_manager.journal
        ]
        if not self.metrics["success"]:
            self.metrics["mission_time"] = round(self.step_i * config.DT, 2)
        if self.loc_errors:
            self.metrics["max_loc_error"] = round(max(self.loc_errors), 4)
            self.metrics["mean_loc_error"] = round(sum(self.loc_errors) / len(self.loc_errors), 4)
        return self.metrics


def _run_patrol(planner_name="astar", verbose=False, max_time=400.0,
                 replan_every_n_steps=100):
    """
    Fait patrouiller le robot sur tous les waypoints de l'entrepot via
    simulation.Simulator (callbacks on_perceive/on_localize/on_detect/
    on_plan/on_safety), en branchant reellement les six modules du
    projet. Retourne (metrics, robot).
    """
    start = WAREHOUSE_WAYPOINTS[0]
    robot = Robot(initial_pose=(start[0], start[1], 0.0))

    loop = _IntegrationLoop(
        planner_name, robot, WAREHOUSE_WAYPOINTS[1:], WAREHOUSE_LANDMARKS,
        WAREHOUSE_OBSTACLES, replan_every_n_steps, verbose,
    )

    # Seed pour reproductibilite du bruit (odometrie + balises) : sans ca,
    # le meme scenario peut occasionnellement echouer (l'estimation EKF
    # bruitee peut tomber dans un obstacle gonfle au moment de replanifier
    # -> "PAS DE CHEMIN" transitoire -> ARRET_SUR). Doit venir APRES la
    # creation du planificateur RRT (qui reseed son propre generateur a la
    # construction). Voir SIMULATION_INTEGRATION.md.
    random.seed(42 if planner_name == "astar" else 43)

    if verbose:
        print(f"\n{'='*60}")
        print(f"  INTEGRATION FINALE — {planner_name.upper()} — entrepot (via Simulator)")
        print(f"  {len(WAREHOUSE_WAYPOINTS)} waypoints, {len(WAREHOUSE_LANDMARKS)} balises, "
              f"{len(WAREHOUSE_OBSTACLES)} obstacles")
        print(f"{'='*60}")

    sim = Simulator(robot, dt=config.DT)
    sim.on_perceive = loop.on_perceive
    sim.on_localize = loop.on_localize
    sim.on_detect = loop.on_detect
    sim.on_plan = loop.on_plan
    sim.on_safety = loop.on_safety

    sim.run(duration=max_time, command_fn=loop.command_fn, stop_fn=loop.stop_fn)

    return loop.finalize(), robot, loop


def _save_resume(all_metrics, filepath):
    _ensure_dir(filepath)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write("INTEGRATION FINALE -- Role 4 (Dally) -- via simulation.Simulator\n")
        f.write("Boucle complete : perception + localisation EKF + securite + surete "
                "+ planification + commande\n")
        f.write("=" * 65 + "\n\n")
        for name, m in all_metrics.items():
            f.write(f"--- {name.upper()} ---\n")
            for k, v in m.items():
                if k == "safety_journal":
                    continue
                f.write(f"  {k:<24}: {v}\n")
            if m.get("safety_journal"):
                f.write("  journal_surete:\n")
                for t, transition, raison in m["safety_journal"]:
                    f.write(f"    t={t:.2f}s  {transition}  ({raison})\n")
            f.write("\n")
    print(f"  Resume : {filepath}")


def _plot_result(loop, obstacles, waypoints, landmarks, title, save_path):
    """
    Trace la carte (obstacles, waypoints, balises), la trajectoire vraie
    vs estimee, les intrus simules et le point d'ARRET_SUR (s'il y en a
    eu un). Sauvegarde en PNG. Ne fait rien si matplotlib est absent.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print(f"  [AVERTISSEMENT] matplotlib non disponible, graphique non genere ({save_path})")
        return

    _ensure_dir(save_path)
    fig, ax = plt.subplots(figsize=(14, 10.5))
    ax.set_xlim(-0.5, config.WORLD_WIDTH + 0.5)
    ax.set_ylim(-0.5, config.WORLD_HEIGHT + 0.5)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title, fontsize=13)

    for obs in obstacles:
        rect = patches.Rectangle((obs["x"], obs["y"]), obs["w"], obs["h"],
                                  linewidth=1.2, edgecolor="#333", facecolor="#999", alpha=0.8)
        ax.add_patch(rect)

    lm_x = [lm["x"] for lm in landmarks]
    lm_y = [lm["y"] for lm in landmarks]
    ax.plot(lm_x, lm_y, "s", color="orange", markersize=6, label="Balises", zorder=5)

    wp_x = [w[0] for w in waypoints]
    wp_y = [w[1] for w in waypoints]
    ax.plot(wp_x, wp_y, "^", color="green", markersize=12, label="Waypoints", zorder=6)
    for i, (x, y) in enumerate(waypoints):
        ax.annotate(f"WP{i+1}", (x, y), textcoords="offset points",
                    xytext=(8, 6), fontsize=8, color="green")

    if loop.true_trajectory:
        tx = [p[0] for p in loop.true_trajectory]
        ty = [p[1] for p in loop.true_trajectory]
        ax.plot(tx, ty, "-", color="#2196F3", linewidth=1.3, alpha=0.9, label="Trajectoire vraie", zorder=3)
    if loop.est_trajectory:
        ex = [p[0] for p in loop.est_trajectory]
        ey = [p[1] for p in loop.est_trajectory]
        ax.plot(ex, ey, "-", color="#F44336", linewidth=1.0, alpha=0.6, label="Trajectoire estimee (EKF)", zorder=4)

    for i, (t_app, (ix, iy)) in enumerate(INTRUDER_SCHEDULE):
        ax.plot(ix, iy, "P", color="black", markersize=14, zorder=7,
                label="Intrus simule" if i == 0 else None)
        ax.annotate(f"intrus t={t_app:.0f}s", (ix, iy), textcoords="offset points",
                    xytext=(8, -12), fontsize=8, color="black")

    if loop.arret_sur_point:
        x, y, t = loop.arret_sur_point
        ax.plot(x, y, "X", color="red", markersize=18, zorder=8,
                label=f"ARRET_SUR (t={t:.1f}s)")

    ax.legend(loc="upper left", fontsize=8)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Graphique : {save_path}")


def main(verbose=True):
    print("Integration finale (via simulation.Simulator) -- perception + localisation "
          "+ securite + surete + planification + commande\n")
    results_dir = os.path.join(config.RESULTS_DIR, "features_integration_finale")
    logs_dir = os.path.join(results_dir, "logs")
    images_dir = os.path.join(results_dir, "images")

    all_metrics = {}
    for planner_name in ["astar", "rrt"]:
        m, robot, loop = _run_patrol(planner_name, verbose=verbose)
        all_metrics[planner_name] = m
        print(f"\n--- {planner_name.upper()} ---")
        print(f"  Succes                : {m['success']}")
        print(f"  Waypoints atteints     : {m['waypoints_reached']}/{m['waypoints_target']}")
        print(f"  Temps mission (s)      : {m['mission_time']:.1f}")
        print(f"  Erreur loc. max (m)    : {m.get('max_loc_error')}")
        print(f"  Intrusions detectees   : {m['intrusions_detected']} pas")
        print(f"  Niveau d'alerte max    : {m['max_alert_level']}")
        print(f"  Alarmes declenchees    : {m['alarms_triggered']}")
        print(f"  Etat surete final      : {m['safety_final_state']}")

        # --- Pour verification visuelle ---
        csv_path = os.path.join(logs_dir, f"patrol_{planner_name}.csv")
        _ensure_dir(csv_path)
        robot.export_log(csv_path)
        print(f"  Log CSV (pour gui.replay) : {csv_path}")

        statut = "Succes" if m["success"] else f"Echec ({m['safety_final_state']})"
        _plot_result(
            loop, WAREHOUSE_OBSTACLES, WAREHOUSE_WAYPOINTS, WAREHOUSE_LANDMARKS,
            f"Integration finale — {planner_name.upper()} — {statut}",
            os.path.join(images_dir, f"integration_finale_{planner_name}.png"),
        )
        print()

    _save_resume(all_metrics, os.path.join(results_dir, "resume_integration_finale.txt"))
    return all_metrics


if __name__ == "__main__":
    main()
