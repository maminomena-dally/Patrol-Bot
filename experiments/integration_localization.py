"""
experiments/integration_localization.py — Integration Role 2 (Perception/Localisation) + Role 3 (Planification/Commande).

Role 3 — Koja (apres depart de Kojy)

Cet experiment connecte pour la premiere fois la chaine complete :
    Odometry -> EKF predict -> LandmarkDetector -> EKF correct -> pose estimee
                                                                          |
                                                                      A* / RRT plan
                                                                          |
                                                                   Pure Pursuit control

Difference cle avec run_experiments.py : au lieu d'utiliser robot.get_true_pose()
(verite terrain), on utilise localizer.estimated_pose pour la planification et le
controle, comme dans un vrai robot.

Corrections appliquees au code de Kojy (Role 2) :
    - Bug fix dans localization/localization.py : theta n'etait jamais corrige
      dans la methode correct(), causant une derive non bornee du cap.

Ameliorations Phase 7 (Koja) :
    - Replanification periodique (tous les 100 steps) pour corriger la derive
    - Scenario perte de balise (8 vs 40 balises) pour tester la robustesse
    - Note : la pre-rotation a ete retiree car elle empire la derive avec un EKF simplifie
      (le cap estime est trop imprecis pour une rotation fiable sur place)

Lancer avec :
    python -m experiments.integration_localization
    ou :
    python experiments/integration_localization.py
"""

import math
import os
import csv
import time
import random

import config
from robot.robot import Robot
from simulation.simulator import Simulator
from sensors.odometry import Odometry
from sensors.landmarks import LandmarkDetector
from localization.localization import Localizer
from planning.astar import AStarPlanner, create_test_grid
from planning.rrt import RRTPlanner, grid_to_is_free
from control.pure_pursuit import PurePursuitController


# ======================================================================
# Carte et waypoints (memes que run_experiments.py)
# ======================================================================

PATROL_OBSTACLES = [
    {"type": "rect", "x": 9.5, "y": 0.0, "w": 0.3, "h": 10.0},
]

PATROL_WAYPOINTS = [
    (2.0, 2.0),
    (17.0, 2.0),
    (17.0, 12.0),
    (2.0, 12.0),
]

# Balises (landmarks) placees le long du perimetre de patrouille.
# Le robot doit pouvoir en voir au moins une a tout moment.
# Detection radius = 2.0m (config.LANDMARK_DETECTION_RADIUS)
# Strategie : une balise tous les ~3m le long des bords, plus quelques unes
# a l'interieur pres du parcours du robot.
LANDMARKS = [
    # --- Bords perimetre (le robot patrouille a ~2m des bords) ---
    {"id": 0,  "x": 0.5,  "y": 0.5},   # coin bas-gauche
    {"id": 1,  "x": 0.5,  "y": 2.0},   # bord gauche, hauteur WP1
    {"id": 2,  "x": 0.5,  "y": 4.0},   # bord gauche
    {"id": 3,  "x": 0.5,  "y": 6.0},   # bord gauche, centre
    {"id": 4,  "x": 0.5,  "y": 8.0},   # bord gauche
    {"id": 5,  "x": 0.5,  "y": 10.0},  # bord gauche
    {"id": 6,  "x": 0.5,  "y": 12.0},  # bord gauche, hauteur WP4
    {"id": 7,  "x": 0.5,  "y": 14.5},  # coin haut-gauche
    {"id": 8,  "x": 3.0,  "y": 0.5},   # bord bas
    {"id": 9,  "x": 6.0,  "y": 0.5},   # bord bas
    {"id": 10, "x": 9.0,  "y": 0.5},   # bord bas
    {"id": 11, "x": 12.0, "y": 0.5},  # bord bas
    {"id": 12, "x": 15.0, "y": 0.5},  # bord bas
    {"id": 13, "x": 19.5, "y": 0.5},  # coin bas-droit
    {"id": 14, "x": 19.5, "y": 2.0},  # bord droit, hauteur WP2
    {"id": 15, "x": 19.5, "y": 4.0},  # bord droit
    {"id": 16, "x": 19.5, "y": 6.0},  # bord droit, centre
    {"id": 17, "x": 19.5, "y": 8.0},  # bord droit
    {"id": 18, "x": 19.5, "y": 10.0}, # bord droit
    {"id": 19, "x": 19.5, "y": 12.0}, # bord droit, hauteur WP3
    {"id": 20, "x": 19.5, "y": 14.5}, # coin haut-droit
    {"id": 21, "x": 3.0,  "y": 14.5}, # bord haut
    {"id": 22, "x": 6.0,  "y": 14.5}, # bord haut
    {"id": 23, "x": 9.0,  "y": 14.5}, # bord haut
    {"id": 24, "x": 12.0, "y": 14.5}, # bord haut
    {"id": 25, "x": 15.0, "y": 14.5}, # bord haut
    # --- Interieur : pres des waypoints et des transitions ---
    {"id": 26, "x": 2.0,  "y": 2.0},   # WP1 exactement
    {"id": 27, "x": 17.0, "y": 2.0},  # WP2 exactement
    {"id": 28, "x": 17.0, "y": 12.0},  # WP3 exactement
    {"id": 29, "x": 2.0,  "y": 12.0},  # WP4 exactement
    {"id": 30, "x": 5.0,  "y": 2.0},   # transition basse
    {"id": 31, "x": 10.0, "y": 2.0},  # transition basse
    {"id": 32, "x": 14.0, "y": 2.0},  # transition basse
    {"id": 33, "x": 17.0, "y": 5.0},  # transition droite
    {"id": 34, "x": 17.0, "y": 8.0},  # transition droite
    {"id": 35, "x": 14.0, "y": 12.0}, # transition haute
    {"id": 36, "x": 10.0, "y": 12.0}, # transition haute
    {"id": 37, "x": 5.0,  "y": 12.0}, # transition haute
    {"id": 38, "x": 2.0,  "y": 8.0},  # transition gauche
    {"id": 39, "x": 2.0,  "y": 5.0},  # transition gauche
    # --- Phase 8 : balises pour combler les zones blanches ---
    # Cote droit x=17 : gaps entre y=2-5 et y=8-12
    {"id": 40, "x": 17.0, "y": 3.5},  # gap droit bas
    {"id": 41, "x": 17.0, "y": 10.0}, # gap droit haut
    # Cote gauche x=2 : gaps
    {"id": 42, "x": 2.0,  "y": 10.0}, # gap gauche haut
    {"id": 43, "x": 2.0,  "y": 3.5},  # gap gauche bas
    # Zone centrale (le robot y passe en contournant l'obstacle)
    {"id": 44, "x": 8.0,  "y": 2.0},   # transition basse avant obstacle
    {"id": 45, "x": 12.0, "y": 2.0},  # transition basse apres obstacle
    {"id": 46, "x": 8.0,  "y": 12.0},  # transition haute avant obstacle
    {"id": 47, "x": 12.0, "y": 12.0}, # transition haute apres obstacle
    # Contournement bas (y~1.5, le robot passe ici)
    {"id": 48, "x": 9.0,  "y": 1.5},   # bas avant obstacle
    {"id": 49, "x": 10.5, "y": 1.5}, # bas apres obstacle
    # Phase 8b : balises pour la montee/descente autour de l'obstacle
    # et les diagonales ou il n'y avait aucune couverture
    {"id": 50, "x": 3.0,  "y": 5.0},   # montee gauche
    {"id": 51, "x": 5.0,  "y": 7.0},   # montee gauche
    {"id": 52, "x": 7.0,  "y": 9.0},   # montee vers obstacle
    {"id": 53, "x": 9.0,  "y": 11.0},  # sommet obstacle gauche
    {"id": 54, "x": 11.0, "y": 11.0}, # sommet obstacle droite
    {"id": 55, "x": 13.0, "y": 9.0},  # descente droite
    {"id": 56, "x": 15.0, "y": 6.0},  # descente droite
    {"id": 57, "x": 16.0, "y": 4.0},  # approche WP2
    # Transition haute WP3->WP4 (le robot va de x=17 a x=2 a y=12)
    {"id": 58, "x": 15.0, "y": 12.0}, # chemin de retour
    {"id": 59, "x": 7.0,  "y": 12.0},  # chemin de retour
]

# Sous-ensemble de balises pour le scenario "perte de balise"
# (seulement les 4 aux coins + 4 au centre des bords = 8 balises)
LANDMARKS_REDUCED = [
    {"id": 0,  "x": 0.5,  "y": 0.5},
    {"id": 7,  "x": 0.5,  "y": 14.5},
    {"id": 13, "x": 19.5, "y": 0.5},
    {"id": 20, "x": 19.5, "y": 14.5},
    {"id": 3,  "x": 0.5,  "y": 6.0},
    {"id": 16, "x": 19.5, "y": 6.0},
    {"id": 10, "x": 9.0,  "y": 0.5},
    {"id": 23, "x": 9.0,  "y": 14.5},
]


# ======================================================================
# Utilitaires
# ======================================================================

def _path_length(path):
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        total += math.sqrt(dx * dx + dy * dy)
    return total


def _resample_path(path, max_segment=1.0):
    if len(path) < 2:
        return list(path)
    resampled = [path[0]]
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist <= max_segment:
            resampled.append(path[i])
        else:
            n = max(2, int(math.ceil(dist / max_segment)))
            for j in range(1, n + 1):
                t = j / n
                resampled.append((
                    path[i - 1][0] + t * dx,
                    path[i - 1][1] + t * dy,
                ))
    return resampled


def _normalize_angle(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi


def _ensure_dir(filepath):
    dirpath = os.path.dirname(filepath)
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)


def _make_planner(name, grid, resolution, robot_radius):
    if name == "astar":
        return AStarPlanner(
            grid,
            resolution=resolution,
            robot_radius=robot_radius,
            eight_connected=config.ASTAR_8_CONNECTED,
        )
    elif name == "rrt":
        is_free_fn = grid_to_is_free(grid, resolution, robot_radius)
        return RRTPlanner(
            is_free=is_free_fn,
            bounds=(0, 0, config.WORLD_WIDTH, config.WORLD_HEIGHT),
            robot_radius=robot_radius,
            step_size=config.RRT_STEP_SIZE,
            max_iter=config.RRT_MAX_ITER,
            goal_bias=config.RRT_GOAL_BIAS,
            goal_tolerance=config.RRT_GOAL_TOLERANCE,
            seed=42,
        )
    else:
        raise ValueError(f"Planificateur inconnu: {name}")


def _localization_step(robot, odometry, landmark_detector, localizer, dt):
    """Execute un cycle complet de localisation EKF.

    Returns:
        (est_x, est_y, est_theta, uncertainty, n_detections)
    """
    # 1. Odometry
    d_left, d_right = odometry.read(dt)
    # 2. EKF Predict
    localizer.predict(d_left, d_right)
    # 3. Landmark Detection
    detections = landmark_detector.detect()
    # 4. EKF Correct
    localizer.correct(detections)
    # 5. Recuperer la pose estimee
    est = localizer.estimated_pose
    return est.x, est.y, est.theta, localizer.uncertainty, len(detections)


# NOTE (Phase 7) : La pre-rotation a ete testee mais retiree.
# Avec un EKF simplifie (pas de matrice de covariance), le cap estime
# peut etre a 60+ degres de la realite. Tourner sur place base sur ce cap
# erronee empire la situation. La replanification frequente est plus
# robuste car Pure Pursuit ajuste progressivement le cap en boucle fermee.


# ======================================================================
# Scenario principal : Patrouille avec localisation EKF
# ======================================================================

def scenario_patrouille_localisee(planner_name="astar", verbose=False,
                                   landmarks=None, max_time=300.0,
                                   replan_every_n_steps=100):
    """Patrouille 4 waypoints en utilisant la POSE ESTIMEE (EKF) au lieu
    de la verite terrain.

    Chaine complete par iteration :
        1. Odometry.read(dt) -> d_left, d_right (bruites)
        2. Localizer.predict(d_left, d_right) -> maj pose estimee
        3. LandmarkDetector.detect() -> detections bruitees
        4. Localizer.correct(detections) -> maj pose estimee
        5. PurePursuit.compute_command(estimated_pose, path) -> v, omega
        6. Robot.set_velocity(v, omega) -> Robot.step(dt)

    Ameliorations Phase 7 :
        - Pre-rotation a chaque waypoint pour minimiser la derive de cap
        - Replanification periodique pour corriger la derive de position

    Args:
        planner_name: "astar" ou "rrt"
        verbose: afficher les details
        landmarks: liste de balises (defaut = LANDMARKS, 40 balises)
        max_time: temps max de simulation (s)
        replan_every_n_steps: replanifier tous les N pas pour corriger la derive

    Returns:
        dict avec metriques completes
    """
    dt = config.DT
    if landmarks is None:
        landmarks = LANDMARKS

    # --- Setup carte et planificateur ---
    grid = create_test_grid(
        config.WORLD_WIDTH, config.WORLD_HEIGHT,
        config.GRID_RESOLUTION, PATROL_OBSTACLES,
    )
    planner = _make_planner(planner_name, grid, config.GRID_RESOLUTION,
                            config.ROBOT_RADIUS)
    controller = PurePursuitController(
        lookahead_distance=config.LOOKAHEAD_DISTANCE,
        v_cruise=config.V_CRUISE,
        goal_tolerance=config.GOAL_TOLERANCE,
    )

    # --- Setup robot ---
    robot = Robot(initial_pose=config.INITIAL_POSE)

    # --- Setup perception/localisation (Role 2 - Kojy) ---
    odometry = Odometry(robot)
    landmark_detector = LandmarkDetector(robot, landmarks)

    # Seed pour reproductibilite (odometrie + bruit balises)
    # Doit etre APRES la creation du planificateur (RRT reseed a 42)
    random.seed(42 if planner_name == "astar" else 43)

    localizer = Localizer(
        initial_pose=config.INITIAL_POSE,
        wheel_base=config.WHEEL_BASE,
        process_noise=config.LOCALIZATION_PROCESS_NOISE,
        measurement_noise=config.LOCALIZATION_MEASUREMENT_NOISE,
    )

    # --- Metriques ---
    metrics = {
        "planner": planner_name,
        "n_landmarks": len(landmarks),
        "waypoints_target": len(PATROL_WAYPOINTS),
        "waypoints_reached": 0,
        "success": False,
        "mission_time": 0.0,
        "total_path_length": 0.0,
        "plan_times_ms": [],
        "replan_count": 0,
        "localization_errors": [],
        "theta_errors": [],
        "uncertainty_history": [],
        "detection_counts": [],
        "true_trajectory": [],
        "est_trajectory": [],
        "max_loc_error": 0.0,
        "mean_loc_error": 0.0,
        "max_theta_error": 0.0,
        "max_uncertainty": 0.0,
        "waypoint_errors": [],
    }

    max_steps = int(max_time / dt)
    current_wp_idx = 0
    current_path = []
    total_path_len = 0.0

    print(f"\n{'='*60}")
    print(f"  PATROUILLE LOCALISEE — {planner_name.upper()}")
    print(f"  Waypoints: {PATROL_WAYPOINTS}")
    print(f"  Balises: {len(landmarks)} landmarks, rayon={config.LANDMARK_DETECTION_RADIUS}m")
    print(f"  Replan tous les {replan_every_n_steps} steps")
    print(f"{'='*60}")

    for step_i in range(max_steps):
        t = robot.time

        # --- Localisation EKF ---
        est_x, est_y, est_theta, uncertainty, n_det = _localization_step(
            robot, odometry, landmark_detector, localizer, dt)

        # --- Verite terrain (pour metriques seulement) ---
        true_x, true_y, true_theta = robot.get_true_pose()

        # --- Metriques d'erreur de localisation ---
        loc_error = math.sqrt((est_x - true_x) ** 2 + (est_y - true_y) ** 2)
        theta_error = abs(_normalize_angle(est_theta - true_theta))
        metrics["localization_errors"].append(loc_error)
        metrics["theta_errors"].append(theta_error)
        metrics["uncertainty_history"].append(uncertainty)
        metrics["detection_counts"].append(n_det)
        metrics["true_trajectory"].append((true_x, true_y, true_theta))
        metrics["est_trajectory"].append((est_x, est_y, est_theta))

        # --- Gestion des waypoints ---
        if current_wp_idx < len(PATROL_WAYPOINTS):
            goal = PATROL_WAYPOINTS[current_wp_idx]

            # Verifier si on a atteint le waypoint
            dist_to_wp = math.sqrt((est_x - goal[0]) ** 2 + (est_y - goal[1]) ** 2)

            # Phase 8 : securite anti-fausse arrivee.
            # Ne declarer un WP atteint QUE si :
            #   1) La pose estimee est proche (GOAL_TOLERANCE)
            #   2) L'incertitude est raisonnable (< 0.3m)
            # Cela empeche le robot de declarer "arrive" quand l'EKF
            # a convergé vers la mauvaise position.
            wp_reached = dist_to_wp <= config.GOAL_TOLERANCE and uncertainty < 0.3

            if wp_reached:
                wp_error = math.sqrt((true_x - goal[0]) ** 2 + (true_y - goal[1]) ** 2)
                metrics["waypoint_errors"].append(wp_error)
                metrics["waypoints_reached"] += 1
                if verbose:
                    print(f"  [t={t:.1f}s] WP{current_wp_idx + 1} atteint ! "
                          f"(err vraie={wp_error:.3f}m, err estim={dist_to_wp:.3f}m, "
                          f"loc_err={loc_error:.3f}m)")
                current_wp_idx += 1

                if current_wp_idx >= len(PATROL_WAYPOINTS):
                    robot.set_velocity(0.0, 0.0)
                    robot.step(dt)
                    metrics["success"] = True
                    metrics["mission_time"] = t
                    break

                # Forcer la replanification vers le prochain waypoint
                current_path = []

            # Planifier si pas de chemin, premier segment, ou replanification periodique
            # Phase 8 : replanification adaptative — plus frequent si incertitude haute
            adaptive_replan_interval = replan_every_n_steps
            if uncertainty > 0.2:
                adaptive_replan_interval = max(20, replan_every_n_steps // 3)
            if uncertainty > 0.4:
                adaptive_replan_interval = max(10, replan_every_n_steps // 5)

            need_replan = (
                not current_path
                or controller.goal_reached((est_x, est_y), current_path)
                or (adaptive_replan_interval > 0 and step_i > 0
                    and step_i % adaptive_replan_interval == 0)
            )

            if need_replan:
                t0_plan = time.perf_counter()
                raw_path = planner.plan(start=(est_x, est_y), goal=goal)
                plan_ms = (time.perf_counter() - t0_plan) * 1000
                metrics["plan_times_ms"].append(plan_ms)

                if adaptive_replan_interval > 0 and step_i > 0 and step_i % adaptive_replan_interval == 0:
                    metrics["replan_count"] += 1

                if not raw_path:
                    if verbose:
                        print(f"  [t={t:.1f}s] PAS DE CHEMIN vers wp {current_wp_idx + 1} !")
                    robot.set_velocity(0.0, 0.0)
                    robot.step(dt)
                    continue

                current_path = _resample_path(raw_path, max_segment=1.0)
                total_path_len += _path_length(current_path)
                controller.reset()

                if verbose and (len(metrics["plan_times_ms"]) <= 3 or need_replan and step_i % replan_every_n_steps == 0):
                    print(f"  [t={t:.1f}s] Chemin: {len(current_path)} pts, "
                          f"long={_path_length(current_path):.1f}m, "
                          f"plan={plan_ms:.1f}ms")

        # --- Pure Pursuit Control (Role 3 - Koja) ---
        v, omega = controller.compute_command(
            pose=(est_x, est_y, est_theta),
            path=current_path,
        )

        # Phase 8 : Adaptation vitesse basee sur l'incertitude EKF
        # Ralentir progressivement quand l'incertitude augmente.
        # Le robot a besoin de temps pour que l'EKF corrige la pose.
        if uncertainty > 0.1:
            # Reduction douce : de 100% a 30% entre 0.1m et 0.5m d'incertitude
            u_ratio = min(1.0, (uncertainty - 0.1) / 0.4)
            v *= max(0.3, 1.0 - 0.7 * u_ratio)

        # --- Appliquer commande ---
        robot.set_velocity(v, omega)
        robot.step(dt)

        # --- Verifier limites carte ---
        if est_x < -0.5 or est_x > config.WORLD_WIDTH + 0.5 or \
           est_y < -0.5 or est_y > config.WORLD_HEIGHT + 0.5:
            if verbose:
                print(f"  [t={t:.1f}s] Robot sorti de la carte ! (est: {est_x:.1f}, {est_y:.1f})")
            break

    # --- Calculer metriques finales ---
    metrics["total_path_length"] = total_path_len
    if not metrics["success"]:
        metrics["mission_time"] = robot.time

    if metrics["localization_errors"]:
        metrics["max_loc_error"] = max(metrics["localization_errors"])
        metrics["mean_loc_error"] = sum(metrics["localization_errors"]) / len(metrics["localization_errors"])
    if metrics["theta_errors"]:
        metrics["max_theta_error"] = max(metrics["theta_errors"])
    if metrics["uncertainty_history"]:
        metrics["max_uncertainty"] = max(metrics["uncertainty_history"])

    return metrics


# ======================================================================
# Scenario : Perte de balises (robustesse)
# ======================================================================

def scenario_perte_balise(planner_name="astar", verbose=False):
    """Patrouille avec seulement 8 balises (au lieu de 40).

    Montre l'impact d'un environnement pauvre en balises sur la localisation
    et la navigation.
    """
    return scenario_patrouille_localisee(
        planner_name=planner_name,
        verbose=verbose,
        landmarks=LANDMARKS_REDUCED,
    )


# ======================================================================
# Comparaison : Vraie pose vs Pose estimee
# ======================================================================

def scenario_comparaison_locale(verbose=False):
    """Compare la patrouille avec pose vraie (ref) vs pose estimee (EKF).
    """
    from experiments.run_experiments import scenario_patrouille as _ref_patrouille

    print("\n" + "#" * 60)
    print("#  COMPARAISON : Vraie pose vs Pose estimee (A*)")
    print("#" * 60)

    ref = _ref_patrouille(planner_name="astar", verbose=False)
    est = scenario_patrouille_localisee(planner_name="astar", verbose=verbose)

    print(f"\n{'='*60}")
    print(f"  COMPARAISON FINALE")
    print(f"{'='*60}")
    print(f"  {'Metrique':<35} {'Vraie pose':>12} {'Pose estimee':>12}")
    print(f"  {'-'*56}")
    print(f"  {'Succes':<35} {'OUI' if ref.get('success') else 'NON':>12} {'OUI' if est['success'] else 'NON':>12}")
    print(f"  {'Waypoints atteints':<35} {ref.get('waypoints_reached', 'N/A'):>12} {est['waypoints_reached']:>12}")
    print(f"  {'Temps mission (s)':<35} {ref.get('mission_time', 0):>12.1f} {est['mission_time']:>12.1f}")
    print(f"  {'Longueur chemin (m)':<35} {ref.get('total_path_length', 0):>12.1f} {est['total_path_length']:>12.1f}")
    print(f"  {'---':<35}")
    print(f"  {'Erreur loc. max (m)':<35} {'N/A':>12} {est['max_loc_error']:>12.4f}")
    print(f"  {'Erreur loc. moy (m)':<35} {'N/A':>12} {est['mean_loc_error']:>12.4f}")
    print(f"  {'Erreur theta max (rad)':<35} {'N/A':>12} {est['max_theta_error']:>12.4f}")
    print(f"  {'Incertitude max (m)':<35} {'N/A':>12} {est['max_uncertainty']:>12.4f}")
    print(f"{'='*60}")

    return {"reference": ref, "estimee": est}


# ======================================================================
# Visualisation
# ======================================================================

def _plot_integration(metrics, obstacles, landmarks, waypoints, title, save_path):
    """Trace la trajectoire vraie vs estimee + balises + erreurs.
    """
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print(f"  [AVERTISSEMENT] matplotlib non disponible ({save_path})")
        return

    _ensure_dir(save_path)
    true_traj = metrics["true_trajectory"]
    est_traj = metrics["est_trajectory"]
    loc_errors = metrics["localization_errors"]
    uncertainty_hist = metrics["uncertainty_history"]
    detection_counts = metrics["detection_counts"]

    fig, axes = plt.subplots(2, 2, figsize=(16, 14))

    # --- 1. Trajectoires superposees ---
    ax = axes[0][0]
    ax.set_xlim(-0.5, config.WORLD_WIDTH + 0.5)
    ax.set_ylim(-0.5, config.WORLD_HEIGHT + 0.5)
    ax.set_aspect('equal')
    ax.set_title(f"Trajectoires : Vraie vs Estimee ({metrics['planner'].upper()})")
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')

    for obs in obstacles:
        if obs['type'] == 'rect':
            rect = patches.Rectangle(
                (obs['x'], obs['y']), obs['w'], obs['h'],
                linewidth=1.5, edgecolor='#333', facecolor='#999', alpha=0.7)
            ax.add_patch(rect)

    for lm in landmarks:
        circle = plt.Circle((lm['x'], lm['y']), config.LANDMARK_DETECTION_RADIUS,
                             fill=False, color='orange', linestyle='--', alpha=0.3, linewidth=0.5)
        ax.add_patch(circle)
        ax.plot(lm['x'], lm['y'], 's', color='orange', markersize=6, zorder=5)
    ax.plot([], [], 's', color='orange', markersize=6, label='Balises')

    if true_traj:
        tx = [p[0] for p in true_traj]
        ty = [p[1] for p in true_traj]
        ax.plot(tx, ty, '-', color='#2196F3', linewidth=1.0, alpha=0.8, label='Vraie pose')
    if est_traj:
        ex = [p[0] for p in est_traj]
        ey = [p[1] for p in est_traj]
        ax.plot(ex, ey, '-', color='#F44336', linewidth=1.0, alpha=0.6, label='Pose estimee')

    for i, wp in enumerate(waypoints):
        ax.plot(wp[0], wp[1], '^', color='green', markersize=12, zorder=10)
        ax.annotate(f'WP{i+1}', (wp[0] + 0.3, wp[1] + 0.3), fontsize=9, color='green')
    ax.plot([], [], '^', color='green', markersize=12, label='Waypoints')

    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- 2. Erreur de localisation ---
    ax = axes[0][1]
    if loc_errors:
        dt = config.DT
        times = [i * dt for i in range(len(loc_errors))]
        ax.plot(times, loc_errors, '-', color='#F44336', linewidth=0.8, label='Erreur position (m)')
        ax.axhline(y=config.GOAL_TOLERANCE, color='green', linestyle='--', alpha=0.5,
                   label=f'Goal tolerance ({config.GOAL_TOLERANCE}m)')
    ax.set_title("Erreur de localisation")
    ax.set_xlabel('Temps (s)')
    ax.set_ylabel('Erreur (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- 3. Incertitude EKF ---
    ax = axes[1][0]
    if uncertainty_hist:
        dt = config.DT
        times = [i * dt for i in range(len(uncertainty_hist))]
        ax.plot(times, uncertainty_hist, '-', color='#FF9800', linewidth=0.8,
                label='Incertitude EKF (m)')
        ax.axhline(y=config.LOCALIZATION_UNCERTAINTY_MAX, color='red', linestyle='--',
                   alpha=0.5, label=f'Seuil arret ({config.LOCALIZATION_UNCERTAINTY_MAX}m)')
    ax.set_title("Incertitude EKF")
    ax.set_xlabel('Temps (s)')
    ax.set_ylabel('Incertitude (m)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- 4. Nombre de balises detectees ---
    ax = axes[1][1]
    if detection_counts:
        dt = config.DT
        times = [i * dt for i in range(len(detection_counts))]
        ax.plot(times, detection_counts, '-', color='#4CAF50', linewidth=0.8)
        ax.fill_between(times, detection_counts, alpha=0.2, color='#4CAF50')
    ax.set_title("Balises detectees par step")
    ax.set_xlabel('Temps (s)')
    ax.set_ylabel("Nombre de balises")
    ax.grid(True, alpha=0.3)

    status = "Succes" if metrics["success"] else f"Echec ({metrics['waypoints_reached']}/{metrics['waypoints_target']} WP)"
    plt.suptitle(f"{title} [{status}]", fontsize=14, fontweight='bold', y=1.01)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"  [OK] Graphique : {save_path}")


def _save_csv(metrics, filepath):
    """Sauvegarde les metriques de localisation en CSV.
    """
    _ensure_dir(filepath)
    dt = config.DT
    true_traj = metrics["true_trajectory"]
    est_traj = metrics["est_trajectory"]
    loc_errors = metrics["localization_errors"]
    theta_errors = metrics["theta_errors"]
    uncertainty_hist = metrics["uncertainty_history"]
    detection_counts = metrics["detection_counts"]

    n = len(true_traj)
    with open(filepath, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time', 'true_x', 'true_y', 'true_theta',
            'est_x', 'est_y', 'est_theta',
            'loc_error_m', 'theta_error_rad',
            'uncertainty', 'n_detections'
        ])
        for i in range(n):
            writer.writerow([
                f"{i * dt:.3f}",
                f"{true_traj[i][0]:.4f}", f"{true_traj[i][1]:.4f}", f"{true_traj[i][2]:.4f}",
                f"{est_traj[i][0]:.4f}", f"{est_traj[i][1]:.4f}", f"{est_traj[i][2]:.4f}",
                f"{loc_errors[i]:.4f}", f"{theta_errors[i]:.4f}",
                f"{uncertainty_hist[i]:.4f}", f"{detection_counts[i]}",
            ])
    print(f"  [OK] CSV : {filepath}")


def _save_resume(metrics, filepath):
    """Sauvegarde un resume texte des resultats.
    """
    _ensure_dir(filepath)
    with open(filepath, 'w', encoding='utf-8') as f:
        f.write(f"Integration Role 2 + Role 3 — Resume\n")
        f.write(f"{'='*50}\n\n")
        f.write(f"Planificateur : {metrics['planner'].upper()}\n")
        f.write(f"Balises : {metrics.get('n_landmarks', 'N/A')}\n")
        f.write(f"Succes : {metrics['success']}\n")
        f.write(f"Waypoints atteints : {metrics['waypoints_reached']}/{metrics['waypoints_target']}\n")
        f.write(f"Temps de mission : {metrics['mission_time']:.1f}s\n")
        f.write(f"Longueur chemin total : {metrics['total_path_length']:.1f}m\n")
        f.write(f"Replanifications : {metrics.get('replan_count', 0)}\n\n")
        f.write(f"--- Metriques de localisation ---\n")
        f.write(f"Erreur position max : {metrics['max_loc_error']:.4f} m\n")
        f.write(f"Erreur position moy : {metrics['mean_loc_error']:.4f} m\n")
        f.write(f"Erreur theta max : {metrics['max_theta_error']:.4f} rad ({math.degrees(metrics['max_theta_error']):.1f} deg)\n")
        f.write(f"Incertitude max : {metrics['max_uncertainty']:.4f} m\n\n")
        if metrics['waypoint_errors']:
            f.write(f"--- Erreur vraie a chaque waypoint ---\n")
            for i, err in enumerate(metrics['waypoint_errors']):
                f.write(f"  WP{i+1}: {err:.4f} m\n")
    print(f"  [OK] Resume : {filepath}")


# ======================================================================
# Point d'entree principal
# ======================================================================

def run_all(verbose=True):
    """Lance tous les scenarios d'integration et sauvegarde les resultats."""
    results_dir = os.path.join(config.RESULTS_DIR, "features_integration")
    images_dir = os.path.join(results_dir, "images")
    logs_dir = os.path.join(results_dir, "logs")
    os.makedirs(images_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)

    all_results = {}

    # --- Scenario 1 : Patrouille A* avec localisation ---
    print("\n" + "*" * 60)
    print("*  SCENARIO 1 : Patrouille A* + EKF Localisation (40 balises)")
    print("*" * 60)
    res_astar = scenario_patrouille_localisee("astar", verbose=verbose)
    all_results["astar"] = res_astar

    _save_csv(res_astar, os.path.join(logs_dir, "patrol_localized_astar.csv"))
    _save_resume(res_astar, os.path.join(results_dir, "resume_integration.txt"))
    _plot_integration(
        res_astar, PATROL_OBSTACLES, LANDMARKS, PATROL_WAYPOINTS,
        f"Patrouille A* + EKF (40 balises)",
        os.path.join(images_dir, "integration_astar.png"),
    )

    # --- Scenario 2 : Patrouille RRT avec localisation ---
    print("\n" + "*" * 60)
    print("*  SCENARIO 2 : Patrouille RRT + EKF Localisation (40 balises)")
    print("*" * 60)
    res_rrt = scenario_patrouille_localisee("rrt", verbose=verbose)
    all_results["rrt"] = res_rrt

    _save_csv(res_rrt, os.path.join(logs_dir, "patrol_localized_rrt.csv"))
    _save_resume(res_rrt, os.path.join(results_dir, "resume_rrt_localisee.txt"))
    _plot_integration(
        res_rrt, PATROL_OBSTACLES, LANDMARKS, PATROL_WAYPOINTS,
        f"Patrouille RRT + EKF (40 balises)",
        os.path.join(images_dir, "integration_rrt.png"),
    )

    # --- Scenario 3 : Perte de balise (seulement 8 balises) ---
    print("\n" + "*" * 60)
    print("*  SCENARIO 3 : Perte de balise — A* (8 balises au lieu de 40)")
    print("*" * 60)
    res_perte_a = scenario_perte_balise("astar", verbose=verbose)
    all_results["perte_balise_astar"] = res_perte_a

    _save_csv(res_perte_a, os.path.join(logs_dir, "perte_balise_astar.csv"))
    _save_resume(res_perte_a, os.path.join(results_dir, "resume_perte_balise.txt"))
    _plot_integration(
        res_perte_a, PATROL_OBSTACLES, LANDMARKS_REDUCED, PATROL_WAYPOINTS,
        f"Patrouille A* + EKF (8 balises — perte)",
        os.path.join(images_dir, "perte_balise_astar.png"),
    )

    # --- Scenario 4 : Perte de balise RRT ---
    print("\n" + "*" * 60)
    print("*  SCENARIO 4 : Perte de balise — RRT (8 balises au lieu de 40)")
    print("*" * 60)
    res_perte_r = scenario_perte_balise("rrt", verbose=verbose)
    all_results["perte_balise_rrt"] = res_perte_r

    _save_csv(res_perte_r, os.path.join(logs_dir, "perte_balise_rrt.csv"))
    _plot_integration(
        res_perte_r, PATROL_OBSTACLES, LANDMARKS_REDUCED, PATROL_WAYPOINTS,
        f"Patrouille RRT + EKF (8 balises — perte)",
        os.path.join(images_dir, "perte_balise_rrt.png"),
    )

    # --- Resume global ---
    print(f"\n{'#'*60}")
    print(f"#  RESUME GLOBAL DE L'INTEGRATION")
    print(f"#{' '*58}#")
    for name, res in all_results.items():
        status = "OK" if res["success"] else "ECHEC"
        print(f"#  {name:>25} : {status} — {res['waypoints_reached']}/{res['waypoints_target']} WP, "
              f"err max={res['max_loc_error']:.3f}m, moy={res['mean_loc_error']:.3f}m")
    print(f"#")
    print(f"#  Resultats dans : {results_dir}/")
    print(f"#{' '*58}#")
    print(f"{'#'*60}\n")

    return all_results


if __name__ == "__main__":
    run_all(verbose=True)
