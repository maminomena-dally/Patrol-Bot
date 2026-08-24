"""
experiments/run_experiments.py — À COMPLÉTER par toute l'équipe au fur et
à mesure que les modules sont intégrés.

Rôle attendu (section 19 du cahier des charges) :
    Lancer une série de scénarios de test reproductibles (patrouille
    normale, obstacle imprévu, intrusion, perte de localisation...) et
    sauvegarder les résultats dans config.RESULTS_DIR pour analyse.

Pour l'instant, ce fichier ne contient qu'un exemple minimal basé sur le
module Système/Cinématique déjà fonctionnel (voir main.py à la racine
pour une démonstration plus complète et commentée).
"""


"""
=============== Role 3 (Koja) a ajoute : ================================
    - scenario_patrouille()         : patrouille 4 waypoints
    - scenario_replanification()   : obstacle imprevu en cours de route
    - scenario_comparaison()       : A* vs RRT sur les 2 scenarios
    - Visualisation matplotlib + export CSV dans results/features_planning/

Lancer avec :
    python -m experiments.run_experiments
    ou :
    python experiments/run_experiments.py
========================== fin Role 3 =====================================
"""

import math
import os
import time

import config
from robot.robot import Robot
from simulation.simulator import Simulator
from planning.astar import AStarPlanner, create_test_grid
from planning.rrt import RRTPlanner, grid_to_is_free
from control.pure_pursuit import PurePursuitController


# ======================================================================
# Carte et waypoints (20m x 15m, resolution 0.1m)
# ======================================================================

# Obstacle fixe de la carte : mur vertical au centre.
# Le robot doit contourner par le haut (y > 10) pour traverser.
PATROL_OBSTACLES = [
    {"type": "rect", "x": 9.5, "y": 0.0, "w": 0.3, "h": 10.0},
]

# 4 points de patrouille (perimetre, marge de 1-3m des bords)
PATROL_WAYPOINTS = [
    (2.0, 2.0),
    (17.0, 2.0),
    (17.0, 12.0),
    (2.0, 12.0),
]


# ======================================================================
# Utilitaires
# ======================================================================

def _path_length(path):
    """Longueur totale d'un chemin en metres."""
    if len(path) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path)):
        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]
        total += math.sqrt(dx * dx + dy * dy)
    return total


def _min_dist_to_rects(x, y, obstacles, margin=0.0):
    """Distance min d'un point a des obstacles rectangles."""
    min_d = float("inf")
    for obs in obstacles:
        if obs["type"] == "rect":
            cx = max(obs["x"], min(x, obs["x"] + obs["w"]))
            cy = max(obs["y"], min(y, obs["y"] + obs["h"]))
            d = math.sqrt((x - cx) ** 2 + (y - cy) ** 2) - margin
            if d < min_d:
                min_d = d
    return max(0.0, min_d)


def _resample_path(path, max_segment=1.0):
    """Ajouter des points intermediaires pour que chaque segment <= max_segment.

    Le lissage A*/RRT peut produire des chemins avec des segments
    de 10m+. Pure Pursuit suit mal ces longs segments.
    Cette fonction ressample le chemin tous les max_segment metres.
    """
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


def _make_planner(name, grid, resolution, robot_radius):
    """Creer un planificateur A* ou RRT."""
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


def _follow_path(robot, controller, path, max_time=120.0):
    """Suivre un chemin avec Pure Pursuit jusqu'au but ou timeout.

    1. Pre-rotation : le robot tourne sur place pour faire face au chemin.
    2. Suivi : Pure Pursuit avec ralentissement dans les virages.
    3. Arret si le robot sort de la carte.

    Returns:
        (reached: bool, steps: int, final_dist: float)
    """
    dt = config.DT
    max_steps = int(max_time / dt)
    goal = path[-1]
    x_max = config.WORLD_WIDTH
    y_max = config.WORLD_HEIGHT

    # --- Pre-rotation : faire face au debut du chemin ---
    if len(path) >= 2:
        # Viser quelques points ahead pour eviter les zigzags
        ahead = min(5, len(path) - 1)
        dx = path[ahead][0] - path[0][0]
        dy = path[ahead][1] - path[0][1]
        if dx * dx + dy * dy > 0.01:
            target_th = math.atan2(dy, dx)
            for _ in range(500):  # max 25s
                px, py, pth = robot.get_true_pose()
                err = target_th - pth
                while err > math.pi:
                    err -= 2 * math.pi
                while err < -math.pi:
                    err += 2 * math.pi
                if abs(err) < 0.05:
                    break
                omega = 1.0 if err > 0 else -1.0
                robot.set_velocity(0.0, omega)
                robot.step(dt)
            robot.set_velocity(0.0, 0.0)

    # --- Suivi Pure Pursuit ---
    for step_i in range(max_steps):
        px, py, pth = robot.get_true_pose()
        if px < 0 or px > x_max or py < 0 or py > y_max:
            d = math.sqrt((px - goal[0]) ** 2 + (py - goal[1]) ** 2)
            return False, step_i, d
        if controller.goal_reached((px, py), path):
            return True, step_i, 0.0
        v, omega = controller.compute_command(pose=(px, py, pth), path=path)
        # Ralentir pendant les virages serres
        omega_limit = 0.8
        if abs(omega) > omega_limit:
            scale = omega_limit / abs(omega)
            omega *= scale
            v *= max(0.1, scale)
        robot.set_velocity(v, omega)
        robot.step(dt)
    px, py, _ = robot.get_true_pose()
    d = math.sqrt((px - goal[0]) ** 2 + (py - goal[1]) ** 2)
    return False, max_steps, d


def _ensure_dir(filepath):
    """Cree les repertoires parents si necessaires."""
    dirpath = os.path.dirname(filepath)
    if dirpath:
        os.makedirs(dirpath, exist_ok=True)


def _plot_trajectory(robot_history, obstacles, waypoints, title, save_path,
                     extra_obstacles=None):
    """Trace la trajectoire du robot sur la carte avec matplotlib.

    Sauvegarde en PNG. matplotlib doit etre installe, sinon
    affiche un avertissement et passe.
    """
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print(f"  [AVERTISSEMENT] matplotlib non disponible, "
              f"graphique non genere ({save_path})")
        return

    _ensure_dir(save_path)

    fig, ax = plt.subplots(figsize=(14, 10.5))
    ax.set_xlim(-0.5, config.WORLD_WIDTH + 0.5)
    ax.set_ylim(-0.5, config.WORLD_HEIGHT + 0.5)
    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title, fontsize=13)

    # --- Obstacles fixes (gris) ---
    for obs in obstacles:
        if obs['type'] == 'rect':
            rect = patches.Rectangle(
                (obs['x'], obs['y']), obs['w'], obs['h'],
                linewidth=1.5, edgecolor='#333', facecolor='#999')
            ax.add_patch(rect)

    # --- Obstacle(s) imprévu(s) (rouge, pointillé) ---
    if extra_obstacles:
        for obs in extra_obstacles:
            if obs['type'] == 'rect':
                rect = patches.Rectangle(
                    (obs['x'], obs['y']), obs['w'], obs['h'],
                    linewidth=2, edgecolor='red', facecolor='#ff9999',
                    linestyle='--', label='Obstacle imprévu')
                ax.add_patch(rect)

    # --- Waypoints (triangles verts) ---
    if waypoints:
        wx = [w[0] for w in waypoints]
        wy = [w[1] for w in waypoints]
        ax.plot(wx, wy, 'g^', markersize=14, label='Waypoints', zorder=5)
        for i, (x, y) in enumerate(waypoints):
            ax.annotate(f'WP{i+1}', (x, y), textcoords="offset points",
                       xytext=(10, 8), fontsize=10, fontweight='bold',
                       color='green')
        # Ligne pointillée entre waypoints (ordre de patrouille)
        for i in range(len(waypoints)):
            j = (i + 1) % len(waypoints)
            ax.plot([waypoints[i][0], waypoints[j][0]],
                   [waypoints[i][1], waypoints[j][1]],
                   'g--', alpha=0.3, linewidth=1)

    # --- Trajectoire du robot (ligne bleue) ---
    if robot_history:
        xs = [s['x'] for s in robot_history]
        ys = [s['y'] for s in robot_history]
        ax.plot(xs, ys, 'b-', linewidth=1.5, label='Trajectoire',
                alpha=0.8, zorder=3)
        ax.plot(xs[0], ys[0], 'go', markersize=12, label='Départ',
                zorder=6)
        ax.plot(xs[-1], ys[-1], 'rs', markersize=12, label='Arrivée',
                zorder=6)

    ax.legend(loc='upper left', fontsize=10)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Graphique : {save_path}")


def _save_comparison_txt(patrol, replan, filepath):
    """Sauvegarde le tableau comparatif dans un fichier texte."""
    _ensure_dir(filepath)

    a = patrol["astar"]
    rr = patrol["rrt"]
    ra = replan["astar"]
    rrr = replan["rrt"]
    a_avg = a["total_plan_time_ms"] / max(1, len(a["plan_times_ms"]))
    r_avg = rr["total_plan_time_ms"] / max(1, len(rr["plan_times_ms"]))

    lines = []
    lines.append("=" * 65)
    lines.append("  COMPARAISON A* vs RRT — Resultats")
    lines.append("=" * 65)
    lines.append("")
    lines.append(f"{'Metrique':<40} {'A*':>10} {'RRT':>10}")
    lines.append("-" * 60)
    lines.append(f"{'  PATROUILLE':<40}")
    lines.append(f"  Succes{'':<34} "
                 f"{'OUI' if a['success'] else 'NON':>10} "
                 f"{'OUI' if rr['success'] else 'NON':>10}")
    lines.append(f"  Waypoints atteints{'':<28} "
                 f"{a['waypoints_reached']:>10} {rr['waypoints_reached']:>10}")
    lines.append(f"  Temps mission (s){'':<29} "
                 f"{a['mission_time']:>10.1f} {rr['mission_time']:>10.1f}")
    lines.append(f"  Longueur chemin (m){'':<28} "
                 f"{a['total_path_length']:>10.2f} {rr['total_path_length']:>10.2f}")
    lines.append(f"  Planif moyenne (ms/wp){'':<24} "
                 f"{a_avg:>10.1f} {r_avg:>10.1f}")
    lines.append(f"  Dist min obstacles (m){'':<24} "
                 f"{a['min_obstacle_dist']:>10.3f} {rr['min_obstacle_dist']:>10.3f}")
    lines.append(f"{'  REPLANIFICATION':<40}")
    lines.append(f"  Succes replan{'':<32} "
                 f"{'OUI' if ra['success'] else 'NON':>10} "
                 f"{'OUI' if rrr['success'] else 'NON':>10}")
    lines.append(f"  Temps replan (ms){'':<30} "
                 f"{ra['replan_time_ms']:>10.1f} {rrr['replan_time_ms']:>10.1f}")
    lines.append(f"  Dist au but final (m){'':<25} "
                 f"{ra['goal_dist']:>10.4f} {rrr['goal_dist']:>10.4f}")
    lines.append(f"  Temps total (s){'':<33} "
                 f"{ra['total_time']:>10.1f} {rrr['total_time']:>10.1f}")
    lines.append("=" * 65)

    content = "\n".join(lines)
    with open(filepath, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"  Resultats texte : {filepath}")


# ======================================================================
# Scenario existant (Malala - Role 1)
# ======================================================================

def scenario_avancer_puis_tourner():
    """Scénario simple : le robot avance puis tourne, comme au section 19."""
    robot = Robot()
    sim = Simulator(robot)

    # 3 secondes en ligne droite
    sim.run(duration=3.0, command_fn=lambda r, t: r.set_velocity(0.3, 0.0))
    # puis 2 secondes de rotation
    sim.run(duration=2.0, command_fn=lambda r, t: r.set_velocity(0.0, 0.5))

    path = robot.export_log(f"{config.RESULTS_DIR}/scenario_avancer_tourner.csv")
    print(f"Scénario terminé. État final : {robot.get_state()}")
    print(f"Log exporté : {path}")
    return robot


# ======================================================================
# Scenario Patrouille (Role 3 - Koja)
# ======================================================================

def scenario_patrouille(planner_name="astar", verbose=False,
                        plot_path=None, csv_path=None):
    """Patrouille : le robot visite 4 waypoints avec planification + Pure Pursuit.

    Args:
        planner_name: "astar" ou "rrt"
        verbose: afficher les details par waypoint
        plot_path: chemin PNG pour sauvegarder la trajectoire (None = pas de graphique)
        csv_path: chemin CSV pour sauvegarder le log du robot (None = pas de CSV)

    Returns:
        dict avec metriques (success, mission_time, path_length, etc.)
    """
    grid = create_test_grid(
        config.WORLD_WIDTH, config.WORLD_HEIGHT,
        config.GRID_RESOLUTION, PATROL_OBSTACLES,
    )

    planner = _make_planner(
        planner_name, grid, config.GRID_RESOLUTION, config.ROBOT_RADIUS,
    )
    controller = PurePursuitController(
        lookahead_distance=config.LOOKAHEAD_DISTANCE,
        v_cruise=config.V_CRUISE,
        goal_tolerance=config.GOAL_TOLERANCE,
    )

    robot = Robot()

    metrics = {
        "planner": planner_name,
        "success": False,
        "waypoints_total": len(PATROL_WAYPOINTS),
        "waypoints_reached": 0,
        "mission_time": 0.0,
        "total_path_length": 0.0,
        "total_plan_time_ms": 0.0,
        "plan_times_ms": [],
        "min_obstacle_dist": float("inf"),
    }

    t_start = robot.time

    for wp_idx, wp in enumerate(PATROL_WAYPOINTS):
        px, py, _ = robot.get_true_pose()
        path = planner.plan(start=(px, py), goal=wp)
        pt = planner.last_plan_time_ms
        metrics["plan_times_ms"].append(round(pt, 2))
        metrics["total_plan_time_ms"] += pt

        if not path:
            if verbose:
                print(f"  WP{wp_idx+1} {wp}: PAS DE CHEMIN")
            break

        plen = _path_length(path)
        metrics["total_path_length"] += plen
        path = _resample_path(path, max_segment=0.5)
        controller.reset()

        if verbose:
            print(f"  WP{wp_idx+1} {wp}: {len(path)} pts (resample), "
                  f"plan={pt:.1f}ms, long={plen:.2f}m")

        reached, steps, final_d = _follow_path(robot, controller, path, max_time=180.0)

        if reached:
            metrics["waypoints_reached"] += 1
            if verbose:
                print(f"    -> OK en {steps * config.DT:.1f}s "
                      f"(dist={final_d:.4f}m)")
        else:
            if verbose:
                print(f"    -> TIMEOUT (dist={final_d:.4f}m)")

        # Metrique : distance min aux obstacles pendant ce segment
        for state in robot.history[-steps:] if steps > 0 else []:
            d_obs = _min_dist_to_rects(
                state["x"], state["y"], PATROL_OBSTACLES, config.ROBOT_RADIUS
            )
            if d_obs < metrics["min_obstacle_dist"]:
                metrics["min_obstacle_dist"] = d_obs

    metrics["mission_time"] = round(robot.time - t_start, 1)
    metrics["success"] = (
        metrics["waypoints_reached"] == metrics["waypoints_total"]
    )

    # --- Export graphique ---
    if plot_path and robot.history:
        _plot_trajectory(
            robot_history=robot.history,
            obstacles=PATROL_OBSTACLES,
            waypoints=PATROL_WAYPOINTS,
            title=f"Patrouille — {planner_name.upper()} "
                  f"({'Succes' if metrics['success'] else 'Echec'}, "
                  f"{metrics['mission_time']:.1f}s)",
            save_path=plot_path,
        )

    # --- Export CSV ---
    if csv_path and robot.history:
        _ensure_dir(csv_path)
        robot.export_log(csv_path)
        if verbose:
            print(f"  Log CSV : {csv_path}")

    return metrics


# ======================================================================
# Scenario Replanification (Role 3 - Koja)
# ======================================================================

def scenario_replanification(planner_name="astar", verbose=False,
                             plot_path=None, csv_path=None):
    """Obstacle imprevu pendant le suivi : le robot replanifie.

    1. Planifie un chemin direct.
    2. Suit le chemin pendant quelques secondes.
    3. Un obstacle apparait sur le chemin.
    4. Le robot replanifie et continue.

    Args:
        planner_name: "astar" ou "rrt"
        verbose: afficher les details
        plot_path: chemin PNG pour la trajectoire (None = pas de graphique)
        csv_path: chemin CSV pour le log (None = pas de CSV)

    Returns:
        dict avec metriques de replanification
    """
    # Obstacles initiaux : un mur partiel en bas
    initial_obstacles = [
        {"type": "rect", "x": 5.0, "y": 0.0, "w": 0.3, "h": 5.0},
    ]

    # Obstacle imprevu : mur qui bloque le passage (gap en haut)
    unexpected_obstacle = {
        "type": "rect", "x": 10.0, "y": 1.0, "w": 0.4, "h": 12.0
    }

    start = (2.0, 7.5)
    goal = (18.0, 7.5)
    obstacle_time = 8.0  # l'obstacle apparait apres 8s

    # Grille et planificateur initiaux
    grid = create_test_grid(
        config.WORLD_WIDTH, config.WORLD_HEIGHT,
        config.GRID_RESOLUTION, initial_obstacles,
    )
    planner = _make_planner(
        planner_name, grid, config.GRID_RESOLUTION, config.ROBOT_RADIUS,
    )
    controller = PurePursuitController(
        lookahead_distance=config.LOOKAHEAD_DISTANCE,
        v_cruise=config.V_CRUISE,
        goal_tolerance=config.GOAL_TOLERANCE,
    )

    robot = Robot(initial_pose=(*start, 0.0))
    dt = config.DT

    # Planification initiale
    path = planner.plan(start=start, goal=goal)
    initial_plan_time = planner.last_plan_time_ms

    metrics = {
        "planner": planner_name,
        "success": False,
        "initial_plan_time_ms": round(initial_plan_time, 2),
        "replan_time_ms": 0.0,
        "replan_count": 0,
        "total_time": 0.0,
        "goal_dist": float("inf"),
        "obstacle_appeared": False,
    }

    if not path:
        if verbose:
            print(f"  PAS DE CHEMIN initial")
        return metrics

    if verbose:
        print(f"  Chemin initial: {len(path)} pts, "
              f"plan={initial_plan_time:.1f}ms, "
              f"long={_path_length(path):.2f}m")

    path = _resample_path(path, max_segment=0.5)
    controller.reset()

    # Phase 1 : suivre le chemin jusqu'a l'obstacle
    max_steps = int(obstacle_time / dt)
    for _ in range(max_steps):
        px, py, pth = robot.get_true_pose()
        if controller.goal_reached((px, py), path):
            metrics["success"] = True
            metrics["goal_dist"] = 0.0
            metrics["total_time"] = round(robot.time, 2)
            if verbose:
                print(f"  But atteint AVANT l'obstacle a t={robot.time:.1f}s")
            return metrics
        v, omega = controller.compute_command(pose=(px, py, pth), path=path)
        robot.set_velocity(v, omega)
        robot.step(dt)

    px, py, pth = robot.get_true_pose()

    # Phase 2 : l'obstacle apparait
    metrics["obstacle_appeared"] = True
    all_obstacles = initial_obstacles + [unexpected_obstacle]

    if verbose:
        print(f"  Obstacle imprevu a t={robot.time:.1f}s! "
              f"Robot a ({px:.2f}, {py:.2f})")

    # Replanification complete : grille + planificateur + chemin
    t0 = time.perf_counter()
    grid_updated = create_test_grid(
        config.WORLD_WIDTH, config.WORLD_HEIGHT,
        config.GRID_RESOLUTION, all_obstacles,
    )
    planner = _make_planner(
        planner_name, grid_updated, config.GRID_RESOLUTION, config.ROBOT_RADIUS,
    )
    new_path = planner.plan(start=(px, py), goal=goal)
    replan_elapsed = (time.perf_counter() - t0) * 1000

    metrics["replan_time_ms"] = round(replan_elapsed, 2)
    metrics["replan_count"] = 1

    if not new_path:
        if verbose:
            print(f"  PAS DE CHEMIN apres replanification!")
        metrics["goal_dist"] = round(
            math.sqrt((px - goal[0]) ** 2 + (py - goal[1]) ** 2), 4
        )
        metrics["total_time"] = round(robot.time, 2)
        return metrics

    if verbose:
        print(f"  Replan OK: {len(new_path)} pts, "
              f"long={_path_length(new_path):.2f}m, "
              f"temps={replan_elapsed:.1f}ms")

    # Phase 3 : suivre le nouveau chemin
    new_path = _resample_path(new_path, max_segment=0.5)
    controller.reset()
    reached, steps, final_d = _follow_path(robot, controller, new_path, max_time=180.0)

    if reached:
        metrics["success"] = True

    px, py, _ = robot.get_true_pose()
    metrics["goal_dist"] = round(
        math.sqrt((px - goal[0]) ** 2 + (py - goal[1]) ** 2), 4
    )
    metrics["total_time"] = round(robot.time, 2)

    if verbose:
        status = "OK" if metrics["success"] else "ECHEC"
        print(f"  Resultat: {status}, dist={metrics['goal_dist']:.4f}m, "
              f"temps={metrics['total_time']:.1f}s")

    # --- Export graphique ---
    if plot_path and robot.history:
        _plot_trajectory(
            robot_history=robot.history,
            obstacles=initial_obstacles,
            waypoints=[start, goal],
            title=f"Replanification — {planner_name.upper()} "
                  f"({'Succes' if metrics['success'] else 'Echec'}, "
                  f"replan={metrics['replan_time_ms']:.0f}ms)",
            save_path=plot_path,
            extra_obstacles=[unexpected_obstacle],
        )

    # --- Export CSV ---
    if csv_path and robot.history:
        _ensure_dir(csv_path)
        robot.export_log(csv_path)
        if verbose:
            print(f"  Log CSV : {csv_path}")

    return metrics


# ======================================================================
# Comparaison A* vs RRT (Role 3 - Koja)
# ======================================================================

def scenario_comparaison(verbose=True):
    """Compare A* et RRT sur patrouille + replanification.

    Affiche un tableau pour le rapport (section 4, Role 5 - Tino).
    Sauvegarde les graphiques et resultats dans results/features_planning/.
    Returns dict avec tous les resultats.
    """
    # Repertoires de sortie
    base_dir = os.path.join(config.RESULTS_DIR, "features_planning")
    images_dir = os.path.join(base_dir, "images")
    logs_dir = os.path.join(base_dir, "logs")
    for d in [images_dir, logs_dir]:
        os.makedirs(d, exist_ok=True)

    print()
    print("=" * 65)
    print("  COMPARAISON A* vs RRT - Patrouille")
    print("=" * 65)

    patrol = {}
    for name in ["astar", "rrt"]:
        print(f"\n--- {name.upper()} - PATROUILLE ---")
        r = scenario_patrouille(
            planner_name=name, verbose=verbose,
            plot_path=os.path.join(images_dir, f"patrol_{name}.png"),
            csv_path=os.path.join(logs_dir, f"patrol_{name}.csv"),
        )
        patrol[name] = r
        avg = r["total_plan_time_ms"] / max(1, len(r["plan_times_ms"]))
        print(f"  Succes: {'OUI' if r['success'] else 'NON'}")
        print(f"  Waypoints: {r['waypoints_reached']}/{r['waypoints_total']}")
        print(f"  Temps: {r['mission_time']:.1f}s")
        print(f"  Chemin: {r['total_path_length']:.2f}m")
        print(f"  Planif moy: {avg:.1f}ms/wp")
        print(f"  Dist min obs: {r['min_obstacle_dist']:.3f}m")

    print()
    print("=" * 65)
    print("  COMPARAISON A* vs RRT - Replanification")
    print("=" * 65)

    replan = {}
    for name in ["astar", "rrt"]:
        print(f"\n--- {name.upper()} - REPLANIFICATION ---")
        r = scenario_replanification(
            planner_name=name, verbose=verbose,
            plot_path=os.path.join(images_dir, f"replan_{name}.png"),
            csv_path=os.path.join(logs_dir, f"replan_{name}.csv"),
        )
        replan[name] = r
        print(f"  Succes: {'OUI' if r['success'] else 'NON'}")
        print(f"  Planif init: {r['initial_plan_time_ms']:.1f}ms")
        print(f"  Replan: {r['replan_time_ms']:.1f}ms")
        print(f"  Dist but: {r['goal_dist']:.4f}m")
        print(f"  Temps: {r['total_time']:.1f}s")

    # Tableau resume
    a = patrol["astar"]
    rr = patrol["rrt"]
    ra = replan["astar"]
    rrr = replan["rrt"]
    a_avg = a["total_plan_time_ms"] / max(1, len(a["plan_times_ms"]))
    r_avg = rr["total_plan_time_ms"] / max(1, len(rr["plan_times_ms"]))

    print()
    print("=" * 65)
    print("  TABLEAU RECAPITULATIF")
    print("=" * 65)
    print(f"{'Metrique':<40} {'A*':>10} {'RRT':>10}")
    print("-" * 60)
    print(f"{'  PATROUILLE':<40}")
    print(f"  Succes{'':<34} "
          f"{'OUI' if a['success'] else 'NON':>10} "
          f"{'OUI' if rr['success'] else 'NON':>10}")
    print(f"  Waypoints atteints{'':<28} "
          f"{a['waypoints_reached']:>10} {rr['waypoints_reached']:>10}")
    print(f"  Temps mission (s){'':<29} "
          f"{a['mission_time']:>10.1f} {rr['mission_time']:>10.1f}")
    print(f"  Longueur chemin (m){'':<28} "
          f"{a['total_path_length']:>10.2f} {rr['total_path_length']:>10.2f}")
    print(f"  Planif moyenne (ms/wp){'':<24} "
          f"{a_avg:>10.1f} {r_avg:>10.1f}")
    print(f"  Dist min obstacles (m){'':<24} "
          f"{a['min_obstacle_dist']:>10.3f} {rr['min_obstacle_dist']:>10.3f}")
    print(f"{'  REPLANIFICATION':<40}")
    print(f"  Succes replan{'':<32} "
          f"{'OUI' if ra['success'] else 'NON':>10} "
          f"{'OUI' if rrr['success'] else 'NON':>10}")
    print(f"  Temps replan (ms){'':<30} "
          f"{ra['replan_time_ms']:>10.1f} {rrr['replan_time_ms']:>10.1f}")
    print(f"  Dist au but final (m){'':<25} "
          f"{ra['goal_dist']:>10.4f} {rrr['goal_dist']:>10.4f}")
    print(f"  Temps total (s){'':<33} "
          f"{ra['total_time']:>10.1f} {rrr['total_time']:>10.1f}")
    print("=" * 65)

    # --- Sauvegarde du tableau en texte ---
    txt_path = os.path.join(base_dir, "comparaison.txt")
    _save_comparison_txt(patrol, replan, txt_path)

    print(f"\n  Resultats sauvegardes dans : {base_dir}/")
    print(f"    - Tableau     : {txt_path}")
    print(f"    - Graphiques  : {images_dir}/")
    print(f"    - Logs CSV    : {logs_dir}/")

    return {"patrol": patrol, "replanification": replan}


# ======================================================================
# Point d'entree
# ======================================================================

if __name__ == "__main__":
    print("Patrol-Bot - Experiments")
    print(f"Carte: {config.WORLD_WIDTH}x{config.WORLD_HEIGHT}m, "
          f"res={config.GRID_RESOLUTION}m")
    print(f"Waypoints: {PATROL_WAYPOINTS}")
    print()

    results = scenario_comparaison(verbose=True)
    print("\nFin des experiences.")

