"""
experiments/demo_safety.py — Demonstration visuelle du SafetyManager (Role 5 - Tino).

Rejoue le cas limite "couloir totalement bloque" pas a pas, en appelant
reellement `SafetyManager.check()` a CHAQUE pas de simulation (contrairement
a experiments/campagne_essais.py qui ne le simule qu'au moment du bilan).
Produit un graphique montrant la trajectoire du robot, coloree selon
l'etat de surete (NOMINAL / ALERTE / ARRET_SUR), avec un marqueur explicite
a l'endroit ou l'arret sur se declenche reellement.

C'est la "preuve visuelle" que le module de surete developpe pour ce role
fonctionne correctement, en attendant que sim.on_safety soit branche dans
la boucle d'integration finale (Phase 5, avec dally).

Lancer avec :
    python -m experiments.demo_safety
"""

import os
import time

import config
from robot.robot import Robot
from planning.astar import create_test_grid
from control.pure_pursuit import PurePursuitController
from experiments.run_experiments import (
    _make_planner,
    _resample_path,
    _ensure_dir,
)
from safety.safety_manager import SafetyManager, EtatSurete
from sensors.lidar import LidarSensor
from sensors.odometry import Odometry
from sensors.landmarks import LandmarkDetector
from localization.localization import Localizer
from experiments.campagne_localisation import LANDMARKS


# Reprend exactement la geometrie du cas limite "couloir bloque"
# (voir experiments/campagne_essais.py -> cas_limite_couloir_bloque)
INITIAL_OBSTACLES = [
    {"type": "rect", "x": 5.0, "y": 0.0, "w": 0.3, "h": 5.0},
]
UNEXPECTED_OBSTACLE = {
    "type": "rect", "x": 10.0, "y": 0.0, "w": 0.4, "h": config.WORLD_HEIGHT,
}
START = (2.0, 7.5)
GOAL = (18.0, 7.5)
OBSTACLE_TIME = 8.0


def rejouer_avec_surete(planner_name="astar", verbose=True):
    """
    Rejoue le scenario "couloir bloque" en appelant SafetyManager.check()
    a chaque pas. Retourne (robot, historique_surete) ou historique_surete
    est une liste de (t, x, y, etat) pour la visualisation.
    """
    dt = config.DT
    robot = Robot(initial_pose=(START[0], START[1], 0.0))
    controller = PurePursuitController()
    sm = SafetyManager(tentatives_max_replanification=3)
    lidar = LidarSensor(robot, obstacles=INITIAL_OBSTACLES)  # LIAISON : vrai capteur
    odometry = Odometry(robot)
    landmarks_detector = LandmarkDetector(robot, LANDMARKS)  # memes balises que campagne_localisation.py
    localizer = Localizer(initial_pose=(START[0], START[1], 0.0))  # LIAISON : vraie localisation

    grid = create_test_grid(config.WORLD_WIDTH, config.WORLD_HEIGHT,
                             config.GRID_RESOLUTION, INITIAL_OBSTACLES)
    planner = _make_planner(planner_name, grid, config.GRID_RESOLUTION, config.ROBOT_RADIUS)
    path = planner.plan(start=START, goal=GOAL)
    path = _resample_path(path, max_segment=0.5)
    controller.reset()

    historique_surete = []
    obstacle_apparu = False
    chemin_bloque = False

    max_steps = int(30.0 / dt)  # 30s de marge, largement suffisant pour ce cas
    for _ in range(max_steps):
        px, py, pth = robot.get_true_pose()

        # -- Localisation reelle : predict (odometrie) + correct (balises) --
        d_left, d_right = odometry.read(dt)
        localizer.predict(d_left, d_right)
        detections = landmarks_detector.detect()
        localizer.correct(detections)

        # -- L'obstacle imprevu apparait (une seule fois) --
        if robot.time >= OBSTACLE_TIME and not obstacle_apparu:
            obstacle_apparu = True
            all_obstacles = INITIAL_OBSTACLES + [UNEXPECTED_OBSTACLE]
            lidar.update_obstacles(all_obstacles)  # LIAISON : le lidar voit l'obstacle des son apparition
            grid_updated = create_test_grid(config.WORLD_WIDTH, config.WORLD_HEIGHT,
                                             config.GRID_RESOLUTION, all_obstacles)
            planner_replan = _make_planner(planner_name, grid_updated,
                                            config.GRID_RESOLUTION, config.ROBOT_RADIUS)
            new_path = planner_replan.plan(start=(px, py), goal=GOAL)
            path_found = bool(new_path)
            if path_found:
                path = _resample_path(new_path, max_segment=0.5)
                controller.reset()
            else:
                chemin_bloque = True  # reste bloque tant que rien ne change
            if verbose:
                print(f"  t={robot.time:.1f}s obstacle imprevu -> "
                      f"chemin_trouve={path_found}")
        elif chemin_bloque:
            # Le blocage persiste : on continue de tenter (sans succes),
            # comme le ferait reellement le planificateur en boucle.
            path_found = False
        else:
            path_found = True  # rien a signaler avant l'obstacle

        # -- SafetyManager verifie la situation a CE pas --
        etat = sm.check(robot, localization_uncertainty=localizer.uncertainty,
                         obstacle_distance=lidar.min_distance(), path_found=path_found)
        historique_surete.append((robot.time, px, py, etat.name))

        if etat == EtatSurete.ARRET_SUR:
            if verbose:
                print(f"  t={robot.time:.1f}s -> ARRET_SUR declenche par le SafetyManager "
                      f"(robot.stopped={robot.stopped})")
            robot.step(dt)  # un dernier pas pour figer la position d'arret dans l'historique
            break

        if chemin_bloque:
            # En attente de resolution : le robot ne doit pas continuer a
            # avancer vers un obstacle qui bloque tout le passage.
            robot.set_velocity(0.0, 0.0)
            robot.step(dt)
            continue

        if not path:
            break

        if controller.goal_reached((px, py), path):
            break

        v, omega = controller.compute_command(pose=(px, py, pth), path=path)
        robot.set_velocity(v, omega)
        robot.step(dt)

    return robot, historique_surete


def tracer_demo(robot, historique_surete, save_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print(f"  [AVERTISSEMENT] matplotlib non disponible, graphique non genere")
        return

    _ensure_dir(save_path)
    fig, ax = plt.subplots(figsize=(14, 10.5))
    ax.set_xlim(-0.5, config.WORLD_WIDTH + 0.5)
    ax.set_ylim(-0.5, config.WORLD_HEIGHT + 0.5)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("Demo SafetyManager (Role 5) — cas limite : couloir totalement bloque",
                  fontsize=13)

    for obs in INITIAL_OBSTACLES:
        rect = patches.Rectangle((obs["x"], obs["y"]), obs["w"], obs["h"],
                                  linewidth=1.5, edgecolor="#333", facecolor="#999")
        ax.add_patch(rect)

    rect = patches.Rectangle(
        (UNEXPECTED_OBSTACLE["x"], UNEXPECTED_OBSTACLE["y"]),
        UNEXPECTED_OBSTACLE["w"], UNEXPECTED_OBSTACLE["h"],
        linewidth=2, edgecolor="red", facecolor="#ff9999",
        linestyle="--", label="Obstacle imprevu (bloque tout le couloir)")
    ax.add_patch(rect)

    ax.plot(*START, "go", markersize=14, label="Depart", zorder=6)
    ax.plot(*GOAL, "b*", markersize=18, label="Objectif (jamais atteint)", zorder=6)

    couleurs = {"NOMINAL": "#2A7A2A", "ALERTE": "#E8A33D", "ARRET_SUR": "#C44E52"}
    for statut in ["NOMINAL", "ALERTE", "ARRET_SUR"]:
        pts = [(x, y) for (t, x, y, e) in historique_surete if e == statut]
        if pts:
            xs, ys = zip(*pts)
            ax.plot(xs, ys, "o", color=couleurs[statut], markersize=4,
                    label=f"Etat surete : {statut}", alpha=0.85, zorder=4)

    # Marqueur explicite du point d'arret sur
    arrets = [(x, y, t) for (t, x, y, e) in historique_surete if e == "ARRET_SUR"]
    if arrets:
        x, y, t = arrets[0]
        ax.plot(x, y, "X", color="black", markersize=20, zorder=7,
                label=f"ARRET_SUR declenche (t={t:.1f}s)")
        ax.annotate(f"Arret sur\nt={t:.1f}s", (x, y),
                    textcoords="offset points", xytext=(15, 15),
                    fontsize=11, fontweight="bold", color="black")

    ax.legend(loc="upper left", fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Graphique : {save_path}")


def main():
    print("Demo SafetyManager -- cas limite 'couloir totalement bloque'\n")
    for planner_name in ["astar", "rrt"]:
        print(f"--- {planner_name.upper()} ---")
        robot, historique = rejouer_avec_surete(planner_name, verbose=True)
        save_path = os.path.join(
            config.RESULTS_DIR, "features_experimentation", "images",
            f"safety_arret_sur_{planner_name}.png",
        )
        tracer_demo(robot, historique, save_path)
        print()


if __name__ == "__main__":
    main()
