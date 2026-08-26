"""
experiments/campagne_localisation.py — Erreur de localisation + cas limite
"perte de balise" (Role 5 - Tino).

Debloque deux points laisses en attente dans ROLE5_WORKFLOW.md / TINO_WORKFLOW.md :

  1. La metrique "erreur de localisation" du cahier des charges (section 5) :
     jusqu'ici non calculable, les scenarios de Koja (experiments/run_experiments.py)
     n'utilisant que la pose REELLE du robot (get_true_pose()), jamais une pose
     estimee comparable. Ici, Odometry + LandmarkDetector + Localizer (deja
     implementes, cf. localization/localization.py) sont branches reellement
     dans la boucle, pas simules avec une valeur fixe comme dans
     experiments/campagne_essais.py et experiments/demo_safety.py.

  2. Le cas limite 3 du cahier des charges (section 5) : "Perte temporaire
     d'une balise pendant la phase de replanification" -- jusqu'ici marque
     NON IMPLEMENTABLE (cf. experiments/campagne_essais.py::cas_limite_perte_balise)
     car le module de localisation n'etait pas encore verifie disponible.
     Il l'est : ce cas limite est maintenant teste.

N'utilise PAS sensors/lidar.py (toujours un stub vide) ni security/ (idem) :
uniquement des modules deja implementes (Odometry, LandmarkDetector, Localizer,
planning/, control/), donc ne depend d'aucune tache encore bloquee du groupe.

Lancer avec :
    python -m experiments.campagne_localisation
"""

import csv
import math
import os
import statistics as stats

import config
from robot.robot import Robot
from sensors.odometry import Odometry
from sensors.landmarks import LandmarkDetector
from localization.localization import Localizer
from control.pure_pursuit import PurePursuitController
from safety.safety_manager import SafetyManager, EtatSurete
from experiments.run_experiments import (
    PATROL_WAYPOINTS,
    _make_planner,
    _resample_path,
    _ensure_dir,
)
from planning.astar import create_test_grid


# Balises dediees a CE scenario de test (corridor y=7.5, cf. START/GOAL
# ci-dessous). Les 4 balises "points de controle" de Koja (PATROL_WAYPOINTS)
# sont a >4.5m de ce corridor : hors de LANDMARK_DETECTION_RADIUS (2.0m),
# donc jamais detectees ici -- inutilisables pour CE test precis. On place
# ici des balises le long du corridor pour permettre une correction reelle
# de la position estimee pendant le trajet (rayon de detection = 2.0m,
# balises espacees de 4m => couverture continue le long du corridor).
LANDMARKS = [
    {"id": 0, "x": 2.0, "y": 7.5},
    {"id": 1, "x": 6.0, "y": 7.5},
    {"id": 2, "x": 14.0, "y": 7.5},
    {"id": 3, "x": 18.0, "y": 7.5},
    # Pas de balise en x=10 (zone de l'obstacle imprevu) : c'est
    # deliberement une zone de couverture plus faible, cf. section "cas
    # limite perte de balise" plus bas -- le detour du robot passe hors
    # champ de x=10 pendant la replanification.
]

# Meme geometrie que le cas limite "chemin le plus court" (obstacle avec
# detour possible, contrairement au "couloir bloque" qui empeche tout
# chemin -- ici on veut justement voir la replanification reussir pendant
# que la balise est perdue).
INITIAL_OBSTACLES = [
    {"type": "rect", "x": 5.0, "y": 0.0, "w": 0.3, "h": 5.0},
]
UNEXPECTED_OBSTACLE = {"type": "rect", "x": 10.0, "y": 1.0, "w": 0.4, "h": 12.0}
START = (2.0, 7.5)
GOAL = (18.0, 7.5)
OBSTACLE_TIME = 8.0


def executer_essai(planner_name="astar", perte_balise=False, duree_perte=3.0, verbose=False):
    """
    Rejoue un essai de replanification avec localisation REELLE branchee
    (odometrie + balises + fusion), et SafetyManager alimente par la vraie
    incertitude (pas une valeur simulee fixe).

    Args:
        perte_balise: si True, les balises deviennent indetectables pendant
            `duree_perte` secondes a partir de l'apparition de l'obstacle
            imprevu -- reproduit le cas limite 3 du cahier des charges.
    """
    dt = config.DT
    robot = Robot(initial_pose=(START[0], START[1], 0.0))
    odometry = Odometry(robot)
    landmarks_detector = LandmarkDetector(robot, LANDMARKS)
    localizer = Localizer(initial_pose=(START[0], START[1], 0.0))
    controller = PurePursuitController()
    sm = SafetyManager(tentatives_max_replanification=3)

    grid = create_test_grid(config.WORLD_WIDTH, config.WORLD_HEIGHT,
                             config.GRID_RESOLUTION, INITIAL_OBSTACLES)
    planner = _make_planner(planner_name, grid, config.GRID_RESOLUTION, config.ROBOT_RADIUS)
    path = planner.plan(start=START, goal=GOAL)
    path = _resample_path(path, max_segment=0.5)

    erreurs = []
    obstacle_apparu = False
    balise_coupee = False
    max_steps = int(60.0 / dt)

    for _ in range(max_steps):
        px, py, pth = robot.get_true_pose()

        # -- Localisation reelle : predict (odometrie) + correct (balises) --
        d_left, d_right = odometry.read(dt)
        localizer.predict(d_left, d_right)

        balise_indisponible = (
            perte_balise and obstacle_apparu
            and (robot.time - OBSTACLE_TIME) < duree_perte
        )
        if balise_indisponible:
            balise_coupee = True
            detections = []  # balise "coupee" : aucune correction possible
        else:
            detections = landmarks_detector.detect()
        localizer.correct(detections)

        erreur = math.hypot(px - localizer.estimated_pose.x, py - localizer.estimated_pose.y)
        erreurs.append(erreur)

        # -- Obstacle imprevu + replanification (comme campagne_essais.py) --
        path_found = True
        if robot.time >= OBSTACLE_TIME and not obstacle_apparu:
            obstacle_apparu = True
            all_obstacles = INITIAL_OBSTACLES + [UNEXPECTED_OBSTACLE]
            grid_updated = create_test_grid(config.WORLD_WIDTH, config.WORLD_HEIGHT,
                                             config.GRID_RESOLUTION, all_obstacles)
            planner_replan = _make_planner(planner_name, grid_updated,
                                            config.GRID_RESOLUTION, config.ROBOT_RADIUS)
            new_path = planner_replan.plan(start=(px, py), goal=GOAL)
            path_found = bool(new_path)
            if path_found:
                path = _resample_path(new_path, max_segment=0.5)
                controller.reset()
            if verbose:
                print(f"  t={robot.time:.1f}s obstacle imprevu -> chemin_trouve={path_found}")

        # -- SafetyManager avec la VRAIE incertitude de localisation --
        etat = sm.check(robot, localization_uncertainty=localizer.uncertainty,
                         obstacle_distance=2.0, path_found=path_found)

        if etat == EtatSurete.ARRET_SUR:
            if verbose:
                print(f"  t={robot.time:.1f}s -> ARRET_SUR "
                      f"(incertitude={localizer.uncertainty:.3f}m)")
            break

        if not path or controller.goal_reached((px, py), path):
            break

        v, omega = controller.compute_command(pose=(px, py, pth), path=path)
        robot.set_velocity(v, omega)
        robot.step(dt)

    return {
        "planner": planner_name,
        "perte_balise": perte_balise,
        "duree_perte": duree_perte if perte_balise else 0.0,
        "balise_effectivement_coupee": balise_coupee,
        "erreur_localisation_moy_m": round(stats.mean(erreurs), 4) if erreurs else None,
        "erreur_localisation_max_m": round(max(erreurs), 4) if erreurs else None,
        "incertitude_finale_m": round(localizer.uncertainty, 4),
        "etat_surete_final": sm.etat.name,
        "arret_sur_declenche": sm.etat == EtatSurete.ARRET_SUR,
        "temps_total_s": round(robot.time, 2),
        "journal_surete": [(ev.t, ev.transition, ev.raison) for ev in sm.journal],
    }


def lancer_campagne_localisation(verbose=True):
    resultats = []
    for planner_name in ["astar", "rrt"]:
        for perte_balise in [False, True]:
            if verbose:
                label = "avec perte de balise" if perte_balise else "sans perte de balise"
                print(f"--- {planner_name.upper()} — {label} ---")
            r = executer_essai(planner_name, perte_balise=perte_balise, verbose=verbose)
            resultats.append(r)
            if verbose:
                print(f"  erreur_moy={r['erreur_localisation_moy_m']}m "
                      f"erreur_max={r['erreur_localisation_max_m']}m "
                      f"etat_final={r['etat_surete_final']}")
                print()
    return resultats


def sauvegarder(resultats, base_dir=None):
    if base_dir is None:
        base_dir = os.path.join(config.RESULTS_DIR, "features_experimentation")
    os.makedirs(base_dir, exist_ok=True)

    csv_path = os.path.join(base_dir, "campagne_localisation.csv")
    champs = [k for k in resultats[0].keys() if k != "journal_surete"]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=champs)
        writer.writeheader()
        for r in resultats:
            writer.writerow({k: r[k] for k in champs})

    txt_path = os.path.join(base_dir, "resume_localisation.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("CAMPAGNE LOCALISATION -- Role 5 (Tino)\n")
        f.write("Erreur de localisation + cas limite 'perte de balise'\n")
        f.write("=" * 60 + "\n\n")
        for r in resultats:
            label = "avec" if r["perte_balise"] else "sans"
            f.write(f"--- {r['planner'].upper()} ({label} perte de balise) ---\n")
            for k, v in r.items():
                if k == "journal_surete":
                    continue
                f.write(f"  {k:<32}: {v}\n")
            if r["journal_surete"]:
                f.write("  journal_surete:\n")
                for t, transition, raison in r["journal_surete"]:
                    f.write(f"    t={t:.2f}s  {transition}  ({raison})\n")
            f.write("\n")

    return csv_path, txt_path


def main():
    print("Campagne localisation -- erreur de localisation + perte de balise\n")
    resultats = lancer_campagne_localisation(verbose=True)
    csv_path, txt_path = sauvegarder(resultats)
    print(f"Resultats sauvegardes :\n  - {csv_path}\n  - {txt_path}")


if __name__ == "__main__":
    main()
