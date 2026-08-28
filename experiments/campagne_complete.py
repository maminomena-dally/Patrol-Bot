import csv
import math
import os
import random
import statistics as stats

import config
from robot.robot import Robot
from sensors.odometry import Odometry
from sensors.landmarks import LandmarkDetector
from sensors.lidar import LidarSensor
from localization.localization import Localizer
from control.pure_pursuit import PurePursuitController
from safety.safety_manager import SafetyManager, EtatSurete
from planning.astar import create_test_grid
from experiments.run_experiments import (
    _make_planner, _resample_path, _path_length, _ensure_dir,
    _min_obstacle_dist_sur_trajet,
)
from experiments.campagne_essais import (
    generer_essais_nominaux,
    cas_limite_chemin_le_plus_court,
    cas_limite_couloir_bloque,
)
from experiments.campagne_localisation import LANDMARKS, INITIAL_OBSTACLES, START, GOAL


def cas_limite_perte_balise():
    """
    3e cas limite exigé par le sujet : perte temporaire d'une balise
    pendant la replanification.

    Marqué NON IMPLEMENTABLE dans campagne_essais.py au moment où le
    rôle Perception/Localisation était vacant — ce n'est plus le cas
    (Odometry, LandmarkDetector, Localizer sont fonctionnels et déjà
    testés isolément dans campagne_localisation.py). Même géométrie que
    cas_limite_chemin_le_plus_court (détour possible) : on veut voir la
    replanification réussir PENDANT que la balise est coupée, pas juste
    constater un blocage total (déjà couvert par cas_limite_couloir_bloque).
    """
    return {
        "id": "cas_limite_perte_balise",
        "obstacle": {"type": "rect", "x": 10.0, "y": 1.0, "w": 0.4, "h": 12.0},
        "obstacle_time": 2.0,
        "perte_balise": True,
        "duree_perte": 3.0,
    }


CAS_LIMITES = [
    cas_limite_chemin_le_plus_court(),
    cas_limite_couloir_bloque(),
    cas_limite_perte_balise(),
]


def executer_essai(essai, planner_name, verbose=False):
    """
    Rejoue un essai (nominal ou cas limite) avec la boucle complète :
    Odometry -> Localizer(EKF) -> LandmarkDetector -> A*/RRT -> Pure
    Pursuit -> SafetyManager, et calcule les 5 métriques exigées par le
    sujet en une seule passe.
    """
    random.seed(42 if planner_name == "astar" else 43)
    dt = config.DT
    perte_balise = essai.get("perte_balise", False)
    duree_perte = essai.get("duree_perte", 3.0)
    unexpected_obstacle = essai["obstacle"]
    obstacle_time = essai["obstacle_time"]

    robot = Robot(initial_pose=(START[0], START[1], 0.0))
    odometry = Odometry(robot)
    landmark_detector = LandmarkDetector(robot, LANDMARKS)
    localizer = Localizer(initial_pose=(START[0], START[1], 0.0))
    controller = PurePursuitController(
        lookahead_distance=config.LOOKAHEAD_DISTANCE,
        v_cruise=config.V_CRUISE,
        goal_tolerance=config.GOAL_TOLERANCE,
    )
    sm = SafetyManager(tentatives_max_replanification=3)
    lidar = LidarSensor(robot, obstacles=INITIAL_OBSTACLES)

    grid = create_test_grid(config.WORLD_WIDTH, config.WORLD_HEIGHT,
                             config.GRID_RESOLUTION, INITIAL_OBSTACLES)
    planner = _make_planner(planner_name, grid, config.GRID_RESOLUTION, config.ROBOT_RADIUS)
    initial_path = planner.plan(start=START, goal=GOAL)

    if not initial_path:
        # N'arrive pas avec cette geometrie fixe, mais gere proprement au
        # cas ou : goal_reached([]) renverrait True a tort sinon.
        return {
            "id_essai": essai["id"], "planner": planner_name, "success": False,
            "mission_time_s": 0.0, "path_length_m": 0.0,
            "min_obstacle_dist_m": None, "erreur_localisation_moy_m": None,
            "erreur_localisation_max_m": None, "incertitude_finale_m": None,
            "perte_balise": perte_balise, "balise_effectivement_coupee": False,
            "aucun_chemin_trouve": True, "safety_reaction": "non_teste",
            "safety_ok": False, "etat_surete_final": EtatSurete.NOMINAL.name,
        }

    path_length_total = _path_length(initial_path)
    path = _resample_path(initial_path, max_segment=0.5)
    controller.reset()

    erreurs = []
    obstacle_apparu = False
    balise_coupee = False
    path_found = True
    success = False
    max_steps = int(90.0 / dt)

    for _ in range(max_steps):
        px, py, pth = robot.get_true_pose()

        # --- Perception + Localisation EKF ---
        d_l, d_r = odometry.read(dt)
        localizer.predict(d_l, d_r)

        balise_indisponible = (
            perte_balise and obstacle_apparu
            and (robot.time - obstacle_time) < duree_perte
        )
        if balise_indisponible:
            balise_coupee = True
            detections = []
        else:
            detections = landmark_detector.detect()
        localizer.correct(detections)

        erreurs.append(math.hypot(px - localizer.estimated_pose.x, py - localizer.estimated_pose.y))

        # --- Obstacle imprevu + replanification ---
        if robot.time >= obstacle_time and not obstacle_apparu:
            obstacle_apparu = True
            all_obstacles = INITIAL_OBSTACLES + [unexpected_obstacle]
            lidar.update_obstacles(all_obstacles)
            grid_updated = create_test_grid(config.WORLD_WIDTH, config.WORLD_HEIGHT,
                                             config.GRID_RESOLUTION, all_obstacles)
            planner_replan = _make_planner(planner_name, grid_updated,
                                            config.GRID_RESOLUTION, config.ROBOT_RADIUS)
            new_path = planner_replan.plan(start=(px, py), goal=GOAL)
            path_found = bool(new_path)
            if path_found:
                path_length_total += _path_length(new_path)
                path = _resample_path(new_path, max_segment=0.5)
                controller.reset()
            if verbose:
                print(f"  t={robot.time:.1f}s obstacle imprevu -> chemin_trouve={path_found}")

        # --- Surete, avec la VRAIE incertitude EKF ---
        etat = sm.check(robot, localization_uncertainty=localizer.uncertainty,
                         obstacle_distance=lidar.min_distance(), path_found=path_found)

        if etat == EtatSurete.ARRET_SUR:
            if verbose:
                print(f"  t={robot.time:.1f}s -> ARRET_SUR (incertitude={localizer.uncertainty:.3f}m)")
            break

        if controller.goal_reached((px, py), path):
            success = True
            break

        v, omega = controller.compute_command(pose=(px, py, pth), path=path)
        robot.set_velocity(v, omega)
        robot.step(dt)

    min_obs_dist = _min_obstacle_dist_sur_trajet(robot, INITIAL_OBSTACLES, unexpected_obstacle, obstacle_time)
    aucun_chemin = not path_found

    if aucun_chemin:
        safety_reaction = sm.etat.name
        safety_ok = (sm.etat == EtatSurete.ARRET_SUR)
    else:
        safety_reaction = "non_declenche"
        safety_ok = True

    return {
        "id_essai": essai["id"],
        "planner": planner_name,
        "success": success,
        "mission_time_s": round(robot.time, 2),
        "path_length_m": round(path_length_total, 2),
        "min_obstacle_dist_m": round(min_obs_dist, 4),
        "erreur_localisation_moy_m": round(stats.mean(erreurs), 4) if erreurs else None,
        "erreur_localisation_max_m": round(max(erreurs), 4) if erreurs else None,
        "incertitude_finale_m": round(localizer.uncertainty, 4),
        "perte_balise": perte_balise,
        "balise_effectivement_coupee": balise_coupee,
        "aucun_chemin_trouve": aucun_chemin,
        "safety_reaction": safety_reaction,
        "safety_ok": safety_ok,
        "etat_surete_final": sm.etat.name,
    }


def lancer_campagne(planner_name, n_nominaux=10, verbose=False):
    essais = generer_essais_nominaux(n_nominaux) + CAS_LIMITES
    resultats = []
    for essai in essais:
        if verbose:
            print(f"  [{planner_name}] {essai['id']} ...")
        resultats.append(executer_essai(essai, planner_name, verbose=False))
    return resultats


def agreger(resultats):
    """Les 5 metriques exigees par le sujet, agregees sur un algo."""
    n = len(resultats)
    succes = [r["success"] for r in resultats]
    temps = [r["mission_time_s"] for r in resultats]
    longueurs = [r["path_length_m"] for r in resultats if r["path_length_m"] > 0]
    dist_obs = [r["min_obstacle_dist_m"] for r in resultats if r["min_obstacle_dist_m"] is not None]
    err_loc = [r["erreur_localisation_moy_m"] for r in resultats if r["erreur_localisation_moy_m"] is not None]

    return {
        "nb_essais": n,
        "taux_succes": round(sum(succes) / n, 3),
        "temps_mission_moy_s": round(stats.mean(temps), 2) if temps else None,
        "longueur_trajet_moy_m": round(stats.mean(longueurs), 2) if longueurs else None,
        "distance_min_obstacle_m": round(min(dist_obs), 4) if dist_obs else None,
        "erreur_localisation_moy_m": round(stats.mean(err_loc), 4) if err_loc else None,
        "erreur_localisation_max_m": round(max(
            (r["erreur_localisation_max_m"] for r in resultats if r["erreur_localisation_max_m"] is not None),
            default=0.0), 4),
        "nb_arrets_surs_attendus": sum(r["aucun_chemin_trouve"] for r in resultats),
        "nb_arrets_surs_confirmes": sum(r["aucun_chemin_trouve"] and r["safety_ok"] for r in resultats),
    }


def sauvegarder(resultats_par_algo, resume_par_algo, base_dir=None):
    if base_dir is None:
        base_dir = os.path.join(config.RESULTS_DIR, "features_experimentation")
    os.makedirs(base_dir, exist_ok=True)

    csv_path = os.path.join(base_dir, "campagne_complete.csv")
    tous = [r for essais in resultats_par_algo.values() for r in essais]
    if tous:
        champs = list(tous[0].keys())
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=champs)
            writer.writeheader()
            for r in tous:
                writer.writerow(r)

    txt_path = os.path.join(base_dir, "resume_campagne_complete.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("CAMPAGNE COMPLETE -- conforme au sujet (10 essais + 3 cas limites)\n")
        f.write("Metriques : taux de succes, temps, longueur de trajet, "
                "distance min. obstacles, erreur de localisation\n")
        f.write("=" * 70 + "\n\n")
        for algo, resume in resume_par_algo.items():
            f.write(f"--- {algo.upper()} ---\n")
            for k, v in resume.items():
                f.write(f"  {k:<28}: {v}\n")
            f.write("\n")
        f.write("Cas limites couverts : chemin_le_plus_court, couloir_bloque, "
                "perte_balise\n")
        f.write(f"Total essais par algo : {10} nominaux + {len(CAS_LIMITES)} cas limites\n")

    return csv_path, txt_path


def main():
    print("Campagne complete -- 10 essais nominaux + 3 cas limites, x2 algos")
    print("Metriques : succes, temps, longueur, distance min. obstacles, localisation\n")

    resultats_par_algo = {}
    resume_par_algo = {}

    for algo in ["astar", "rrt"]:
        print(f"--- Campagne {algo.upper()} ---")
        resultats = lancer_campagne(algo, n_nominaux=10, verbose=True)
        resultats_par_algo[algo] = resultats
        resume = agreger(resultats)
        resume_par_algo[algo] = resume
        print(f"  Taux de succes          : {resume['taux_succes']*100:.0f}%")
        print(f"  Temps mission moy (s)    : {resume['temps_mission_moy_s']}")
        print(f"  Longueur trajet moy (m)  : {resume['longueur_trajet_moy_m']}")
        print(f"  Distance min obstacle (m): {resume['distance_min_obstacle_m']}")
        print(f"  Erreur localisation (m)  : moy={resume['erreur_localisation_moy_m']} "
              f"max={resume['erreur_localisation_max_m']}")
        print(f"  Arrets surs attendus/OK  : "
              f"{resume['nb_arrets_surs_attendus']}/{resume['nb_arrets_surs_confirmes']}")
        print()

    csv_path, txt_path = sauvegarder(resultats_par_algo, resume_par_algo)
    print(f"Resultats sauvegardes :\n  - {csv_path}\n  - {txt_path}")


if __name__ == "__main__":
    main()
