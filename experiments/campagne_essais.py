"""
experiments/campagne_essais.py — Campagne d'essais statistique (Role 5 — Tino).

Etend experiments/run_experiments.py (Koja) sans le modifier en profondeur :
seuls des parametres optionnels (position/instant de l'obstacle imprevu)
ont ete ajoutes a scenario_replanification(), avec des valeurs par defaut
identiques a l'original (aucune regression sur son code).

Implemente le protocole du cahier des charges (section 5) :
    - au moins 10 essais nominaux
    - au moins 3 cas limites
    - repete pour A* et RRT
    - metriques : taux de succes, temps de mission, longueur du trajet,
      distance min aux obstacles, temps de replanification

Lancer avec :
    python -m experiments.campagne_essais
"""

import csv
import os
import statistics as stats

import config
from experiments.run_experiments import (
    scenario_patrouille,
    scenario_replanification,
)
from safety.safety_manager import SafetyManager, EtatSurete


# ======================================================================
# 1. Essais nominaux (obstacle imprevu, position/instant variables)
#
#    L'obstacle garde un "gap" (h=12 sur 15m de hauteur de carte) pour
#    qu'un detour reste toujours possible -> essai nominal, pas cas limite.
# ======================================================================

def generer_essais_nominaux(n=10):
    """
    Genere n variantes d'obstacle imprevu (position x, instant d'apparition)
    couvrant le trajet et le temps de patrouille, pour un essai reproductible
    (pas de tirage aleatoire : positions espacees deterministes).
    """
    positions_x = [7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 8.5, 10.5, 11.5]
    instants = [3.0, 5.0, 8.0, 10.0, 12.0, 4.0, 6.0, 9.0, 11.0, 7.0]

    essais = []
    for i in range(n):
        x = positions_x[i % len(positions_x)]
        t = instants[i % len(instants)]
        essais.append({
            "id": f"nominal_{i+1:02d}",
            "obstacle": {"type": "rect", "x": x, "y": 1.0, "w": 0.4, "h": 12.0},
            "obstacle_time": t,
        })
    return essais


# ======================================================================
# 2. Cas limites (section 5 du cahier des charges)
# ======================================================================

def cas_limite_chemin_le_plus_court():
    """Obstacle directement sur le trajet le plus direct entre deux points."""
    return {
        "id": "cas_limite_chemin_court",
        "obstacle": {"type": "rect", "x": 10.0, "y": 1.0, "w": 0.4, "h": 12.0},
        "obstacle_time": 2.0,  # tres tot : le robot vient a peine de partir
    }


def cas_limite_couloir_bloque():
    """Obstacle qui bloque TOTALEMENT le couloir (pas de detour possible)."""
    return {
        "id": "cas_limite_couloir_bloque",
        # h = WORLD_HEIGHT : plus aucun gap en haut ni en bas
        "obstacle": {"type": "rect", "x": 10.0, "y": 0.0, "w": 0.4,
                     "h": config.WORLD_HEIGHT},
        "obstacle_time": 8.0,
    }


def cas_limite_perte_balise():
    """
    Perte temporaire d'une balise pendant la replanification.

    NON IMPLEMENTABLE pour l'instant : la simulation de perte de balise
    depend du module de localisation (Role 2 - Kojy), retire de l'equipe.
    A reactiver des qu'un module de localisation avec balises existe.
    """
    return None


CAS_LIMITES = [
    cas_limite_chemin_le_plus_court(),
    cas_limite_couloir_bloque(),
]


# ======================================================================
# 3. Execution d'un essai + verification de la reaction de surete
# ======================================================================

def executer_essai(essai, planner_name, verbose=False):
    """
    Lance un essai de replanification et verifie, en plus des metriques de
    Koja, que le SafetyManager reagit correctement si aucun chemin n'est
    trouve (cas_limite_couloir_bloque en particulier).
    """
    resultat = scenario_replanification(
        planner_name=planner_name,
        verbose=verbose,
        unexpected_obstacle=essai["obstacle"],
        obstacle_time=essai["obstacle_time"],
    )
    resultat["id_essai"] = essai["id"]
    resultat["planner"] = planner_name

    # --- Verification du SafetyManager sur ce resultat ---
    # scenario_replanification (Koja) expose "path_found_after_replan"
    # (champ ajoute par Role 5) : False = aucun chemin trouve apres
    # l'obstacle imprevu -> on simule ce que ferait le SafetyManager.
    aucun_chemin = not resultat.get("path_found_after_replan", True)
    resultat["aucun_chemin_trouve"] = aucun_chemin

    if aucun_chemin:
        sm = SafetyManager(tentatives_max_replanification=3)
        etat = None
        for _ in range(3):  # simule les tentatives de replanification
            etat = sm.check(_RobotFictif(), localization_uncertainty=0.05,
                             obstacle_distance=0.0, path_found=False)
        resultat["safety_reaction"] = etat.name
        resultat["safety_ok"] = (etat == EtatSurete.ARRET_SUR)
    else:
        resultat["safety_reaction"] = "non_declenche"
        resultat["safety_ok"] = True  # rien a verifier, pas de situation critique

    return resultat


class _RobotFictif:
    """Petit objet minimal pour tester SafetyManager hors boucle robot reelle."""
    def __init__(self):
        self.time = 0.0
        self.stopped = False
        self.v = 0.0
        self.omega = 0.0
        self.security = {}

    def emergency_stop(self):
        self.stopped = True
        self.v, self.omega = 0.0, 0.0

    def resume(self):
        self.stopped = False


# ======================================================================
# 4. Campagne complete + agregation
# ======================================================================

def lancer_campagne(planner_name, n_nominaux=10, verbose=False):
    essais = generer_essais_nominaux(n_nominaux) + CAS_LIMITES
    resultats = []
    for essai in essais:
        if verbose:
            print(f"  [{planner_name}] {essai['id']} ...")
        resultats.append(executer_essai(essai, planner_name, verbose=False))
    return resultats


def agreger(resultats):
    """Statistiques agregees sur un ensemble d'essais (un algo)."""
    n = len(resultats)
    succes = [r["success"] for r in resultats]
    temps_replan = [r["replan_time_ms"] for r in resultats if r["replan_count"] > 0]
    goal_dist = [r["goal_dist"] for r in resultats if r["goal_dist"] != float("inf")]
    dist_obstacles = [r["min_obstacle_dist"] for r in resultats if r["min_obstacle_dist"] != float("inf")]

    resume = {
        "nb_essais": n,
        "taux_succes": round(sum(succes) / n, 3),
        "temps_replanification_moy_ms": round(stats.mean(temps_replan), 2) if temps_replan else None,
        "temps_replanification_ecart_type_ms": (
            round(stats.stdev(temps_replan), 2) if len(temps_replan) > 1 else None
        ),
        "goal_dist_moy_m": round(stats.mean(goal_dist), 4) if goal_dist else None,
        "distance_min_obstacle_m": round(min(dist_obstacles), 4) if dist_obstacles else None,
        "nb_arrets_surs_attendus": sum(r["aucun_chemin_trouve"] for r in resultats),
        "nb_arrets_surs_confirmes": sum(
            r["aucun_chemin_trouve"] and r["safety_ok"] for r in resultats
        ),
    }
    return resume


# ======================================================================
# 5. Sauvegarde des resultats (results/features_experimentation/)
# ======================================================================

def sauvegarder(resultats_par_algo, resume_par_algo, base_dir=None):
    if base_dir is None:
        base_dir = os.path.join(config.RESULTS_DIR, "features_experimentation")
    os.makedirs(base_dir, exist_ok=True)

    # --- CSV detaille, un essai par ligne ---
    csv_path = os.path.join(base_dir, "campagne_essais.csv")
    tous = [r for essais in resultats_par_algo.values() for r in essais]
    if tous:
        champs = list(tous[0].keys())
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=champs)
            writer.writeheader()
            for r in tous:
                writer.writerow(r)

    # --- Resume texte, pour le rapport ---
    txt_path = os.path.join(base_dir, "resume_campagne.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("CAMPAGNE D'ESSAIS -- Role 5 (Tino), Experimentation/Surete\n")
        f.write("=" * 65 + "\n\n")
        for algo, resume in resume_par_algo.items():
            f.write(f"--- {algo.upper()} ---\n")
            for k, v in resume.items():
                f.write(f"  {k:<38}: {v}\n")
            f.write("\n")
        f.write("Cas limites couverts : chemin_le_plus_court, couloir_bloque\n")
        f.write("Cas limite NON teste : perte_balise "
                "(depend du module localisation, retire de l'equipe)\n")

    return csv_path, txt_path


# ======================================================================
# Point d'entree
# ======================================================================

def main():
    print("Campagne d'essais -- Role 5 (Tino)")
    print(f"{10} essais nominaux + {len(CAS_LIMITES)} cas limites, x2 algos\n")

    resultats_par_algo = {}
    resume_par_algo = {}

    for algo in ["astar", "rrt"]:
        print(f"--- Campagne {algo.upper()} ---")
        resultats = lancer_campagne(algo, n_nominaux=10, verbose=True)
        resultats_par_algo[algo] = resultats
        resume = agreger(resultats)
        resume_par_algo[algo] = resume
        print(f"  Taux de succes         : {resume['taux_succes']*100:.0f}%")
        print(f"  Replan moyen (ms)       : {resume['temps_replanification_moy_ms']}")
        print(f"  Arrets surs attendus/OK : "
              f"{resume['nb_arrets_surs_attendus']}/{resume['nb_arrets_surs_confirmes']}")
        print()

    csv_path, txt_path = sauvegarder(resultats_par_algo, resume_par_algo)
    print(f"Resultats sauvegardes :\n  - {csv_path}\n  - {txt_path}")


if __name__ == "__main__":
    main()
