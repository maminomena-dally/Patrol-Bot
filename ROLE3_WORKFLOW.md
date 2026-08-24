# 🎯 Role 3 — Planification & Commande : Suivi de travail
# Koja — M2 SDIA

> **Derniere mise a jour** : Phase 5 terminee (integration + comparaison + visualisation)
> **Deadline** : J+7
> **Statut global** : 🟢 Phases 1-5 terminees, Phase 6 a faire (tests finaux + PR)

> **Sources lues** :
> - ✅ Cadrage_Mini_Projet_Robotique_Mobile.pdf
> - ✅ Sujet-SDIA M2-2026-Mini_projet_Robotique_mobile.docx
> - ✅ Robotique_mobile_M2_2h_Dr_Randria.pdf

---

## 📌 Exigences OFFICIELLES pour le Role 3

### Ce que le prof attend de TOI
1. **A\*** : planification sur grille, optimal, deterministe (slide 25)
2. **RRT** : echantillonnage aleatoire, plus rapide en grand espace (slide 24)
3. **Comparaison** des deux sur : temps de calcul initial ET vitesse de replanification
4. **Replanification** quand obstacle imprévu detecte
5. **Pure Pursuit** : suivi de trajectoire (slide 26)
6. Patrouille entre **3-4 points de controle** (boucle)

### Critere de precision
- Robot atteint chaque point a **< 10 cm** (GOAL_TOLERANCE = 0.10 m)

### Metriques a fournir (pour Role 5 - Tino)
| Metrique | Description | A* | RRT |
|----------|-------------|-----|-----|
| Taux de succes | % essais ou tous points atteints sans collision | 100% (4/4) | 100% (4/4) |
| Temps de mission | Duree totale patrouille (s) | 189.3 | 200.0 |
| Longueur du trajet | Distance totale (m) | 50.13 | 53.33 |
| Dist. min aux obstacles | Plus petite distance robot-obstacle (m) | 0.000* | 0.000* |
| **Temps de replanification** | **Delai -> nouvelle trajectoire (ms)** | **238.6** | **259.9** |

*\* La distance est calculee avec marge = ROBOT_RADIUS, donc 0.000m signifie que le robot passe exactement a la distance de securite (inflation correcte).*

---

## Checklist principale

### Phase 1 — Setup & Comprehension ✅ TERMINE
- [x] Cloner le repo, installer les dependances, faire tourner `main.py`
- [x] Comprendre `Robot` et `Simulator`
- [x] Creer la branche `feature/planning`
- [x] Decommenter `numpy` dans `requirements.txt`
- [x] Ajouter les params planning dans `config.py`

### Phase 2 — A* ✅ TERMINE
- [x] Implementer `planning/astar.py` (classe `AStarPlanner`)
  - [x] Grille d'occupation avec inflation des obstacles (rayon robot)
  - [x] Algorithme A* 8-connecte, heuristique euclidienne
  - [x] Lissage du chemin (line-of-sight smoothing)
  - [x] Retourner chemin en coordonnees monde (metres)
  - [x] Mesurer le temps de calcul
- [x] Tests unitaires A* (`tests/test_planning.py`) — 14 tests
- **Commit** : `feat(planning): A* implemente avec inflation, lissage et 14 tests`

### Phase 3 — RRT ✅ TERMINE
- [x] Implementer `planning/rrt.py` (classe `RRTPlanner`)
  - [x] Arbre exploratoire avec extension par pas
  - [x] Goal bias vers le but
  - [x] Verification collision sur les segments
  - [x] Extraction et lissage du chemin
  - [x] Mesurer le temps de calcul
  - [x] Fonction utilitaire `grid_to_is_free()`
- [x] Tests unitaires RRT — 9 tests
- [x] Tests comparaison A* vs RRT — 3 tests
- **Commit** : `feat(planning): RRT implemente avec lissage et 9 tests (+ 3 comparaison)`

### Phase 4 — Pure Pursuit ✅ TERMINE
- [x] Implementer `control/pure_pursuit.py` (classe `PurePursuitController`)
  - [x] Trouver le lookahead point sur le chemin
  - [x] Calculer omega = (v/Ld) * sin(alpha)
  - [x] Arret quand but atteint (< 10 cm)
  - [x] Gestion chemin vide / deja arrive
  - [x] Ralentissement pres du but (ramp lineaire dans 2x lookahead)
  - [x] Methode `reset()` pour reinitialiser entre les segments
  - [x] Methode `goal_reached()` pour verifier l'arrivee
- [x] Tests unitaires Pure Pursuit — 8 tests
- [x] Tests integration A*+PP et RRT+PP — 2 tests
- **Commit** : `feat(control): Pure Pursuit implemente avec 15 tests (32 total)`

### Phase 5 — Integration ✅ TERMINE
- [x] Scenario de patrouille : A* + Pure Pursuit (4 waypoints)
- [x] Scenario de patrouille : RRT + Pure Pursuit (4 waypoints)
- [x] **Scenario replanification** : obstacle imprévu -> replan -> re-suivi
- [x] Comparaison A* vs RRT avec metriques (tableau recapitulatif)
- [x] Visualisation matplotlib (trajectoires PNG dans `results/features_planning/images/`)
- [x] Export CSV des logs robot dans `results/features_planning/logs/`
- [x] Tableau comparatif en texte dans `results/features_planning/comparaison.txt`
- [x] Tout integre dans `experiments/run_experiments.py` (framework de Malala respecte)
- **Commit** : `feat(experiments): integration Phase 5, patrouille + replanification + visualisation`

### Phase 6 — Tests & Livraison ⏸ A FAIRE
- [ ] Tous les tests passent (32/32)
- [ ] Resultats de comparaison pour Role 5 (Tino)
- [ ] Interface documentee pour Role 4 (Dally)
- [ ] Commit, push, PR vers `main`
- [ ] Preparer contribution pour la section 4 du rapport

---

## Historique des commits

| Date | Commit | Fichiers | Tests |
|------|--------|---------|-------|
| Jour 1 | `feat(planning): A* implemente avec inflation, lissage et 14 tests` | `requirements.txt`, `config.py`, `planning/astar.py`, `tests/test_planning.py`, `ROLE3_WORKFLOW.md` | 31 pass (17 kinematique + 14 A*) |
| Jour 2-3 | `feat(planning): RRT implemente avec lissage et 9 tests (+ 3 comparaison)` | `planning/rrt.py`, `tests/test_planning.py` | 40 pass (17 kin + 14 A* + 9 RRT) |
| Jour 3 | `feat(control): Pure Pursuit implemente avec 15 tests (32 total)` | `control/pure_pursuit.py`, `tests/test_planning.py`, `ROLE3_WORKFLOW.md` | 32 pass (17 kin + 15 planning/control) |
| Jour 4 | `feat(experiments): integration Phase 5, patrouille + replan + visu` | `experiments/run_experiments.py`, `ROLE3_WORKFLOW.md` | 32 pass + 4 scenarios OK |

---

## Journal de bord

### Jour 1 — Setup + A*
- **Fait** : Branche creee, config mise a jour, A* complet avec 14 tests, 31/31 tests passent
- **Bloquants** : Aucun
- **Decisions** : GOAL_TOLERANCE = 0.10 m (exigence < 10 cm du cadrage)

### Jour 2-3 — RRT
- **Fait** : RRT complet avec 9 tests + 3 tests comparaison A* vs RRT, 40/40 tests passent
- **Bloquants** : Aucun
- **Decisions** : RRT utilise `grid_to_is_free()` pour etre compatible avec les memes grilles que A*

### Jour 3 — Pure Pursuit
- **Fait** : Pure Pursuit complet avec 8 tests unitaires + 2 tests integration, 32/32 tests passent
- **Bloquants** : Aucun
- **Decisions** : Ralentissement pres du but (ramp lineaire), lookahead distance = 0.5m, v_cruise = 0.3 m/s

### Jour 4 — Integration (Phase 5)
- **Fait** :
  - Patrouille 4 waypoints avec A* et RRT (4/4 succes pour les deux)
  - Replanification obstacle imprévu (succes, ~210-260ms de replan)
  - Tableau comparatif A* vs RRT genere automatiquement
  - Visualisation matplotlib (4 PNG : patrol_astar, patrol_rrt, replan_astar, replan_rrt)
  - Logs CSV du robot pour chaque scenario
  - Tout dans `results/features_planning/` pour ne pas polluer le dossier results
- **Bloquants** : Aucun
- **Decisions** :
  - Integration faite DANS `experiments/run_experiments.py` de Malala (pas de nouveau fichier)
  - Scenario de Malala (`scenario_avancer_puis_tourner`) preservé intact
   - Matplotlib optionnel (try/except, avertissement si pas installe)
  - Replanification scriptee (timer t=8s) car la detection d'obstacle est le role de l'equipe Perception + Surete (voir note ci-dessous)
- **Note importante sur la replanification** :
  - Dans notre scenario, l'obstacle apparait a un instant fixe (t=8s) car nous testons la **capacite de replanification** du module planning en isolation.
  - Dans le systeme final integre, la chaine sera : `Lidar (Kojy) -> SafetyManager (equipe Surete) -> A*.plan() / RRT.plan() (Koja) -> PurePursuit (Koja)`.
  - La detection d'obstacle et la decision de replanifier NE SONT PAS le role du Role 3.

---

## Interface publique de tes modules

### AStarPlanner
```python
from planning.astar import AStarPlanner

planner = AStarPlanner(
    grid: np.ndarray,           # 2D array, 0=libre, 1=obstacle
    resolution: float = 0.1,    # m par cellule
    robot_radius: float = 0.18  # inflation des obstacles
)
path: list[tuple[float, float]] = planner.plan(
    start: tuple[float, float],  # (x, y) en metres
    goal: tuple[float, float]    # (x, y) en metres
)  # -> liste de points, ou [] si pas de chemin
# planner.last_plan_time_ms = temps de calcul en ms
```

### RRTPlanner
```python
from planning.rrt import RRTPlanner

planner = RRTPlanner(
    is_free: callable,           # (x, y) -> bool
    bounds: tuple,              # (x_min, y_min, x_max, y_max)
    step_size: float = 0.3,
    max_iter: int = 2000,
    goal_bias: float = 0.10,
    goal_tolerance: float = 0.3
)
path: list[tuple[float, float]] = planner.plan(
    start: tuple[float, float],
    goal: tuple[float, float]
)  # -> liste de points, ou [] si pas de chemin
# planner.last_plan_time_ms = temps de calcul en ms
```

### PurePursuitController
```python
from control.pure_pursuit import PurePursuitController

controller = PurePursuitController(
    lookahead_distance: float = 0.5,
    v_cruise: float = 0.3,
    goal_tolerance: float = 0.10  # <= 10 cm comme exige
)
v: float, omega: float = controller.compute_command(
    pose: tuple[float, float, float],  # (x, y, theta)
    path: list[tuple[float, float]]   # chemin du planificateur
)  # -> (0, 0) si but atteint
controller.goal_reached(
    position: tuple[float, float],   # (x, y)
    path: list[tuple[float, float]]
)  # -> bool
controller.reset()  # reinitialiser avant un nouveau segment
```

### Utilitaires (pour experiments ou integration)
```python
from planning.astar import create_test_grid   # obstacles dicts -> numpy grid
from planning.rrt import grid_to_is_free       # numpy grid -> is_free callable
```

---

## 📋 Guide pour Role 4 (Dally) — Integration

> Dally, voici comment brancher mes modules dans la boucle de simulation.
> Tout est pret, tu n'as qu'a instancier et appeler.

### Etape 1 : Creer la carte (grille d'occupation)

```python
from planning.astar import create_test_grid
import config

# obstacles = liste de dicts, ex:
# [ {"type": "rect", "x": 9.5, "y": 0.0, "w": 0.3, "h": 10.0} ]
# Tu peux aussi construire la grille toi-meme (numpy 2D, 0=libre, 1=obstacle)

grid = create_test_grid(
    config.WORLD_WIDTH,      # 20.0 m
    config.WORLD_HEIGHT,     # 15.0 m
    config.GRID_RESOLUTION,  # 0.1 m
    obstacles,               # liste d'obstacles
)
```

### Etape 2 : Creer un planificateur (A* ou RRT)

```python
from planning.astar import AStarPlanner
from planning.rrt import RRTPlanner, grid_to_is_free
import config

# --- Option A : A* (optimal, sur grille) ---
planner_astar = AStarPlanner(
    grid,
    resolution=config.GRID_RESOLUTION,
    robot_radius=config.ROBOT_RADIUS,
    eight_connected=config.ASTAR_8_CONNECTED,
)

# --- Option B : RRT (echantillonnage, espace continu) ---
is_free = grid_to_is_free(grid, config.GRID_RESOLUTION, config.ROBOT_RADIUS)
planner_rrt = RRTPlanner(
    is_free=is_free,
    bounds=(0, 0, config.WORLD_WIDTH, config.WORLD_HEIGHT),
    step_size=config.RRT_STEP_SIZE,
    max_iter=config.RRT_MAX_ITER,
    goal_bias=config.RRT_GOAL_BIAS,
    goal_tolerance=config.RRT_GOAL_TOLERANCE,
    seed=42,
)
```

### Etape 3 : Creer le controleur

```python
from control.pure_pursuit import PurePursuitController

controller = PurePursuitController(
    lookahead_distance=config.LOOKAHEAD_DISTANCE,  # 0.5 m
    v_cruise=config.V_CRUISE,                        # 0.3 m/s
    goal_tolerance=config.GOAL_TOLERANCE,             # 0.10 m
)
```

### Etape 4 : Brancher dans la boucle de simulation

```python
from robot.robot import Robot
from simulation.simulator import Simulator

robot = Robot(initial_pose=(2.0, 2.0, 0.0))
sim = Simulator(robot)

# Plan initial
start = robot.get_true_pose()[:2]
goal = (17.0, 12.0)
current_path = planner_astar.plan(start=start, goal=goal)
controller.reset()

# Command function pour la boucle
def command_fn(r, t):
    pose = r.get_true_pose()  # ou pose estimee (Role 2 - Kojy)
    
    if controller.goal_reached(pose[:2], current_path):
        # But atteint -> planifier vers le prochain waypoint
        # ou appeler robot.set_velocity(0, 0) pour s'arreter
        r.set_velocity(0.0, 0.0)
        return
    
    v, omega = controller.compute_command(pose=pose, path=current_path)
    r.set_velocity(v, omega)

sim.run(duration=120.0, command_fn=command_fn)
```

### Etape 5 : Replanification (quand un obstacle est detecte)

```python
# Quand le SafetyManager ou le lidar detecte un obstacle bloquant :
# 1. Mettre a jour la grille avec le nouvel obstacle
# 2. Replanifier depuis la position actuelle

new_grid = create_test_grid(config.WORLD_WIDTH, config.WORLD_HEIGHT,
                             config.GRID_RESOLUTION, updated_obstacles)
planner_astar = AStarPlanner(new_grid, resolution=config.GRID_RESOLUTION,
                             robot_radius=config.ROBOT_RADIUS)

current_pose = robot.get_true_pose()
new_path = planner_astar.plan(start=current_pose[:2], goal=goal)

if not new_path:
    # Pas de chemin -> le SafetyManager doit faire robot.emergency_stop()
    robot.emergency_stop()
else:
    controller.reset()
    # La boucle de simulation suit maintenant new_path
```

### Points d'attention pour Dally
- **Le chemin est en coordonnees monde (metres)**, pas en indices de grille
- **`planner.plan()` retourne `[]` (liste vide) si pas de chemin** — toujours verifier
- **`planner.last_plan_time_ms`** donne le temps de calcul en ms (pour les metriques)
- **`controller.reset()`** doit etre appele avant chaque nouveau chemin
- **`controller.goal_reached()`** verifie si le robot est arrive (distance < 0.10m)
- **Pure Pursuit retourne `(v, omega)`** — appliquer via `robot.set_velocity(v, omega)`
- **Ne pas modifier `planning/` ni `control/`** — si tu as besoin de quelque chose, dis-le moi
- Les **params sont tous dans `config.py`** — ne les hardcode pas

---

## 📋 Guide pour Role 5 (Tino) — Experimentation

> Tino, voici les resultats et comment les utiliser pour le rapport (section 4).

### Resultats deja prets

Les resultats sont dans `results/features_planning/` :

```
results/features_planning/
    comparaison.txt              ← Tableau A* vs RRT (texte, pour le rapport)
    images/
        patrol_astar.png         ← Trajectoire patrouille A* (a mettre dans le rapport)
        patrol_rrt.png           ← Trajectoire patrouille RRT
        replan_astar.png         ← Trajectoire replanification A*
        replan_rrt.png           ← Trajectoire replanification RRT
    logs/
        patrol_astar.csv         ← Log complet du robot (x, y, theta, v, omega, t)
        patrol_rrt.csv
        replan_astar.csv
        replan_rrt.csv
```

### Tableau comparatif (extrait de comparaison.txt)

```
Metrique                                         A*        RRT
  PATROUILLE
  Succes                                          OUI        OUI
  Waypoints atteints                                4          4
  Temps mission (s)                               189.3      200.0
  Longueur chemin (m)                              50.13      53.33
  Planif moyenne (ms/wp)                           63.1       42.1
  Dist min obstacles (m)                          0.000      0.000
  REPLANIFICATION
  Succes replan                                   OUI        OUI
  Temps replan (ms)                               238.6      259.9
  Dist au but final (m)                           0.0993     0.0995
  Temps total (s)                                  72.8       76.5
```

### Comment relancer les experiences

```bash
# Relancer tout (patrouille + replan + comparaison + graphiques + CSV)
python -m experiments.run_experiments

# Ou lancer un scenario individuel :
python -c "
from experiments.run_experiments import scenario_patrouille, scenario_replanification
resultat = scenario_patrouille(planner_name='astar', verbose=True)
print(resultat)
"
```

### Points a tester / verifier pour Tino

1. **Taux de succes** : nos 2 experiences montrent 100% (4/4 waypoints), mais tu pourrais relancer plusieurs fois pour verifier la reproductibilite (surtout pour RRT qui est aleatoire — bien que seed=42 le rend deterministe)

2. **Temps de replanification** : mesure le temps TOTAL de la replanification (recree la grille + recree le planificateur + calcule le nouveau chemin). Nos resultats : A* ~239ms, RRT ~260ms.

3. **Precision** : le robot atteint chaque waypoint a < 0.10m (GOAL_TOLERANCE). Verifiable dans les CSV logs (colonne x, y vs coordonnees des waypoints).

4. **Graphiques** : les PNG dans `images/` montrent la trajectoire bleue du robot, les obstacles gris, les waypoints verts, et l'obstacle imprévu rouge pointillé (replanification).

5. **Metriques manquantes** : si tu as besoin de metriques supplementaires (ex: 10 essais statistiques, ecart-type, etc.), dis-le moi et je peux ajouter un mode "N essais" aux scenarios.

### Interface pour acceder aux fonctionnalites

```python
# Import direct — tout est accessible sans passer par la simulation
from planning.astar import AStarPlanner, create_test_grid
from planning.rrt import RRTPlanner, grid_to_is_free
from control.pure_pursuit import PurePursuitController
from experiments.run_experiments import (
    scenario_patrouille,           # -> dict de metriques
    scenario_replanification,     # -> dict de metriques
    scenario_comparaison,         # -> dict complet + affichage console
)

# Exemple : lancer la patrouille A* et recuperer les metriques
resultat = scenario_patrouille(planner_name='astar', verbose=True)
print(resultat["success"])          # True/False
print(resultat["mission_time"])    # secondes
print(resultat["total_path_length"])  # metres
print(resultat["plan_times_ms"])   # liste des temps par waypoint

# Exemple : lancer la replanification
resultat = scenario_replanification(planner_name='rrt', verbose=True)
print(resultat["replan_time_ms"])  # temps de replan en ms
print(resultat["success"])          # a-t-il atteint le but apres l'obstacle?

# Exemple : lancer la comparaison complete (genere aussi les fichiers)
resultats = scenario_comparaison(verbose=True)
```

---

## Notes de collaboration

### Interface avec les autres roles
- **Role 1 (Malala - Cinematique)** : ✅ Termine — `Robot.set_velocity(v, omega)`, `Robot.get_true_pose()`
- **Role 2 (Kojy - Perception/Localisation)** : ⏳ En cours — mon Pure Pursuit utilise `get_true_pose()` en test, puis basculera sur la pose estimee de Kojy en integration
- **Role 4 (Dally - Simulation/Integration)** : ⏳ En cours — guide complet ci-dessus. Dally branche mes modules dans `simulation/simulator.py` via les callbacks `on_plan`.
- **Role 5 (Tino - Experimentation)** : ⏳ En cours — resultats + guide ci-dessus. Les fichiers dans `results/features_planning/` sont prets pour le rapport.
- **Equipe Surete (safety/)** : ⏳ Stub — leur `SafetyManager.check()` appellera mon `planner.plan()` pour la replanification. Mon code est pret pour ca.
- **Equipe Securite (security/)** : ⏳ Stub — pas d'interface directe avec mes modules.

### Convention Git
- Branche : `feature/planning`
- Messages : `feat(planning): ...` / `feat(control): ...` / `feat(experiments): ...` / `fix(...): ...`
- Ne jamais modifier `robot/`, `simulation/`, `gui/` sans concertation
- `config.py` est partage -> previens le groupe des ajouts

### Fichiers modifies par Role 3
| Fichier | Role | Description |
|---------|------|-------------|
| `planning/astar.py` | Nouveau | Planificateur A* avec inflation et lissage |
| `planning/rrt.py` | Nouveau | Planificateur RRT avec lissage |
| `control/pure_pursuit.py` | Nouveau | Controleur Pure Pursuit |
| `tests/test_planning.py` | Nouveau | 32 tests (A*, RRT, PP, integration) |
| `experiments/run_experiments.py` | Modifie (ajouts) | 3 scenarios + comparaison + visualisation (code de Malala preservé) |
| `config.py` | Modifie (ajouts) | Params planning (WORLD_*, ASTAR_*, RRT_*, LOOKAHEAD_*, etc.) |
| `requirements.txt` | Modifie | Decommenté numpy |
| `ROLE3_WORKFLOW.md` | Nouveau | Ce fichier |