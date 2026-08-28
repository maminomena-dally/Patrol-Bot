# 🎯 Role 3 — Planification & Commande : Suivi de travail
# Koja — M2 SDIA

> **Derniere mise a jour** : Phase 7 terminee — Module Securite (Role 1) implemente + doc pour l'equipe
> **Deadline** : J+7
> **Statut global** : 🟢 Phases 1-7 terminees, Phase 6 a faire (tests finaux + PR)

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

---

## 🆕 Phase 7 — Module Securite / Role 1 (TERMINE)

### Contexte
Le Role 1 (Securite) n'etait assigne a personne. Koja l'a pris en charge et a implemente le pipeline complet de securite : detection d'intrusion, gestion d'alertes, et alarme sonore.

### Pipeline complet de securite

```
Camera.observe(targets)             # Tino (Role 2) ou stub
    |
    v
IntrusionDetector.check(           # Koja (Role 1)
    observations, robot_pose,       #   - Filtre FOV
    camera_fov_deg=90.0)            #   - Filtre obstacles connus
    |                                #   - Deduplication + cooldown
    v                                #
[IntrusionAlert, ...]              # -> positions + distances
    |
    v
AlertManager.update(              # Koja (Role 1)
    alerts, robot_pose)             #   - Niveaux: NOMINAL/INFO/WARNING/DANGER
    |                                #   - warning_dist=4.0m, danger_dist=2.0m
    v                                #
AlertEvent                         # -> niveau + message + position
    |
    +---> am.get_intrusion_confirmed()  --> SafetyManager.check()  [Dally - Role 4]
    +---> am.should_alarm()              --> speaker.update()      [Speaker]
    |
    v
Speaker.update(                   # Koja (Role 1)
    should_alarm, level_name)       #   - Pattern: NONE / SLOW / FAST
    |                                #   - Anti-faux-positifs (threshold=2)
    v                                #
AlarmPattern                      # -> NONE (silence) / SLOW (bip lent) / FAST (bip rapide)
```

### Niveaux d'alerte (AlertLevel)

| Niveau | Distance | Description | Alarme |
|--------|----------|-------------|--------|
| NOMINAL | — | Aucune intrusion | Silence |
| INFO | > 4.0m | Intrusion lointaine | Silence |
| WARNING | 2.0m - 4.0m | Intrusion a distance moyenne | Bip lent (SLOW) |
| DANGER | <= 2.0m | Intrusion imminente | Bip rapide (FAST) |

### Ce qui a ete fait
- [x] `security/intrusion_detector.py` — IntrusionAlert + IntrusionDetector
  - Filtrage par FOV (champ de vision de la camera)
  - Filtrage des obstacles connus (racks) avec zone de tolerance
  - Deduplication + cooldown (evite les alertes repetees)
- [x] `security/alert_manager.py` — AlertLevel + AlertEvent + AlertManager
  - 4 niveaux de gravite (NOMINAL → INFO → WARNING → DANGER)
  - Distances configurables (warning_dist=4.0, danger_dist=2.0)
  - Resolution delay (ne revient pas a NOMINAL instantanement)
  - Le niveau ne descend pas instantanement (securite)
- [x] `security/speaker.py` — AlarmPattern + Speaker
  - 3 patterns: NONE / SLOW / FAST
  - Anti-faux-positifs (activation_threshold=2)
  - Statistiques: trigger_count, total_alarm_time
- [x] `security/__init__.py` — Imports publics du module
- [x] 29 tests unitaires + 4 tests integration (tous passent)
  - `tests/test_intrusion_detector.py` — 12 unit + 1 integration
  - `tests/test_alert_manager.py` — 11 unit + 1 integration
  - `tests/test_speaker.py` — 6 unit + 1 integration (pipeline complet)

### Bridges vers les autres roles

| Bridge | De | Vers | Usage |
|--------|----|------|-------|
| `detector.get_intrusion_positions()` | IntrusionDetector | A*/RRT (Role 3) | Obstacles dynamiques pour replanification |
| `detector.is_close_intrusion(d)` | IntrusionDetector | SafetyManager (Role 4) | Arret d'urgence si intrus trop proche |
| `am.get_intrusion_confirmed()` | AlertManager | SafetyManager.check() (Role 4) | Arret d'urgence si WARNING/DANGER |
| `am.should_alarm()` | AlertManager | Speaker.update() | Declenche/maintient l'alarme sonore |

### Fichiers du module securite

| Fichier | Role | Description |
|---------|------|-------------|
| `security/intrusion_detector.py` | Nouveau | IntrusionAlert + IntrusionDetector (FOV, filtres, dedup) |
| `security/alert_manager.py` | Nouveau | AlertLevel + AlertEvent + AlertManager (4 niveaux) |
| `security/speaker.py` | Nouveau | AlarmPattern + Speaker (NONE/SLOW/FAST) |
| `security/__init__.py` | Nouveau | Imports publics |
| `tests/test_intrusion_detector.py` | Nouveau | 12 unit + 1 integration |
| `tests/test_alert_manager.py` | Nouveau | 11 unit + 1 integration |
| `tests/test_speaker.py` | Nouveau | 6 unit + 1 integration pipeline complet |

---

## 🆕 Phase 5b — Integration Role 2 ↔ Role 3 (TERMINE)

### Ce qui a ete fait
L'objectif etait de connecter la localisation EKF de Kojy (Role 2) avec la planification + commande de Koja (Role 3). **Le pipeline complet fonctionne :**

```
Odometry.read(d_l, d_r)
    → EKF.predict(d_l, d_r)
        → LandmarkDetector.detect()
            → EKF.correct(observations)
                → PurePursuit.compute_command(pose=ESTIMATED)  ← clef !
                    → Robot.step(dt)
```

**Point critique** : Le controleur Pure Pursuit utilise la **pose estimee par l'EKF**, pas la vraie pose. C'est le comportement reel du systeme.

### Fichiers crees / modifies
| Fichier | Action | Description |
|---------|--------|-------------|
| `tests/test_integration_role2_role3.py` | Nouveau | 6 tests + 4 scenarios visuels (main) |
| `localization/localization.py` | Reecrit | EKF 3×3 complet (Jacobien F/Q, Joseph form) |
| `config.py` | Modifie | `LANDMARK_DETECTION_RADIUS = 6.0` (etait 2.0) |

### Tests unitaires — 6/6 PASS
```
test_warehouse_grid_creation          PASS
test_astar_finds_path_in_warehouse    PASS
test_rrt_finds_path_in_warehouse      PASS
test_localization_pipeline_works      PASS
test_full_patrol_one_waypoint_localized  PASS
test_safety_blocks_on_full_blockage   PASS
```

### Resultats de l'integration (extraits)

**Patrouille A* avec localisation EKF :**
```
  WP1 (1.0, 9.75):  1 pts → OK (true_d=0.050m, loc_err_max=0.000m)
  WP2 (18.0, 9.75): 23 pts → OK (true_d=0.123m, loc_err_max=0.076m)
  WP3 (18.0, 4.75):  8 pts → OK (true_d=0.116m, loc_err_max=0.059m)
  WP4 (1.0, 4.75):  23 pts → OK (true_d=0.091m, loc_err_max=0.087m)
  Resultat : SUCCES — 4/4 waypoints, 100% zones, incertitude finale 0.026m
```

**Metriques cles :**
| Metrique | A* | RRT |
|----------|----|----|
| Waypoints atteints | 4/4 | 4/4 |
| Erreur loc. max (m) | 0.087 | ~0.09 |
| Incertitude finale (m) | 0.026 | ~0.03 |
| Zones couvertes | 100% | 100% |
| Temps mission (s) | 145.3 | ~150 |

### Resultats visuels generes
Le fichier `tests/test_integration_role2_role3.py` contient un `main()` qui genere :

```
results/warehouse_integration/
    warehouse_layout.png              Plan de l'entrepot (9 etageres, 4 zones, 9 balises, 4 WPs)
    patrol_astar_localized.png       Patrouille A* : paths planifies (orange) + trajectoire reelle (bleu) + EKF (rouge)
    patrol_rrt_localized.png        Patrouille RRT : meme chose
    dynamic_blockage.png             Path Normale (orange) + Path Anticipation blocage (gris) + obstacle (rouge)
    extreme_blockage.png             Path original (orange) + blocage total (rouge) + ARRET_SUR (X noir)
    localization_error_astar.png     Graphique erreur position (m) et erreur cap (deg) vs temps
    ekf_comparison_astar.csv         CSV true vs estime (t, x, y, theta, err_pos, err_theta)
    patrol_astar_log.csv             Log robot complet
    patrol_rrt_log.csv              Log robot complet
    mission_report.txt               Rapport texte avec zones, %, alertes
```

**Comment lire les images :**
- **🟠 Orange** = Path Normal (chemin planifie par A* ou RRT)
- **🔵 Bleu** = Trajectoire reelle du robot (vraie pose, pour comparaison)
- **🔴 Rouge pointille** = Trajectoire EKF estimee (ce que le robot "croit" etre)
- **Lignes grises** reliant bleu↔rouge = erreur de localisation (plus petit = meilleur EKF)
- **灰色 Gris** = Path Anticipation blocage (chemin replanifie apres obstacle)
- **🟡 Jaune** = Balises (landmarks pour recalage EKF)
- **🟢 Triangle vert** = Depart / **🟥 Carre rouge** = Arrivee
- **X noir** = Point d'ARRET_SUR (blocage extreme)

**Ce qui prouve que Role 2↔3 est connecte :**
1. L'erreur de localisation reste < 0.1m sur toute la mission (EKF converge bien)
2. Le robot atteint 4/4 waypoints en utilisant la POSE ESTIMEE (pas la vraie)
3. La trajectoire bleu (reelle) suit le path orange (planifie) avec un decalage minime
4. Le blocage dynamique declenche une replanification avec succes
5. Le blocage extreme declenche ARRET_SUR correctement

### Notes techniques sur l'EKF
- Etat 3×3 : `[x, y, theta]`
- Jacobien F pour la prediction, Jacobien H pour la correction
- Forme de Joseph pour la mise a jour de covariance (stabilite numerique)
- Mise a jour sequentielle des observations (une balise a la fois)
- `LANDMARK_DETECTION_RADIUS = 6.0m` — **critique** : avec 2.0m, le robot ne voyait aucune balise
- Propriete `uncertainty` retourne `max(std_x, std_y)` pour compatibilite SafetyManager

---

## 📌 Metriques completes (pour Role 5 - Tino)

### Phase 5 — Planning seul (sans localisation)
| Metrique | Description | A* | RRT |
|----------|-------------|-----|-----|
| Taux de succes | % essais ou tous points atteints sans collision | 100% (4/4) | 100% (4/4) |
| Temps de mission | Duree totale patrouille (s) | 189.3 | 200.0 |
| Longueur du trajet | Distance totale (m) | 50.13 | 53.33 |
| Temps de replanification | Delai -> nouvelle trajectoire (ms) | 238.6 | 259.9 |

### Phase 5b — Avec localisation EKF
| Metrique | A* | RRT |
|----------|----|----|
| Taux de succes | 100% (4/4) | 100% (4/4) |
| Erreur loc. max (m) | 0.087 | ~0.09 |
| Incertitude finale EKF (m) | 0.026 | ~0.03 |
| Zones couvertes | 100% | 100% |
| Temps mission (s) | 145.3 | ~150 |
| Replan dynamique | Succes | — |
| Blocage extreme | ARRET_SUR OK | — |

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

### Phase 5 — Integration (sans localisation) ✅ TERMINE
- [x] Scenario de patrouille : A* + Pure Pursuit (4 waypoints)
- [x] Scenario de patrouille : RRT + Pure Pursuit (4 waypoints)
- [x] **Scenario replanification** : obstacle imprévu -> replan -> re-suivi
- [x] Comparaison A* vs RRT avec metriques
- [x] Visualisation matplotlib (trajectoires PNG)
- [x] Export CSV des logs robot
- [x] Tout integre dans `experiments/run_experiments.py`
- **Commit** : `feat(experiments): integration Phase 5, patrouille + replanification + visualisation`

### Phase 5b — Integration Role 2 ↔ Role 3 ✅ TERMINE
- [x] EKF 3×3 reecrit (Jacobien, Joseph form, corrections sequentielles)
- [x] `LANDMARK_DETECTION_RADIUS` corrige 2.0 → 6.0
- [x] Pipeline complet : Odometry → EKF → A*/RRT → Pure Pursuit (pose estimee) → Robot
- [x] 4 scenarios visuels avec localisation bruitee
  - [x] Patrouille A* localisee (4/4 WP, err < 0.1m)
  - [x] Patrouille RRT localisee (4/4 WP, err < 0.1m)
  - [x] Blocage dynamique avec replanification
  - [x] Blocage extreme avec ARRET_SUR
- [x] Visualisations : paths planifies (orange) + replanifies (gris) + trajectoires
- [x] Graphiques erreur de localisation EKF vs temps
- [x] CSV comparatif true vs estime
- [x] 6 tests unitaires integration — 6/6 PASS
- [x] Couverture de zone : 100% (4/4 zones visitees)
- **Commit** : `feat(integration): Role 2↔3 pipeline EKF + A*/RRT + Pure Pursuit`

### Phase 7 — Module Securite / Role 1 ✅ TERMINE
- [x] IntrusionDetector avec FOV, filtres, dedup, cooldown
- [x] AlertManager avec 4 niveaux (NOMINAL/INFO/WARNING/DANGER)
- [x] Speaker avec 3 patterns (NONE/SLOW/FAST)
- [x] 29 tests unitaires + 4 tests integration
- [x] Pipeline complet valide (Camera → Detector → AM → Speaker)
- [x] Bridges documentes vers Dally (SafetyManager) et Koja (A*/RRT)
- **Commit** : `feat(security): pipeline securite complet (detecteur + alertes + speaker)`

### Phase 6 — Tests & Livraison ⏸ A FAIRE
- [x] Tous les tests passent (32 planning + 6 integration + 29 securite + 4 securite integration = 71)
- [x] Resultats de comparaison pour Role 5 (Tino) — **PRETS** (voir sections ci-dessous)
- [x] Interface documentee pour Role 4 (Dally) — **PRETE** (voir guide ci-dessous)
- [x] Commit, push, PR vers `main`
- [x] Preparer contribution pour la section 4 du rapport

---

## Historique des commits

| Date | Commit | Fichiers | Tests |
|------|--------|---------|-------|
| Jour 1 | `feat(planning): A* implemente avec inflation, lissage et 14 tests` | `requirements.txt`, `config.py`, `planning/astar.py`, `tests/test_planning.py` | 31 pass |
| Jour 2-3 | `feat(planning): RRT implemente avec lissage et 9 tests (+ 3 comparaison)` | `planning/rrt.py`, `tests/test_planning.py` | 40 pass |
| Jour 3 | `feat(control): Pure Pursuit implemente avec 15 tests (32 total)` | `control/pure_pursuit.py`, `tests/test_planning.py` | 32 pass |
| Jour 4 | `feat(experiments): integration Phase 5, patrouille + replan + visu` | `experiments/run_experiments.py` | 32 pass + 4 scenarios OK |
| Jour 5 | `feat(integration): Role 2↔3 pipeline EKF + A*/RRT + Pure Pursuit` | `tests/test_integration_role2_role3.py`, `localization/localization.py`, `config.py` | 6/6 pass + 4 scenarios visuels |
| Jour 6 | `feat(security): pipeline securite complet (detecteur + alertes + speaker)` | `security/intrusion_detector.py`, `security/alert_manager.py`, `security/speaker.py`, `tests/test_intrusion_detector.py`, `tests/test_alert_manager.py`, `tests/test_speaker.py` | 29 unit + 4 integration = 33 pass |

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
- **Decisions** : Ralentissement pres du but (ramp lineaire), lookahead = 0.5m, v_cruise = 0.3 m/s

### Jour 4 — Integration Phase 5 (sans localisation)
- **Fait** : Patrouille 4 WP, replanification, comparaison A* vs RRT, 4 PNG, CSV, tableau comparatif
- **Bloquants** : Aucun
- **Decisions** : Integration dans `experiments/run_experiments.py` de Malala (code preservé)

### Jour 5 — Integration Phase 5b (Role 2 ↔ Role 3)
- **Fait** :
  - EKF 3×3 complet reecrit (l'ancien etait un filtre scalaire qui ne corrigeait pas theta)
  - Pipeline complet testé : Odometry → EKF → Planner → Pure Pursuit (pose estimee) → Robot
  - 6/6 tests integration passent
  - 4 scenarios visuels avec paths planifies (orange) + replanifies (gris)
  - Erreur de localisation < 0.1m sur toute la mission
  - Couverture de zone 100%
  - Blocage dynamique : replanification OK
  - Blocage extreme : ARRET_SUR declenche correctement
- **Bloquants** : Aucun
- **Decisions** :
  - Le test utilise la POSE ESTIMEE pour le controle (comportement reel du systeme)
  - L'EKF est initialise avec la vraie pose (comme si un GPS initial etait disponible)
  - `LANDMARK_DETECTION_RADIUS = 6.0m` car 2.0m ne detectait aucune balise (seulement 15m de traverse)
  - 4 zones de mission definies pour mesurer la couverture spatiale
- **Bug critique resolu** : L'ancien filtre de localisation ne corrigeait que x,y (pas theta), causant 7.69m d'erreur. L'EKF 3×3 corrige les 3 etats.
- **Bug silencieux resolu** : `LANDMARK_DETECTION_RADIUS = 2.0` faisait que le robot ne voyait ZERO balise sur 15 des 17m de parcours.

### Jour 6 — Module Securite / Role 1
- **Fait** :
  - Role 1 (Securite) non assigne -> Koja le prend en charge
  - 3 modules implementes : IntrusionDetector, AlertManager, Speaker
  - Pipeline complet : Camera → Detector → AM → Speaker
  - 29 tests unitaires + 4 tests integration (tous passent)
  - 4 niveaux d'alerte : NOMINAL, INFO, WARNING, DANGER
  - 3 patterns d'alarme : NONE, SLOW, FAST
  - Bridges documentes vers Dally (SafetyManager) et Koja (A*/RRT replanification)
- **Bloquants** : Aucun
- **Decisions** :
  - `warning_dist=4.0m`, `danger_dist=2.0m` (configurables)
  - Anti-faux-positifs : speaker `activation_threshold=2` (il faut 2 updates consecutifs)
  - Le niveau d'alerte ne descend pas instantanement (resolution_delay=1.0s)
  - `IntrusionDetector` filtre les obstacles connus avec une zone de tolerance (defaut 1.0m)
  - Deduplication : meme intrus ne declenche pas d'alerte pendant le cooldown (defaut 2.0s)

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
    pose: tuple[float, float, float],  # (x, y, theta) — peut etre la pose ESTIMEE par EKF
    path: list[tuple[float, float]]
)  # -> (0, 0) si but atteint
controller.goal_reached(
    position: tuple[float, float],
    path: list[tuple[float, float]]
)  # -> bool
controller.reset()
```

### Utilitaires
```python
from planning.astar import create_test_grid   # obstacles dicts -> numpy grid
from planning.rrt import grid_to_is_free       # numpy grid -> is_free callable
```

---

## 📋 Guide pour Role 4 (Dally) — Integration

> Dally, voici comment brancher mes modules dans la boucle de simulation.
> Tout est pret, y compris l'integration avec la localisation de Kojy.

### Pipeline complet du systeme

```
                  +-----------+
Robot.step() <--- | Pure      | <--- planner.plan()
    |             | Pursuit   |         |
    v             | Controller|    +----+----+
True Pose        +-----------+    | A* ou  | <--- create_test_grid()
    |                                | RRT    |     (obstacles)
    v                                +--------+
Odometry.read(d_l, d_r)                 |
    |                                    v
    v                              planner.plan()
EKF.predict(d_l, d_r)               (start, goal)
    |                                    |
    v                                    v
LandmarkDetector.detect()           path: [(x,y), ...]
    |                                    |
    v                                    v
EKF.correct(observations)       controller.compute_command(
    |                                   pose=ESTIMATED,  ← IMPORTANT
    v                                   path=path)
EKF.estimated_pose  -----------------------+
(x, y, theta)
```

### Etape 1 : Creer la carte (grille d'occupation)

```python
from planning.astar import create_test_grid
import config

# --- Option A : utiliser TON entrepot (entrepot_patrouille.py) ---
from entrepot_patrouille import WAREHOUSE_OBSTACLES, WAREHOUSE_WAYPOINTS
obstacles = WAREHOUSE_OBSTACLES  # 12 racks + murs + zone reservee
waypoints = WAREHOUSE_WAYPOINTS    # 8 points de patrouille

# --- Option B : utiliser les obstacles passes en parametre ---
# obstacles = [...]  # ta liste de dicts {"type": "rect", "x", "y", "w", "h"}

grid = create_test_grid(
    config.WORLD_WIDTH,      # 20.0 m
    config.WORLD_HEIGHT,     # 15.0 m
    config.GRID_RESOLUTION,  # 0.1 m
    obstacles,
)
```

> **Note** : Mes modules A*, RRT et Pure Pursuit marchent avec **n'importe quel layout**.
> Ils prennent les obstacles en parametre et ne sont pas couples a un entrepot specifique.
> Dans mes tests d'integration, j'utilise mon propre layout (9 gros racks, 4 WPs) mais
> le code est 100% compatible avec ton `entrepot_patrouille.py` (12 racks, 8 WPs).

> **Attention** : `entrepot_patrouille.py` ne definit pas de balises (landmarks).
> Pour la localisation EKF, Dally devra ajouter des balises dans son entrepot,
> ou Kojy devra fournir une liste compatible. Voir avec Kojy.

### Etape 2 : Creer un planificateur (A* ou RRT)

```python
from planning.astar import AStarPlanner
from planning.rrt import RRTPlanner, grid_to_is_free

# --- A* (optimal, deterministe) ---
planner = AStarPlanner(
    grid,
    resolution=config.GRID_RESOLUTION,
    robot_radius=config.ROBOT_RADIUS,
    eight_connected=config.ASTAR_8_CONNECTED,
)

# --- RRT (echantillonnage, espace continu) ---
is_free = grid_to_is_free(grid, config.GRID_RESOLUTION, config.ROBOT_RADIUS)
planner = RRTPlanner(
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
    lookahead_distance=config.LOOKAHEAD_DISTANCE,
    v_cruise=config.V_CRUISE,
    goal_tolerance=config.GOAL_TOLERANCE,
)
```

### Etape 4 : Boucle de simulation avec localisation EKF

```python
from robot.robot import Robot
from sensors.odometry import Odometry
from sensors.landmarks import LandmarkDetector
from localization.localization import Localizer

robot = Robot(initial_pose=(2.0, 2.0, 0.0))
odom = Odometry(robot)
detector = LandmarkDetector(robot, LANDMARKS)  # LANDMARKS = liste de {"id", "x", "y"}
localizer = Localizer(initial_pose=robot.get_true_pose())  # GPS initial

dt = config.DT
path = planner.plan(start=(2.0, 2.0), goal=(17.0, 12.0))
path = [p for p in path]  # resample si besoin
controller.reset()

for _ in range(max_steps):
    # 1. Localisation
    est = localizer.estimated_pose
    est_pose = (est.x, est.y, est.theta)

    # 2. Verifier si le but est atteint
    if controller.goal_reached((est.x, est.y), path):
        robot.set_velocity(0.0, 0.0)
        break

    # 3. Commande avec POSE ESTIMEE (pas la vraie !)
    v, omega = controller.compute_command(pose=est_pose, path=path)

    # 4. Appliquer et simuler
    robot.set_velocity(v, omega)
    robot.step(dt)

    # 5. Mettre a jour la localisation
    d_l, d_r = odom.read(dt)
    localizer.predict(d_l, d_r)
    localizer.correct(detector.detect())
```

### Etape 5 : Replanification (obstacle detecte)

```python
# Quand un obstacle est detecte (par le lidar ou SafetyManager) :
# 1. Mettre a jour la grille
new_grid = create_test_grid(
    config.WORLD_WIDTH, config.WORLD_HEIGHT,
    config.GRID_RESOLUTION, updated_obstacles,
)
planner = AStarPlanner(new_grid, resolution=config.GRID_RESOLUTION,
                        robot_radius=config.ROBOT_RADIUS)

# 2. Replanifier depuis la pose ESTIMEE
est = localizer.estimated_pose
new_path = planner.plan(start=(est.x, est.y), goal=goal)

if not new_path:
    # Pas de chemin -> le SafetyManager declenche ARRET_SUR
    robot.set_velocity(0.0, 0.0)
else:
    controller.reset()
    path = new_path
    # La boucle continue de suivre le nouveau chemin
```

### Points d'attention pour Dally
- **Le chemin est en coordonnees monde (metres)**, pas en indices de grille
- **`planner.plan()` retourne `[]` si pas de chemin** — toujours verifier avant de suivre
- **`planner.last_plan_time_ms`** donne le temps de calcul en ms (metriques)
- **`controller.reset()`** doit etre appele avant chaque nouveau chemin
- **`controller.goal_reached()`** verifie si arrivee (distance < 0.10m)
- **Pure Pursuit retourne `(v, omega)`** — appliquer via `robot.set_velocity(v, omega)`
- **Dans le systeme integre, utiliser `localizer.estimated_pose`** (pas `robot.get_true_pose()`)
- **Ne pas modifier `planning/` ni `control/`** — si besoin, demander a Koja
- Les **params sont tous dans `config.py`** — ne pas les hardcoder
- **Limiter omega** : `if abs(omega) > 1.5: omega *= 1.5/abs(omega)` (stabilite cinématique)

### Resultats deja prets pour toi
```
results/warehouse_integration/
    warehouse_layout.png          ← Plan de l'entrepot de test
    patrol_astar_localized.png   ← Patrouille A* avec EKF
    patrol_rrt_localized.png    ← Patrouille RRT avec EKF
    dynamic_blockage.png         ← Replanification (path normal orange + path replanifie gris)
    extreme_blockage.png         ← Blocage total + ARRET_SUR
    localization_error_astar.png ← Graphique erreur EKF vs temps
    mission_report.txt           ← Rapport de mission
```

---

## 📋 Guide pour Role 5 (Tino) — Experimentation

> Tino, voici tous les resultats disponibles pour le rapport (section 4).

### Resultats Phase 5 — Planning seul (sans localisation)

```
results/features_planning/
    comparaison.txt              ← Tableau A* vs RRT (texte)
    images/
        patrol_astar.png         ← Trajectoire patrouille A*
        patrol_rrt.png           ← Trajectoire patrouille RRT
        replan_astar.png         ← Trajectoire replanification A*
        replan_rrt.png           ← Trajectoire replanification RRT
    logs/
        patrol_astar.csv         ← Log robot (x, y, theta, v, omega, t)
        patrol_rrt.csv
        replan_astar.csv
        replan_rrt.csv
```

### Resultats Phase 5b — Avec localisation EKF (NOUVEAU)

```
results/warehouse_integration/
    warehouse_layout.png              ← Plan d'entrepot (9 etageres, 4 zones, 9 balises)
    patrol_astar_localized.png       ← A* : path orange + trajectoire bleu + EKF rouge
    patrol_rrt_localized.png        ← RRT : meme chose
    dynamic_blockage.png             ← Path normal (orange) + replanifie (gris) + obstacle rouge
    extreme_blockage.png             ← Path orange + blocage rouge + ARRET_SUR (X noir)
    localization_error_astar.png     ← Erreur position (m) et cap (deg) vs temps
    ekf_comparison_astar.csv         ← CSV true vs estime complet
    patrol_astar_log.csv
    patrol_rrt_log.csv
    mission_report.txt
```

### Tableau comparatif complet

```
Metrique                                    A*          RRT
--- PLANNING SEUL (Phase 5) ---
  Taux de succes                            100%        100%
  Temps mission (s)                         189.3       200.0
  Longueur trajet (m)                        50.13       53.33
  Temps replanification (ms)                238.6       259.9
  Dist. min obstacles (m)                   0.000       0.000

--- AVEC LOCALISATION EKF (Phase 5b) ---
  Taux de succes                           100%        100%
  Waypoints atteints                         4/4         4/4
  Erreur localisation max (m)               0.087       ~0.09
  Incertitude EKF finale (m)                0.026       ~0.03
  Zones couvertes                           100%        100%
  Temps mission (s)                         145.3       ~150
  Replanification dynamique                  OK          —
  Blocage extreme                           ARRET_SUR   —
```

### Comment relancer les experiences

```bash
# --- Phase 5 : Planning seul (avec vrai pose) ---
python -m experiments.run_experiments

# --- Phase 5b : Integration complete avec EKF (NOUVEAU) ---
# Genere toutes les images + CSV + rapport dans results/warehouse_integration/
python tests/test_integration_role2_role3.py

# --- Tests unitaires integration ---
pytest tests/test_integration_role2_role3.py -v -s
```

### Points a tester / verifier pour Tino

1. **Reproductibilite RRT** : RRT est seedé (seed=42), donc deterministe. Tu peux verifier en relancant plusieurs fois avec differentes seeds.

2. **Impact de la localisation** : compare les trajectoires Phase 5 (vraie pose) vs Phase 5b (pose estimee). L'erreur EKF < 0.1m montre que la localisation est fiable.

3. **Temps de replanification** : A* ~239ms, RRT ~260ms (mesures Phase 5). En Phase 5b, la replanification inclut aussi la recréation de la grille.

4. **Precision** : le robot atteint chaque waypoint a < 0.10m meme avec la pose estimee. Verifiable dans les CSV (ekf_comparison_astar.csv).

5. **Graphiques pour le rapport** :
   - `patrol_astar_localized.png` : montre path planifie (orange) vs trajectoire reelle (bleu) — la proximite des deux prouve que le pipeline fonctionne
   - `dynamic_blockage.png` : montre path normal (orange) vs path replanifie (gris) — preuve visuelle de la replanification
   - `localization_error_astar.png` : graphique de l'erreur EKF au fil du temps — reste sous 0.1m
   - `extreme_blockage.png` : montre le X noir ou ARRET_SUR est declenche

6. **Metriques supplementaires** : si tu as besoin de N essais statistiques, ecart-type, etc., demande a Koja.

### Interface pour acceder aux fonctionnalites

```python
# --- Planning seul (Phase 5) ---
from experiments.run_experiments import (
    scenario_patrouille,
    scenario_replanification,
    scenario_comparaison,
)
resultat = scenario_comparaison(verbose=True)

# --- Integration avec EKF (Phase 5b) ---
# Lancer le script directement (genere images + CSV + rapport) :
#   python tests/test_integration_role2_role3.py
#
# Ou importer les fonctions individuelles :
from tests.test_integration_role2_role3 import (
    scenario_patrol_localized,       # -> (metrics, robot, est_history, planned_paths)
    scenario_dynamic_blockage,      # -> (metrics, robot, obstacle, safety_hist, orig_path, replan_path)
    scenario_extreme_blockage,      # -> (metrics, robot, obstacle, safety_hist, orig_path)
)
m, robot, est_hist, planned = scenario_patrol_localized("astar", verbose=True)
print(m["waypoints_reached"])     # 4
print(m["max_localization_error"]) # 0.087
print(m["completion_pct"])         # 100.0
```

---

## Notes de collaboration

### Interface avec les autres roles
- **Role 1 (Malala - Cinematique)** : ✅ Termine — `Robot.set_velocity(v, omega)`, `Robot.get_true_pose()`
- **Role 2 (Kojy - Perception/Localisation)** : ✅ **INTEGRE** — Pipeline EKF + A*/RRT + Pure Pursuit valide (6/6 tests, err < 0.1m)
  - `Localizer.estimated_pose` → `(x, y, theta)` utilisee par Pure Pursuit
  - `Localizer.uncertainty` → `float` utilisee par SafetyManager
  - `LANDMARK_DETECTION_RADIUS = 6.0` dans `config.py` (parametre critique)
- **Role 4 (Dally - Simulation/Integration)** : ⏳ En cours — guide complet ci-dessus. Dally branche mes modules dans la boucle via le pipeline decrit.
- **Role 5 (Tino - Experimentation)** : ⏳ En cours — resultats Phase 5 + Phase 5b prets (voir guide ci-dessus)
- **Equipe Surete (safety/)** : ⏳ Stub — `SafetyManager.check()` peut appeler `planner.plan()` pour la replanification. Mon code est pret.
- **Equipe Securite (security/)** : ✅ **IMPLEMENTE** par Koja — Pipeline complet (Detector + AlertManager + Speaker), 33 tests passent. Voir Phase 7 ci-dessus.

### Responsabilite sur la couverture de zone

> **Question** : Si le robot ne couvre que 30% des zones a cause de blocages, est-ce la faute du Role 3 ?

| Situation | Responsable | Pourquoi |
|-----------|-------------|----------|
| Contournement existe mais le planner ne le trouve pas | **Role 3 (Koja)** | Le planner aurait du trouver le chemin alternatif |
| Le robot couvre 100% car les chemins alternatifs existent | **Role 3 (Koja)** | Le planner a bien fait son travail |
| Aucun contournement physique n'existe (couloir completement ferme) | **Pas Role 3** | Le SafetyManager declenche ARRET_SUR — cas de force majeure |
| Le robot rate un WP car l'EKF a diverge | **Pas Role 3** | C'est Role 2 (localisation) |

**En resume** : Le Role 3 est responsable de trouver des chemins viables et des alternatives. Si un contournement existe physiquement et que le planner ne le trouve pas, c'est une limitation du Role 3. Si aucun contournement n'existe, le SafetyManager gere.

### Deux layouts d'entrepot dans le projet ⚠️ IMPORTANT pour Dally et Tino

| | **Koja (test_integration_role2_role3.py)** | **Dally (entrepot_patrouille.py)** |
|---|---|---|
| **Role** | Test d'integration Role 2↔3 | Integration finale du systeme |
| **Racks** | 9 gros blocs (3.5m × 1.5m), 3 rangées de 3 | 12 racks fins (2m × 0.4m), 3 colonnes de 4 |
| **Waypoints** | 4 (dans les 2 allées principales) | 8 (circuit complet quai → allées → fond → retour) |
| **Balises EKF** | 9 (pour le recalage) | ✅ **9 balises ajoutees** |
| **Zones de mission** | 4 zones (metrique couverture) | ❌ Aucune |
| **Layout utilise pour** | Tests unitaires + visuels dans `results/warehouse_integration/` | Integration finale par Dally |

**Pour Dally** : Mes modules A*, RRT, Pure Pursuit sont 100% compatibles avec ton layout.
Ils prennent n'importe quelle liste d'obstacles en parametre. Il suffit de faire :
```python
from entrepot_patrouille import WAREHOUSE_OBSTACLES, WAREHOUSE_WAYPOINTS
grid = create_test_grid(20.0, 15.0, 0.1, WAREHOUSE_OBSTACLES)
path = planner.plan(start=(1.5, 1.0), goal=WAREHOUSE_WAYPOINTS[1])
```

**Pour Kojy** : ✅ **FAIT** — `WAREHOUSE_LANDMARKS` (9 balises) a ete ajoute a `entrepot_patrouille.py`.
Dally n'a plus qu'a l'importer :
```python
from entrepot_patrouille import WAREHOUSE_LANDMARKS
detector = LandmarkDetector(robot, WAREHOUSE_LANDMARKS)
```

### Convention Git
- Branche : `feature/planning`
- Messages : `feat(planning): ...` / `feat(control): ...` / `feat(integration): ...` / `fix(...): ...`
- Ne jamais modifier `robot/`, `simulation/`, `gui/` sans concertation
- `config.py` est partage -> previens le groupe des ajouts
- **`entrepot_patrouille.py` appartient a Dally** — ne pas le modifier sans son accord

### Fichiers modifies par Role 3
| Fichier | Role | Description |
|---------|------|-------------|
| `planning/astar.py` | Nouveau | Planificateur A* avec inflation et lissage |
| `planning/rrt.py` | Nouveau | Planificateur RRT avec lissage |
| `control/pure_pursuit.py` | Nouveau | Controleur Pure Pursuit |
| `tests/test_planning.py` | Nouveau | 32 tests (A*, RRT, PP, integration) |
| `tests/test_integration_role2_role3.py` | Nouveau | 6 tests + 4 scenarios visuels avec EKF |
| `localization/localization.py` | Reecrit | EKF 3×3 (Jacobien, Joseph form) |
| `experiments/run_experiments.py` | Modifie | 3 scenarios + comparaison + visualisation |
| `config.py` | Modifie | Params planning + `LANDMARK_DETECTION_RADIUS = 6.0` |
| `requirements.txt` | Modifie | Decommenté numpy |
| `security/intrusion_detector.py` | Nouveau | IntrusionAlert + IntrusionDetector (FOV, filtres, dedup) |
| `security/alert_manager.py` | Nouveau | AlertLevel + AlertEvent + AlertManager (4 niveaux) |
| `security/speaker.py` | Nouveau | AlarmPattern + Speaker (NONE/SLOW/FAST) |
| `security/__init__.py` | Nouveau | Imports publics du module |
| `tests/test_intrusion_detector.py` | Nouveau | 12 unit + 1 integration |
| `tests/test_alert_manager.py` | Nouveau | 11 unit + 1 integration |
| `tests/test_speaker.py` | Nouveau | 6 unit + 1 integration pipeline complet |
| `ROLE3_WORKFLOW.md` | Nouveau | Ce fichier |


---

## Interface publique du module Securite (Role 1)

### IntrusionDetector
```python
from security.intrusion_detector import IntrusionAlert, IntrusionDetector

detector = IntrusionDetector(
    known_obstacles=[(4.0, 3.0), (8.0, 3.0), (6.0, 7.0)],  # centres des racks
    tolerance_zone=1.0,     # m — rayon de tolerance autour de chaque rack
    dedup_radius=0.5,       # m — rayon de deduplication
    cooldown=2.0,           # s — delai entre deux alertes pour le meme intrus
)
alerts: list[IntrusionAlert] = detector.check(
    observations=[(7.0, 5.0), (8.0, 5.5)],  # cibles detectees par Camera
    robot_pose=(2.0, 5.0, 0.0),              # (x, y, theta) du robot
    camera_fov_deg=90.0,                     # champ de vision de la camera
)
# alerts[0].position -> (7.0, 5.0)
# alerts[0].distance -> 5.0 (en metres)
# alerts[0].timestamp -> time.time()

# Bridge vers Role 3 (Koja) — obstacles dynamiques pour replanification
intruder_positions = detector.get_intruder_positions()
# -> [(7.0, 5.0), (8.0, 5.5)]

# Bridge vers Role 4 (Dally) — arret d'urgence si intrus trop proche
is_close = detector.is_close_intrusion(danger_distance=2.0)
# -> True / False

# Reset (nouvelle mission)
detector.reset()
```

### AlertManager
```python
from security.alert_manager import AlertLevel, AlertEvent, AlertManager

am = AlertManager(
    warning_dist=4.0,     # m — seuil WARNING
    danger_dist=2.0,      # m — seuil DANGER
    resolution_delay=1.0, # s — delai avant retour a NOMINAL
)
event: AlertEvent = am.update(alerts, robot_pose=(2.0, 5.0, 0.0))
# event.level -> AlertLevel.WARNING
# event.intruder_position -> (7.0, 5.0)
# event.distance -> 3.0
# event.message -> "⚠ Intrusion a 3.0m en (7.0, 5.0)"

# Bridge vers Role 4 (Dally) — SafetyManager.check()
confirmed = am.get_intrusion_confirmed()  # True si WARNING ou DANGER
is_danger = am.is_danger()                 # True si DANGER uniquement

# Bridge vers Speaker
should_alarm = am.should_alarm()           # True si WARNING ou DANGER

# Historique
history = am.get_history()  # liste de tous les AlertEvent
am.reset()
```

### Speaker
```python
from security.speaker import AlarmPattern, Speaker

speaker = Speaker(activation_threshold=2)  # 2 triggers consecutifs pour activer
pattern = speaker.update(should_alarm=True, level_name="WARNING")
# -> AlarmPattern.SLOW (apres 2 appels)

pattern = speaker.update(should_alarm=True, level_name="DANGER")
# -> AlarmPattern.FAST

speaker.update(should_alarm=False)  # arret de l'alarme

# Stats
stats = speaker.get_stats()
# {"trigger_count": 1, "total_alarm_time": 2.3, "current_pattern": "none", "is_on": False}

speaker.reset()
```

---

## 📋 Guide pour Dally — Integration Securite

> Dally, voici comment integrer le module securite dans ta boucle de simulation.

### Pipeline securite dans la boucle principale

```python
from security import IntrusionDetector, AlertManager, Speaker

# --- Setup ---
# Obstacles connus (racks de ton entrepot)
from entrepot_patrouille import WAREHOUSE_OBSTACLES
rack_centers = [(obs["x"] + obs["w"]/2, obs["y"] + obs["h"]/2)
                 for obs in WAREHOUSE_OBSTACLES]

detector = IntrusionDetector(
    known_obstacles=rack_centers,
    tolerance_zone=1.0,
    cooldown=2.0,
)
am = AlertManager(warning_dist=4.0, danger_dist=2.0, resolution_delay=1.0)
speaker = Speaker(activation_threshold=2)

# --- Dans la boucle de simulation ---
for step in range(max_steps):
    # 1. Localisation (deja existant)
    est = localizer.estimated_pose
    est_pose = (est.x, est.y, est.theta)

    # 2. Securite : detection d'intrusion
    #    camera.observe() retourne les cibles visibles
    observations = camera.observe(all_targets)  # [(x, y), ...]
    alerts = detector.check(observations, robot.get_true_pose(), camera_fov_deg=90.0)

    # 3. Mise a jour du niveau d'alerte
    event = am.update(alerts, est_pose)

    # 4. Declencher l'alarme si necessaire
    speaker.update(should_alarm=am.should_alarm(), level_name=event.level.name)

    # 5. Arret d'urgence si intrusion confirmee
    if am.is_danger():
        robot.set_velocity(0.0, 0.0)
        continue  # ou break selon la strategie

    # 6. Replanification si intrus detecte (bridge vers Role 3)
    intruder_positions = detector.get_intruder_positions()
    if intruder_positions and not am.is_danger():
        # Ajouter les intrus comme obstacles dynamiques et replanifier
        # (Koja's A*/RRT peuvent utiliser ces positions)
        pass  # ta logique de replanification

    # 7. Commande (deja existant)
    v, omega = controller.compute_command(pose=est_pose, path=path)
    robot.set_velocity(v, omega)
    robot.step(dt)
    d_l, d_r = odom.read(dt)
    localizer.predict(d_l, d_r)
    localizer.correct(detector_landmark.detect())
```

### Points d'attention
- **`robot.get_true_pose()`** est utilise pour le FOV filtering (le detecteur a besoin de la vraie pose pour le champ de vision)
- **`am.update()`** prend la pose estimee (comme Pure Pursuit)
- **`am.is_danger()`** doit etre verifie AVANT le calcul de commande
- **`speaker.update()`** doit etre appele a chaque step, meme si pas d'alarme
- **`speaker` a un `activation_threshold=2`** par defaut : 2 appels consecutifs avec `should_alarm=True` pour declencher l'alarme (anti-faux-positifs)
- **Le niveau ne descend pas instantanement** : attend `resolution_delay` (1.0s par defaut)

### Tests
```bash
# Lancer tous les tests securite
pytest tests/test_intrusion_detector.py tests/test_alert_manager.py tests/test_speaker.py -v -s

# Resultat attendu : 29 unit + 4 integration = 33 pass
```

---

## 📋 Guide pour Tino — Securite pour le rapport

> Tino, voici les metriques du module securite pour la section du rapport.

### Metriques du module securite

```
Module                    Tests    Status
IntrusionDetector         13 pass  FOV + filtres + dedup + cooldown + bridge
AlertManager              12 pass  4 niveaux + resolution_delay + bridges
Speaker                    7 pass  3 patterns + anti-FAUX_POSITIFS + stats
Pipeline integration       1 pass  Camera -> Detector -> AM -> Speaker
---                                ---
TOTAL                     33 pass  0 fail
```

### Niveaux d'alerte (schema pour le rapport)

| Niveau | Condition | Action robot | Alarme |
|--------|-----------|--------------|--------|
| NOMINAL | Aucune intrusion | Patrouille normale | Silence |
| INFO | Intrus > 4.0m | Patrouille continue | Silence |
| WARNING | 2.0m < Intrus <= 4.0m | Ralentir + logger | Bip lent |
| DANGER | Intrus <= 2.0m | Arret d'urgence | Bip rapide |

### Architecture pour le diagramme du rapport

```
Camera.observe(targets)
    |
    v
IntrusionDetector.check()       [Filtre FOV + Obstacles + Dedup]
    |
    v
[IntrusionAlert]                [position, distance, timestamp]
    |
    v
AlertManager.update()          [Niveaux: NOMINAL/INFO/WARNING/DANGER]
    |           |           |
    |           +-----------+   |
    v                       v   v
SafetyManager          Speaker.update()
(Dally - arret)         [NONE/SLOW/FAST]
    |
    v
Replanification
(A*/RRT avec
 obstacles dynamiques)
