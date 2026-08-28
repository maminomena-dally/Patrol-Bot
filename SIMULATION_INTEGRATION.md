# Simulation / Intégration — Documentation du module

Rôle 4 (Dally). Ce document décrit l'assemblage final des six modules du
projet en une seule boucle de simulation, sur la carte d'entrepôt. Tout ce
qui suit est basé sur le code réellement écrit et des exécutions réelles
(tests unitaires + campagnes) — rien n'est inventé.

---

## 1. Vue d'ensemble du rôle

Avant ce travail, chaque binôme avait développé et testé son module
séparément, avec plusieurs scripts d'intégration partielle dans
`experiments/` (planification+commande seuls, ou +localisation, ou
+sûreté — jamais les six ensemble) et un module de sécurité
(`security/`) jamais testé qu'avec un faux robot. Le rôle
Simulation/Intégration consiste à :

1. Faire l'inventaire exact de ce qui existe et de ce qui manque (pas de
   suppositions sur les noms de fichiers ou les interfaces).
2. Assembler réellement les six modules dans une seule boucle qui utilise
   le vrai `robot.robot.Robot`.
3. Corriger les incohérences trouvées entre les modules au moment de les
   brancher ensemble (c'est le travail typique d'intégration).

## 2. Constat de départ (avant les corrections)

Trois problèmes concrets ont été trouvés en essayant de suivre le guide
d'intégration laissé par l'équipe (`ROLE3_WORKFLOW.md`, section "Guide
pour Dally") :

| Problème | Où | Nature |
|---|---|---|
| `config.py` contenait des commandes shell (`git log ...`) collées par erreur dans le fichier Python | `config.py` | Cassait l'import de **tout** le projet (`SyntaxError`) |
| Une f-string avec un backslash dans son expression | `tests/test_integration_role2_role3.py:1175` | Syntaxe invalide avant Python 3.12 ; le fichier ne se chargeait pas du tout sur le Python 3.11 du `venv` du projet |
| `IntrusionDetector.check()` lisait `self.robot.x` / `.y` / `.theta` | `security/intrusion_detector.py` | Ces attributs plats n'existent pas sur le vrai `Robot` (qui a `robot.pose.x` et `robot.get_true_pose()`) → le module n'avait jamais tourné qu'avec un `MockRobot` factice dans ses propres tests |

Un quatrième problème, plus subtil, concernait le **guide d'intégration
sécurité** lui-même (`ROLE3_WORKFLOW.md`, lignes 943-1017) : ses exemples
de code (`AlertManager(warning_dist=...)`, `detector.check(observations,
pose, camera_fov_deg=...)`, `Speaker(activation_threshold=2)`,
`from security import ...`) ne correspondent à **aucune** des signatures
réellement implémentées dans `security/*.py`. Le suivre tel quel aurait
fait planter le code immédiatement. `experiments/integration_finale.py`
utilise les vraies signatures, vérifiées en lisant le code source.

## 3. Corrections apportées

- `security/intrusion_detector.py` : `self.robot.x/.y/.theta` remplacés
  par `self.robot.get_true_pose()` dans `check()` — cohérent avec
  `sensors/cameras.py`, qui faisait déjà ça correctement.
- `tests/test_integration_role2_role3.py` : la f-string fautive
  remplacée par une variable calculée avant (`statut_blocage = "..." if
  ... else "..."`), équivalent fonctionnel, compatible Python 3.11+.
- `tests/test_perception_localization.py::test_predict_augmente_incertitude` :
  le test supposait `uncertainty == 0.0` à l'initialisation, ce qui était
  vrai pour l'ancien filtre pondéré simple mais plus pour l'EKF actuel
  (qui démarre avec une covariance non nulle, `P = diag([1e-4, ...])`).
  Corrigé pour comparer l'incertitude avant/après `predict()` plutôt que
  supposer un zéro initial.
- `config.py` : corrigé indépendamment (par un autre membre de l'équipe)
  pendant ce travail — les lignes de commandes shell ont disparu du
  fichier avant que cette correction soit nécessaire de mon côté.

Après ces corrections : **118 tests passent** (`pytest tests/ -q`), zéro
régression.

## 4. La boucle d'intégration finale

### 4.1 La carte

`experiments/entrepot_patrouille.py` — définit :
- `WAREHOUSE_OBSTACLES` : 19 rectangles (4 murs périmétriques + 1
  séparation quai/entrepôt + 12 racks en 3 rangées + 2 cloisons de zone
  réservée)
- `WAREHOUSE_WAYPOINTS` : 8 points de patrouille (quai → allées → fond →
  retour)
- `WAREHOUSE_LANDMARKS` : 9 balises, placées le long des murs et au
  centre des allées pour couvrir le circuit de patrouille

### 4.2 Ordre des opérations — via `simulation.Simulator`

Première version (voir §4.4) : boucle `for` écrite à la main. Version
actuelle : `experiments/integration_finale.py` utilise réellement
`simulation.Simulator` et ses callbacks `on_perceive` / `on_localize` /
`on_detect` / `on_plan` / `on_safety`, comme prévu depuis le départ par
`README.md` et `simulation/simulator.py` mais jamais branché par aucun
rôle jusqu'ici (chaque script précédent écrivait sa propre boucle). Une
classe `_IntegrationLoop` regroupe l'état partagé entre les callbacks (un
callback ne reçoit que `(robot, t)`, donc tout ce qui doit survivre d'un
pas à l'autre — l'EKF, le chemin courant, les compteurs — vit dans cette
classe plutôt que dans des variables locales d'une fonction de boucle qui
n'existe plus).

`Simulator.run()` fixe l'ordre d'appel à chaque pas `dt` :

```
on_perceive  -> Odometry.read(dt)                    -> d_left, d_right (bruités)
                LidarSensor.min_distance()            -> distance au plus proche obstacle
on_localize  -> Localizer.predict(d_left, d_right)    -> pose estimée (EKF, avance)
                LandmarkDetector.detect()             -> balises visibles (bruitées)
                Localizer.correct(detections)         -> pose estimée (EKF, recale)
on_detect    -> IntrusionDetector.check(cibles, t)    -> intrusion confirmée ? + alertes
                AlertManager.update(alertes, t)       -> niveau NOMINAL/INFO/WARNING/DANGER
                Speaker.update(should_alarm, t)
on_plan      -> gestion des waypoints
                AStarPlanner/RRTPlanner.plan(start=pose ESTIMÉE, goal=...)
                -> replanifie si : pas de chemin courant, but atteint, ou
                   périodiquement (tous les 100 pas, plus fréquent si
                   incertitude > 0.2m)
on_safety    -> SafetyManager.check(robot,
                    localization_uncertainty=localizer.uncertainty,
                    obstacle_distance=...,
                    path_found=...,
                    intrusion_confirmed=alert_manager.get_intrusion_confirmed())
                -> si ARRET_SUR : robot.emergency_stop() (interne à SafetyManager)
command_fn   -> PurePursuitController.compute_command(pose=pose ESTIMÉE, path=...)
                Robot.set_velocity(v, omega)
             -> Robot.step(dt)
stop_fn      -> vérifié après le pas : arrête la simulation dès que la
                mission réussit ou qu'un ARRET_SUR est déclenché
```

Cet ordre (imposé par `Simulator.run()`, pas choisi au cas par cas comme
dans les scripts précédents) place la planification **avant** la
vérification de sûreté — donc `path_found` reflète toujours la tentative
de planification *du pas courant*, jamais une valeur restée d'un pas
précédent.

### 4.3 Ajout rétro-compatible à `simulation/simulator.py`

`Simulator.run()` ne s'arrêtait jamais avant d'avoir consommé toute la
`duration` demandée — un souci mineur pour une démo de 5s, mais
problématique ici (`duration` doit couvrir le pire cas, ~400s, alors
qu'une patrouille réussie se termine vers 170s ; sans arrêt anticipé, la
simulation continuerait inutilement robot immobile pendant ~230s). Un
paramètre optionnel `stop_fn(robot, t) -> bool` a été ajouté, vérifié
après chaque pas ; `None` par défaut, donc **aucun appelant existant n'est
affecté** (`main.py`, `tests/test_planning.py`,
`experiments/run_experiments.py` n'utilisent pas ce paramètre).

### 4.4 Ancienne version (pour référence)

Une première version de `integration_finale.py` (avant ce refactoring)
utilisait une boucle `for` manuelle, comme tous les autres scripts
`experiments/`. Elle a été remplacée par la version basée sur `Simulator`
ci-dessus, à la demande explicite de suivre l'architecture prévue par le
projet plutôt que de perpétuer la convention "boucle à la main" adoptée
par tous les rôles précédents faute d'exemple fonctionnel.

**Point clé, comme dans `integration_localization.py`** : la
planification et la commande utilisent `localizer.estimated_pose`, jamais
`robot.get_true_pose()` — sauf `IntrusionDetector`, qui a légitimement le
droit de lire la vérité terrain (c'est un capteur simulé, comme les
caméras dont il se sert en interne).

### 4.3 Intrusions simulées

Le projet étant hors ligne, les intrus sont des positions `(x, y)`
injectées à des instants fixés (`INTRUDER_SCHEDULE` dans
`integration_finale.py`) : un premier intrus apparaît à t=15s entre deux
rangées de racks, un second à t=70s près de la zone de stockage réservée.
`IntrusionDetector` crée en interne ses deux caméras (frontale 90°,
surveillance 180°/120°, portée 5m) et ne "voit" un intrus que s'il est
dans leur champ de vision réel à ce pas de temps — donc la détection
dépend de l'orientation du robot au moment où l'intrus apparaît, pas
d'une distance seule.

## 5. Guide d'utilisation / test

```bash
# Suite de tests complète (118 tests)
venv/Scripts/python.exe -m pytest tests/ -q

# Boucle d'intégration finale (A* puis RRT sur la carte entrepôt, via Simulator)
venv/Scripts/python.exe -m experiments.integration_finale
```

> Le `venv` du projet contient `numpy` (nécessaire à `planning/astar.py`)
> et, après ce travail, `pytest` — le Python système ne les a pas.

**Résultats réels obtenus** (seed fixe, reproductibles à l'identique d'un
run à l'autre) :

| | A* | RRT |
|---|---|---|
| Succès | **Non** — `ARRET_SUR` | Oui |
| Waypoints atteints | 4/7 | 7/7 |
| Temps de mission | 85.1 s | 172.4 s |
| Erreur de localisation max | 0.073 m | 0.203 m |
| Pas avec intrusion confirmée | 221 | 642 |
| Niveau d'alerte max atteint | WARNING | WARNING |
| Alarmes déclenchées (niveau DANGER) | 0 | 0 |
| État de sûreté final | **ARRET_SUR** | NOMINAL |

Résultats sauvegardés dans
`results/features_integration_finale/resume_integration_finale.txt`.

**Ce résultat A* a changé après le passage à `Simulator`** (voir section
7) : avant ce refactoring, avec la même graine aléatoire, A* réussissait
7/7. `Simulator.run()` appelle les callbacks dans un ordre fixe
(perceive→localize→detect→plan→safety), légèrement différent de l'ancien
code écrit à la main — cela ne change pas la séquence de bruit tirée
(seule `Odometry`/`LandmarkDetector` consomment `random`, toujours dans le
même ordre), mais décale de quelques pas *quand* la replanification
périodique est déclenchée. Résultat : au pas où A* replanifie vers WP6,
la pose estimée (légèrement différente à cet instant précis) tombe cette
fois dans la marge de sécurité gonflée autour du mur de la zone réservée
→ `planner.plan()` échoue transitoirement → `SafetyManager` déclenche
`ARRET_SUR` à raison. C'est reproductible avec ce code exact, mais
sensible au moindre décalage de timing — voir la limite correspondante en
section 7.

## 6. Paramètres de réglage

En plus de ceux déjà documentés par les autres rôles (`config.py`) :

| Paramètre | Où | Effet |
|---|---|---|
| `INTRUDER_SCHEDULE` | `integration_finale.py` | Position et instant d'apparition des intrus simulés — change quand/si `IntrusionDetector` les voit |
| `replan_every_n_steps` | `_run_patrol()` | Fréquence de replanification périodique (défaut 100 pas = 5s) ; réduite automatiquement si l'incertitude EKF dépasse 0.2m |
| `random.seed(42/43)` | `_run_patrol()` | Fixe le bruit d'odométrie/balises pour un résultat reproductible d'un run à l'autre — **sans ce seed, le scénario est occasionnellement différent** (voir limite ci-dessous) |
| `tentatives_max_replanification=3` | `SafetyManager(...)` | Nombre d'échecs de planification consécutifs tolérés avant `ARRET_SUR` |

## 7. Limites connues

- **Comportement sensible au timing exact de la replanification** :
  l'estimation EKF, bruitée, peut tomber suffisamment près d'un obstacle
  gonflé (inflation de sécurité du rayon du robot) au moment précis d'une
  replanification pour que `planner.plan()` échoue transitoirement, ce
  qui déclenche à raison un `ARRET_SUR`. **Constaté concrètement** :
  A* réussit 7/7 dans la version "boucle manuelle" (§4.4) et échoue à
  WP6/8 dans la version `Simulator` (§4.2, résultat documenté en
  section 5), avec la **même graine aléatoire** — l'ordre d'appel fixé
  par `Simulator.run()` décale de quelques pas le moment exact des
  replanifications périodiques, ce qui suffit à faire tomber (ou pas)
  l'estimation dans la marge gonflée au bon moment. **Ce n'est pas un
  bug** : c'est le système de sûreté qui réagit correctement à une
  situation où le robot ne peut momentanément plus faire confiance à sa
  position ; c'est documenté, pas corrigé, pour ne pas masquer ce
  comportement réel — et c'est un exemple concret, pas seulement
  théorique, à présenter en soutenance.
- **`SafetyManager.obstacle_distance` n'est jamais comparé à
  `config.OBSTACLE_SAFE_DISTANCE`** dans la logique de décision actuelle
  de `safety/safety_manager.py` — seule l'absence totale de mesure
  (`None`) déclenche un arrêt par prudence. Un obstacle proche mais
  mesuré ne déclenche donc aucune réaction de sûreté à lui seul
  aujourd'hui. Limite du module de sûreté, pas introduite par ce travail.
- **`AlertManager.get_intrusion_confirmed()` était conçu pour alimenter
  `SafetyManager.check(intrusion_confirmed=...)`** (documenté des deux
  côtés) mais n'était branché nulle part avant ce fichier — c'est
  maintenant fait dans `integration_finale.py`, mais uniquement là.
- **Deux cartes d'entrepôt coexistent** dans le dépôt :
  `experiments/entrepot_patrouille.py` (utilisée ici) et une carte
  différente intégrée dans `tests/test_integration_role2_role3.py` (9
  racks, 4 waypoints, utilisée pour les visuels de Koja dans
  `results/warehouse_integration/`). Elles ne sont pas interchangeables
  sans adapter les balises/waypoints.
- **`IntrusionDetector` ne filtre pas les racks inflatés** de la même
  marge que le planificateur (`obstacle_tolerance` par défaut 0.5m contre
  l'inflation `robot_radius` d'A\*) — un intrus juste au bord d'un rack
  peut être classé différemment par les deux modules.

