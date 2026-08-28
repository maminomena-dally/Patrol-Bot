# Robot de patrouille de sécurité — Projet mini-projet Robotique Mobile

Robot mobile différentiel autonome (patrouille + sécurité), conçu et simulé
en 2D, développé en équipe. Ce dépôt est structuré pour que chaque binôme
puisse travailler dans son propre module sans toucher au reste du code.

**État actuel : tous les modules sont terminés et testés (120 tests).**
Les six briques (cinématique, perception, localisation, planification,
commande, sécurité, sûreté) sont réellement assemblées dans une seule
boucle de simulation — voir `experiments/integration_finale.py` et
`SIMULATION_INTEGRATION.md`.

## 1. Démarrage rapide

```bash
python3 -m venv venv
source venv/bin/activate        # Windows : venv\Scripts\activate
pip install -r requirements.txt

python main.py                                # démo cinématique + aperçu du système complet
python -m experiments.integration_finale       # démo complète (A* + RRT, graphiques, logs)
python -m pytest tests/ -q                      # suite de tests complète (120 tests)
```

Voir `INSTALLATION.md` pour le détail pas à pas.

## 2. Architecture du projet

```
robot_patrouille/
├── main.py                 # démo cinématique + aperçu du système complet
├── config.py                # TOUS les paramètres physiques et de simulation
│
├── robot/                   # ✅ TERMINÉ — Système et Cinématique
│   ├── kinematics.py         # équations pures (v, ω, intégration de pose)
│   └── robot.py               # classe Robot : état, commande, limites, logs
│
├── simulation/               # ✅ TERMINÉ — boucle temporelle générique
│   └── simulator.py           # fait avancer le robot pas à pas (callbacks on_perceive/
│                                 on_localize/on_detect/on_plan/on_safety + stop_fn)
│
├── gui/                       # ✅ TERMINÉ — interfaces graphiques 2D
│   ├── robot_view.py            # rendu Matplotlib du robot (réutilisable)
│   ├── app.py                     # interface interactive Tkinter + Matplotlib (pilotage manuel)
│   ├── safety_app.py                # interface interactive de test du SafetyManager
│   └── replay.py                    # rejeu graphique d'un log CSV exporté
│
├── sensors/                  # ✅ TERMINÉ — Perception
│   ├── odometry.py            # encodeurs de roues simulés (bruités)
│   ├── lidar.py                 # capteur de distance 360°, scan par intersection rayon/obstacle
│   ├── landmarks.py            # balises pour recalage de position (distance + angle, bruités)
│   └── cameras.py               # caméra frontale + caméra de surveillance (FOV, portée)
│
├── localization/             # ✅ TERMINÉ — Localisation
│   └── localization.py         # EKF 3×3 (Jacobiens F/H, forme de Joseph) : fusion odométrie+balises
│
├── planning/                  # ✅ TERMINÉ — Planification
│   ├── astar.py                 # A* sur grille d'occupation (inflation obstacles, 8-connexe)
│   └── rrt.py                     # RRT / replanification rapide
│
├── control/                   # ✅ TERMINÉ — Contrôle
│   └── pure_pursuit.py           # suivi de trajectoire (Pure Pursuit)
│
├── security/                  # ✅ TERMINÉ — Sécurité (détection d'intrusion)
│   ├── intrusion_detector.py    # caméras -> détection d'intrus, filtrage obstacles connus
│   ├── alert_manager.py           # niveaux NOMINAL/INFO/WARNING/DANGER
│   └── speaker.py                   # alarme sonore simulée (journalisée)
│
├── safety/                     # ✅ TERMINÉ — Sûreté
│   └── safety_manager.py          # machine à états NOMINAL/ALERTE/ARRET_SUR, arrêt sûr, journal
│
├── experiments/                # scénarios de test reproductibles
│   ├── run_experiments.py         # patrouille + replanification (vérité terrain)
│   ├── campagne_essais.py           # campagne statistique (10 essais nominaux + cas limites)
│   ├── campagne_localisation.py      # erreur de localisation + perte de balise
│   ├── integration_localization.py    # patrouille avec pose ESTIMÉE (EKF), sans sûreté
│   ├── demo_safety.py                   # démo visuelle du SafetyManager
│   ├── entrepot_patrouille.py             # carte d'entrepôt (obstacles, waypoints, balises)
│   └── integration_finale.py               # ✅ boucle complète (6 modules) via simulation.Simulator
│
├── tests/                        # tests unitaires (pytest / unittest) — 120 tests
│
├── logs/                          # historiques d'état exportés (CSV)
├── results/                       # résultats des scénarios/expériences (CSV, PNG, résumés texte)
├── requirements.txt
├── .gitignore
├── INSTALLATION.md
├── README.md                        # ce fichier
├── PERCEPTION_LOCALISATION.md         # doc détaillée : rôle Perception/Localisation
├── SIMULATION_INTEGRATION.md            # doc détaillée : boucle d'intégration finale
├── ROLE3_WORKFLOW.md                      # suivi de travail : Planification/Commande/Sécurité
└── TINO_WORKFLOW.md                        # suivi de travail : Sûreté/Expérimentation
```

## 3. Règle d'or : comment s'intégrer au module Système/Cinématique

Tous les autres modules pilotent/lisent le robot **uniquement** via
l'interface publique de `robot.robot.Robot`, jamais en modifiant `robot.pose`
directement :

| Besoin | Méthode à utiliser |
|---|---|
| Commander le robot (v, ω) | `robot.set_velocity(v, omega)` |
| Commander via les roues | `robot.set_wheel_velocity(vL, vR)` |
| Lire la vérité terrain (pour un capteur simulé) | `robot.get_true_pose()` |
| Lire l'état complet (pour logs / UI) | `robot.get_state()` |
| Récupérer la géométrie (pour collisions) | `robot.get_footprint()` |
| Déclencher un arrêt sûr | `robot.emergency_stop()` / `robot.resume()` |
| Faire avancer le temps d'un pas | `robot.step(dt)` (via `Simulator`) |

⚠️ **Important** : `localization/localization.py` ne lit jamais
`get_true_pose()` en dehors de ses propres tests de validation — il estime
sa propre pose à partir des capteurs (`sensors/odometry.py`,
`sensors/landmarks.py`), pour que la boucle perception → localisation →
planification → commande (slide 10 du cours) garde son sens.

## 4. Interfaces graphiques 2D

```bash
python -m gui.app            # pilotage manuel interactif
python -m gui.safety_app     # test interactif du SafetyManager
```

`gui/app.py` ouvre une fenêtre avec :
- une vue 2D du robot (position, orientation, trace de trajectoire),
- deux curseurs pour piloter v et ω (ou les flèches du clavier),
- un bouton **ARRÊT D'URGENCE** / **Reprendre**,
- un panneau d'état en temps réel (x, y, θ, v, ω, arrêt sûr),
- un export du log en un clic.

Pour rejouer un log déjà enregistré (traçabilité, section 19) :

```bash
python -m gui.replay logs/robot_state_log.csv
python -m gui.replay results/features_integration_finale/logs/patrol_astar.csv --speed 8
```

`gui/robot_view.py` contient uniquement la logique de dessin (sans
Tkinter), réutilisée par `app.py`, `safety_app.py` et `replay.py`.

## 5. La boucle complète : `simulation.Simulator`

`simulation/Simulator` expose des callbacks pour insérer chaque module
sans modifier son code (voir `simulation/simulator.py`). C'est réellement
utilisé dans `experiments/integration_finale.py` :

```python
from robot.robot import Robot
from simulation.simulator import Simulator

robot = Robot()
sim = Simulator(robot)

sim.on_perceive = loop.on_perceive   # Odometry.read() + LidarSensor.min_distance()
sim.on_localize  = loop.on_localize  # Localizer.predict() + LandmarkDetector.detect() + correct()
sim.on_detect    = loop.on_detect    # IntrusionDetector -> AlertManager -> Speaker
sim.on_plan      = loop.on_plan      # gestion des waypoints + AStarPlanner/RRTPlanner.plan()
sim.on_safety    = loop.on_safety    # SafetyManager.check(..., intrusion_confirmed=..., intrusion_danger=...)

sim.run(duration=400.0, command_fn=loop.command_fn, stop_fn=loop.stop_fn)
```

`stop_fn` (ajouté à `Simulator.run()`, rétro-compatible) arrête la
simulation dès que la mission réussit ou qu'un `ARRET_SUR` est déclenché,
plutôt que de consommer toute la `duration` demandée. Voir
`SIMULATION_INTEGRATION.md` pour le détail complet de la boucle, les
résultats réels obtenus et les limites connues.

## 6. Workflow Git recommandé pour l'équipe

- Une branche par binôme/module : `feature/perception`, `feature/planning`, etc.
- Ne pas modifier `robot/`, `simulation/`, `config.py` sans concertation :
  c'est le socle partagé. Si une évolution est nécessaire (ex : nouveau
  paramètre dans `config.py`), en discuter avant de merger.
- Chaque module garde ses tests dans `tests/test_<module>.py`.
- Merger régulièrement vers `main` pour éviter les conflits tardifs.

## 7. Références

Voir la section "Références principales" du support de cours
(*Robotique_mobile_M2_2h_Dr_Randria.pdf*) et le cahier des charges complet
(*Systeme_Cinematique_Robot_Patrouille_Securite_Complet.pdf*).
