# Robot de patrouille de sécurité — Projet mini-projet Robotique Mobile

Robot mobile différentiel autonome (patrouille + sécurité), conçu et simulé
en 2D, développé en équipe. Ce dépôt est structuré pour que chaque binôme
puisse travailler dans son propre module sans toucher au reste du code.

**État actuel : le module "Système et Cinématique" est terminé et testé.**
Les autres modules sont des squelettes (stubs) prêts à être complétés.

## 1. Démarrage rapide

```bash
python3 -m venv venv
source venv/bin/activate        # Windows : venv\Scripts\activate
pip install -r requirements.txt

python main.py                  # démo du module Système/Cinématique
python -m unittest discover -s tests -v   # tests unitaires
```

Voir `INSTALLATION.md` pour le détail pas à pas.

## 2. Architecture du projet

```
robot_patrouille/
├── main.py                 # démo exécutable du module Système/Cinématique
├── config.py                # TOUS les paramètres physiques et de simulation
│
├── robot/                   # ✅ TERMINÉ — Système et Cinématique
│   ├── kinematics.py         # équations pures (v, ω, intégration de pose)
│   └── robot.py               # classe Robot : état, commande, limites, logs
│
├── simulation/               # ✅ TERMINÉ — boucle temporelle générique
│   └── simulator.py           # fait avancer le robot pas à pas dans le temps
│
├── gui/                       # ✅ TERMINÉ — interface graphique 2D
│   ├── robot_view.py            # rendu Matplotlib du robot (réutilisable)
│   ├── app.py                     # interface interactive Tkinter + Matplotlib
│   └── replay.py                    # rejeu graphique d'un log CSV exporté
│
├── sensors/                  # ⏳ À FAIRE — binôme Perception
│   ├── odometry.py            # encodeurs de roues simulés
│   ├── lidar.py                 # capteur de distance simulé
│   ├── landmarks.py            # balises pour recalage de position
│   └── cameras.py               # caméra frontale + caméra de surveillance
│
├── localization/             # ⏳ À FAIRE — binôme Localisation
│   └── localization.py         # EKF / AMCL / fusion odométrie+balises
│
├── planning/                  # ⏳ À FAIRE — binôme Planification
│   ├── astar.py                 # planification globale sur grille
│   └── rrt.py                     # planification / replanification rapide
│
├── control/                   # ⏳ À FAIRE — binôme Contrôle
│   └── pure_pursuit.py           # suivi de trajectoire
│
├── security/                  # ⏳ À FAIRE — binôme Sécurité
│   ├── intrusion_detector.py    # décide obstacle / intrusion / faux positif
│   ├── alert_manager.py           # génère et journalise l'événement d'alerte
│   └── speaker.py                   # sirène / son d'alerte simulé
│
├── safety/                     # ⏳ À FAIRE — binôme Sûreté
│   └── safety_manager.py          # arrêt sûr, mode dégradé, replanification
│
├── experiments/                # scénarios de test reproductibles
│   └── run_experiments.py
│
├── tests/                        # tests unitaires (pytest / unittest)
│   └── test_kinematics.py         # 17 tests, tous passants
│
├── logs/                          # historiques d'état exportés (CSV)
├── results/                       # résultats des scénarios/expériences
├── requirements.txt
├── .gitignore
├── INSTALLATION.md
└── README.md                        # ce fichier
```

Chaque dossier `⏳ À FAIRE` contient déjà un fichier avec :
- une explication du **rôle attendu** (avec référence à la section du
  cahier des charges ou du support de cours),
- l'**interface à respecter** pour rester compatible avec `robot/robot.py`,
- un **squelette de code** à compléter.

## 3. Règle d'or : comment s'intégrer au module Système/Cinématique

Tous les autres modules doivent piloter/lire le robot **uniquement** via
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
| Faire avancer le temps d'un pas | `robot.step(dt)` (généralement via `Simulator`) |

⚠️ **Important** : le module `localization/` ne doit jamais utiliser
`get_true_pose()` en dehors de ses propres tests de validation — il doit
estimer sa propre pose à partir des capteurs, sinon la boucle
perception → localisation → planification → commande (slide 10 du cours)
n'a plus de sens.

## 4. Interface graphique 2D

```bash
python -m gui.app
```

Une fenêtre s'ouvre avec :
- une vue 2D du robot (position, orientation, trace de trajectoire),
- deux curseurs pour piloter v et ω (ou les flèches du clavier),
- un bouton **ARRÊT D'URGENCE** / **Reprendre**,
- un panneau d'état en temps réel (x, y, θ, v, ω, arrêt sûr),
- un export du log en un clic.

Pour rejouer un log déjà enregistré (traçabilité, section 19) :

```bash
python -m gui.replay logs/robot_state_log.csv
python -m gui.replay logs/robot_state_log.csv --speed 4   # 4x plus rapide
```

`gui/robot_view.py` contient uniquement la logique de dessin (sans
Tkinter) : les autres binômes peuvent y ajouter le rendu des obstacles,
du chemin planifié ou du champ de vision caméra sans toucher à `app.py`.

## 5. Brancher un module dans la boucle de simulation

`simulation/Simulator` expose des callbacks pour insérer chaque module
sans modifier son code (voir `simulation/simulator.py`) :

```python
from robot.robot import Robot
from simulation.simulator import Simulator

robot = Robot()
sim = Simulator(robot)

# Exemple une fois les modules codés :
sim.on_perceive = lambda r, t: mon_lidar.scan()
sim.on_localize  = lambda r, t: mon_localizer.predict_and_correct(...)
sim.on_detect    = lambda r, t: mon_detecteur.detect(...)
sim.on_plan      = lambda r, t: mon_planificateur.replanifier_si_besoin(...)
sim.on_safety    = lambda r, t: mon_safety_manager.check(r, ...)

sim.run(duration=30.0, command_fn=lambda r, t: mon_controleur.compute_command(r, path))
```

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
