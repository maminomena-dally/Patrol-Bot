"""
config.py — Paramètres globaux du robot de patrouille de sécurité.

Ce fichier centralise TOUTES les constantes physiques et de simulation
du projet. Les autres modules (capteurs, sécurité, planification...)
doivent lire leurs paramètres ici plutôt que de coder des valeurs "en dur",
afin que le projet reste cohérent et facile à régler pour toute l'équipe.

Référence : Robot_Patrouille_Securite_Complet.pdf, sections 4, 6, 9.
"""

# ----------------------------------------------------------------------
# Géométrie du robot (section 4 et 9 du cahier des charges)
# ----------------------------------------------------------------------
ROBOT_RADIUS = 0.18        # m — rayon du châssis (disque), pour la géométrie / collision
WHEEL_BASE = 0.35          # m — entraxe L entre les deux roues motrices (paramètre libre, section 8)
WHEEL_RADIUS = 0.05        # m — rayon d'une roue (utile si un jour on pilote en oméga_roue)

# ----------------------------------------------------------------------
# Limites cinématiques (section 9)
# ----------------------------------------------------------------------
V_MAX = 0.5                 # m/s   — vitesse linéaire maximale autorisée
OMEGA_MAX = 1.5              # rad/s — vitesse angulaire maximale autorisée

# ----------------------------------------------------------------------
# Simulation (section 9)
# ----------------------------------------------------------------------
DT = 0.05                    # s — pas de temps de simulation, fixe pour toutes les comparaisons

# ----------------------------------------------------------------------
# Pose initiale par défaut : x (m), y (m), theta (rad)
# ----------------------------------------------------------------------
INITIAL_POSE = (0.0, 0.0, 0.0)

# ----------------------------------------------------------------------
# Capteurs — paramètres par défaut, à ajuster par le binôme perception
# (utilisés par sensors/lidar.py, sensors/cameras.py quand ils seront codés)
# ----------------------------------------------------------------------
LIDAR_MAX_RANGE = 5.0        # m
LIDAR_NUM_RAYS = 36          # nombre de rayons simulés autour du robot
CAMERA_FRONT_FOV_DEG = 90    # champ de vision caméra frontale, en degrés
CAMERA_SURV_FOV_DEG = 120    # champ de vision caméra de surveillance, en degrés

# ----------------------------------------------------------------------
# Sécurité — paramètres par défaut, à ajuster par le binôme sécurité
# (utilisés par security/*.py, safety/safety_manager.py)
# ----------------------------------------------------------------------
OBSTACLE_SAFE_DISTANCE = 0.4   # m — distance en dessous de laquelle on ralentit/évite
LOCALIZATION_UNCERTAINTY_MAX = 0.5  # m — au-delà : arrêt sûr (à affiner par le binôme localisation)

# ----------------------------------------------------------------------
# Perception / Localisation (Rôle 2 — Kojy)
# ----------------------------------------------------------------------
ODOMETRY_NOISE_STD = 0.01              # m — écart-type du bruit gaussien ajouté à chaque delta de roue
LANDMARK_DETECTION_RADIUS = 6.0        # m — distance max à laquelle une balise est détectée
LANDMARK_NOISE_STD_DISTANCE = 0.05     # m — écart-type du bruit sur la distance mesurée à une balise
LANDMARK_NOISE_STD_ANGLE = 0.03        # rad — écart-type du bruit sur l'angle mesuré vers une balise
LOCALIZATION_PROCESS_NOISE = 0.008     # facteur de croissance de l'incertitude par mètre/radian parcouru (predict)
LOCALIZATION_MEASUREMENT_NOISE = 0.1   # m — confiance accordée à une mesure de balise (correct)

# Securite / intrusion (Role 1 - Koja) — centralise ici pour coherence
# avec le reste du projet (etait code en dur dans security/*.py)
DETECTION_COOLDOWN = 2.0          # s — délai min entre deux nouvelles alertes créées (IntrusionDetector)
ALERT_RESOLUTION_DELAY = 3.0      # s — délai sans intrusion avant retour NOMINAL (AlertManager)
# DOIT rester > DETECTION_COOLDOWN, sinon le niveau d'alerte oscille
# (détecté -> nominal -> détecté...) même avec un intrus visible en
# continu : entre deux créations d'alerte (tous les DETECTION_COOLDOWN),
# ALERT_RESOLUTION_DELAY doit avoir le temps de ne PAS expirer.
# Trouve et corrige par Role 5 (Tino), voir TINO_WORKFLOW.md Jour 7.

# ----------------------------------------------------------------------
# Journalisation (section 18, 19)
# ----------------------------------------------------------------------
LOG_DIR = "logs"
LOG_FILE = "robot_state_log.csv"
RESULTS_DIR = "results"

# ----------------------------------------------------------------------
# Planification (Rôle 3 — Koja)
# Référence : Cadrage, section 4 (environnement 20m×15m, résolution 0.1m)
# ----------------------------------------------------------------------
WORLD_WIDTH = 20.0             # m — largeur de la carte
WORLD_HEIGHT = 15.0            # m — hauteur de la carte
GRID_RESOLUTION = 0.1         # m par cellule → 200×150 cellules
ASTAR_8_CONNECTED = True      # 8 directions (diagonales avec coût √2)

# ----------------------------------------------------------------------
# RRT (Rôle 3 — Koja)
# ----------------------------------------------------------------------
RRT_MAX_ITER = 2000           # itérations max
RRT_STEP_SIZE = 0.3           # m — pas d'extension
RRT_GOAL_BIAS = 0.10          # probabilité de tirer vers le but
RRT_GOAL_TOLERANCE = 0.3      # m — distance au but pour considérer atteint

# ----------------------------------------------------------------------
# Contrôle — Pure Pursuit (Rôle 3 — Koja)
# Référence : Cadrage, critère < 10 cm au point cible
# ----------------------------------------------------------------------
LOOKAHEAD_DISTANCE = 0.5      # m — distance de visée
V_CRUISE = 0.3                # m/s — vitesse de croisière
GOAL_TOLERANCE = 0.10         # m — ≤ 10 cm comme exigé par le cadrage
