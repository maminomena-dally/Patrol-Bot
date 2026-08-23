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
# Journalisation (section 18, 19)
# ----------------------------------------------------------------------
LOG_DIR = "logs"
LOG_FILE = "robot_state_log.csv"
RESULTS_DIR = "results"
