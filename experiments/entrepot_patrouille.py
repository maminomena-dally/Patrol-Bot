# ─── Entrepot realiste (20m x 15m) ──────────────────────────
# Format : {"type": "rect", "x", "y", "w", "h"}
# Resolution : 0.1m/cellule (200x150 grille)
# Robot radius : 0.18m (inflation automatique par AStarPlanner)
#
# Schema :
#   0.2                              19.8
#   ┌──┬──────┬──────┬──────┬──────┬──────┐
#   │  │  A1  │  A2  │  A3  │  A4  │     │
#   │Q │ rack │ rack │ rack │ rack │  S  │
#   │U │──────│──────│──────│──────│  T  │
#   │A │      │      │      │      │  O  │
#   │I │  A5  │  A6  │  A7  │  A8  │  C  │
#   │ │ rack │ rack │ rack │ rack │  K  │
#   │ │──────│──────│──────│──────│     │
#   │ │      │      │      │      │     │
#   │ │  A9  │ A10  │ A11  │ A12  │     │
#   │ │ rack │ rack │ rack │ rack │     │
#   └──┴──────┴──────┴──────┴──────┴──────┘
#   Mur sud (y=0)                    Mur nord (y=15)

WAREHOUSE_OBSTACLES = [
    # Murs perimetriques
    {"type": "rect", "x": 0.0,  "y": 0.0,  "w": 20.0, "h": 0.2},
    {"type": "rect", "x": 0.0,  "y": 14.8, "w": 20.0, "h": 0.2},
    {"type": "rect", "x": 0.0,  "y": 0.0,  "w": 0.2,  "h": 15.0},
    {"type": "rect", "x": 19.8, "y": 0.0, "w": 0.2,  "h": 15.0},

    # Separation quai/entrepot
    {"type": "rect", "x": 0.2,  "y": 6.0,  "w": 2.8,  "h": 0.3},

    # Rangee 1 : racks A1-A4
    {"type": "rect", "x": 4.0,  "y": 1.0,  "w": 2.0, "h": 0.4},
    {"type": "rect", "x": 4.0,  "y": 4.0,  "w": 2.0, "h": 0.4},
    {"type": "rect", "x": 4.0,  "y": 7.0,  "w": 2.0, "h": 0.4},
    {"type": "rect", "x": 4.0,  "y": 10.0, "w": 2.0, "h": 0.4},

    # Rangee 2 : racks A5-A8
    {"type": "rect", "x": 8.0,  "y": 1.0,  "w": 2.0, "h": 0.4},
    {"type": "rect", "x": 8.0,  "y": 4.0,  "w": 2.0, "h": 0.4},
    {"type": "rect", "x": 8.0,  "y": 7.0,  "w": 2.0, "h": 0.4},
    {"type": "rect", "x": 8.0,  "y": 10.0, "w": 2.0, "h": 0.4},

    # Rangee 3 : racks A9-A12
    {"type": "rect", "x": 12.0, "y": 1.0,  "w": 2.0, "h": 0.4},
    {"type": "rect", "x": 12.0, "y": 4.0,  "w": 2.0, "h": 0.4},
    {"type": "rect", "x": 12.0, "y": 7.0,  "w": 2.0, "h": 0.4},
    {"type": "rect", "x": 12.0, "y": 10.0, "w": 2.0, "h": 0.4},

    # Cloisons zone stockage reservee
    {"type": "rect", "x": 16.0, "y": 0.2,  "w": 0.3, "h": 7.5},
    {"type": "rect", "x": 16.0, "y": 9.0,  "w": 0.3, "h": 5.8},
]

# Waypoints de patrouille (dans les allees)
WAREHOUSE_WAYPOINTS = [
    (1.5, 1.0),    # WP1 : entree quai
    (3.5, 2.5),    # WP2 : entree allee 1
    (7.5, 5.5),    # WP3 : entre rangees 1-2
    (11.0, 8.5),   # WP4 : entre rangees 2-3
    (15.0, 5.5),   # WP5 : avant zone stockage
    (15.0, 12.0),  # WP6 : fond entrepot
    (7.5, 12.0),   # WP7 : retour fond
    (1.5, 12.0),   # WP8 : retour quai
]

# Balises pour localisation EKF (Role 2)
# Placees le long des murs et dans les allées pour couvrir le parcours de patrouille.
# Chaque balise est a portee (LANDMARK_DETECTION_RADIUS = 6.0m) d'au moins un segment
# du chemin entre deux waypoints consecutifs.
# Schema :
#   LM0----LM1----LM2     (mur sud, y~0.4)
#   |               |
#   LM3    LM4    LM5     (centre allées)
#   |               |
#   LM6----LM7----LM8     (mur nord, y~14.5)
WAREHOUSE_LANDMARKS = [
    # --- Mur sud (y=0.4) — couvre zone quai et allées basses ---
    {"id": 0, "x":  2.0, "y": 0.4},   # quai / WP1
    {"id": 1, "x": 10.0, "y": 0.4},   # centre sud / transition WP2->WP3
    {"id": 2, "x": 17.0, "y": 0.4},   # droite sud / WP5
    # --- Centre — couvrent les allées entre les colonnes de racks ---
    {"id": 3, "x":  3.5, "y": 5.5},   # WP2 / transition quai-allées
    {"id": 4, "x": 10.0, "y": 5.5},   # centre / WP3-WP4
    {"id": 5, "x": 15.0, "y": 8.5},   # droite centre / WP4-WP5
    # --- Mur nord (y=14.5) — couvre fond de l'entrepôt ---
    {"id": 6, "x":  3.0, "y": 14.5},  # gauche nord / WP8
    {"id": 7, "x": 10.0, "y": 14.5},  # centre nord / WP7
    {"id": 8, "x": 17.0, "y": 14.5},  # droite nord / WP6
]
