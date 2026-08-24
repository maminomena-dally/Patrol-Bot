"""
tests/test_planning.py — Tests unitaires pour les planificateurs A* et RRT.

Rôle 3 — Koja
Couvre :
    - A* : chemin en espace libre, contournement d'obstacle, optimalité,
      absence de chemin, lissage, performance sur grande grille.
    - RRT : (à ajouter Phase 3)

Lancer avec :
    python -m pytest tests/test_planning.py -v
ou :
    python -m unittest tests.test_planning -v
"""

import math
import sys
import os
import unittest
import time

# Permet de lancer ce fichier directement
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import numpy as np

import config
from planning.astar import AStarPlanner, create_test_grid


class TestAStarBase(unittest.TestCase):
    """Tests fondamentaux d'A*."""

    def test_chemin_en_espace_libre(self):
        """Un chemin doit être trouvé entre deux points dans un espace vide."""
        grid = create_test_grid(10.0, 10.0, resolution=0.1)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path = planner.plan(start=(1.0, 1.0), goal=(8.0, 8.0))

        self.assertGreater(len(path), 0, "Un chemin doit être trouvé")
        self.assertAlmostEqual(path[0][0], 1.0, places=1)
        self.assertAlmostEqual(path[0][1], 1.0, places=1)
        self.assertAlmostEqual(path[-1][0], 8.0, places=1)
        self.assertAlmostEqual(path[-1][1], 8.0, places=1)

    def test_chemin_contourne_obstacle(self):
        """Le robot doit contourner un obstacle rectangulaire."""
        obstacles = [
            {"type": "rect", "x": 3.0, "y": 0.0, "w": 0.3, "h": 6.0}
        ]
        grid = create_test_grid(10.0, 10.0, resolution=0.1, obstacles=obstacles)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path = planner.plan(start=(1.0, 3.0), goal=(8.0, 3.0))

        self.assertGreater(len(path), 0, "Un chemin doit contourner l'obstacle")

        # Le chemin ne doit pas traverser l'obstacle (x entre 3.0 et 3.3)
        # Avec l'inflation, le robot passe plus loin.
        # L'obstacle va de y=0 à y=6.0, le robot contourne par au-dessus.
        for (x, y) in path:
            # Le robot ne devrait pas être à l'intérieur du mur + marge
            inside_x = 2.8 <= x <= 3.5
            inside_y = 0.0 <= y < 5.8  # marge pour le contour par le haut
            self.assertFalse(
                inside_x and inside_y,
                f"Point ({x:.2f}, {y:.2f}) est dans l'obstacle"
            )

    def test_pas_de_chemin_si_goal_bloque(self):
        """Aucun chemin si le goal est entouré d'obstacles."""
        # Mur complet au milieu
        obstacles = [
            {"type": "rect", "x": 4.0, "y": 0.0, "w": 0.5, "h": 10.0}
        ]
        grid = create_test_grid(10.0, 10.0, resolution=0.1, obstacles=obstacles)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path = planner.plan(start=(1.0, 5.0), goal=(8.0, 5.0))

        # Le mur + l'inflation du robot bloquent complètement
        self.assertEqual(len(path), 0, "Aucun chemin ne doit être trouvé")

    def test_start_equals_goal(self):
        """Si start == goal, le chemin est un seul point."""
        grid = create_test_grid(10.0, 10.0, resolution=0.1)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path = planner.plan(start=(5.0, 5.0), goal=(5.0, 5.0))

        self.assertEqual(len(path), 1)

    def test_chemin_lisse(self):
        """Le chemin lissé doit avoir moins de points que le chemin brut.

        On vérifie que le lissage fonctionne sur un chemin droit :
        le lissage doit réduire drastiquement le nombre de points.
        """
        grid = create_test_grid(10.0, 10.0, resolution=0.1)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path = planner.plan(start=(1.0, 1.0), goal=(9.0, 1.0))

        # Un chemin droit lissé devrait avoir très peu de points
        # (start + goal, peut-être 1-2 intermédiaires)
        self.assertLessEqual(len(path), 5,
                             "Le chemin droit doit être fortement lissé")

    def test_temps_calcul_mesure(self):
        """Le temps de calcul doit être enregistré dans last_plan_time_ms."""
        grid = create_test_grid(10.0, 10.0, resolution=0.1)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        planner.plan(start=(1.0, 1.0), goal=(8.0, 8.0))

        self.assertGreater(planner.last_plan_time_ms, 0.0)
        self.assertLess(planner.last_plan_time_ms, 5000.0,
                        "La planification ne doit pas prendre plus de 5 s")


class TestAStarOptimalite(unittest.TestCase):
    """Tests d'optimalité et de qualité du chemin."""

    def test_chemin_horizontal_optimal(self):
        """En ligne droite sans obstacle, le chemin doit être quasiment optimal.

        La distance du chemin doit être proche de la distance euclidienne.
        """
        grid = create_test_grid(10.0, 10.0, resolution=0.1)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path = planner.plan(start=(1.0, 1.0), goal=(9.0, 1.0))

        # Calculer la longueur du chemin
        length = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            length += math.sqrt(dx * dx + dy * dy)

        # Distance optimale = 8.0 m
        optimal = 8.0
        # Le chemin lissé ne doit pas être plus de 5% plus long
        self.assertLess(length, optimal * 1.05,
                         f"Chemin trop long : {length:.3f} m vs {optimal} m optimal")

    def test_chemin_diagonal(self):
        """Le chemin diagonal doit utiliser les 8 directions."""
        grid = create_test_grid(10.0, 10.0, resolution=0.1)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18,
                               eight_connected=True)
        path = planner.plan(start=(1.0, 1.0), goal=(9.0, 9.0))

        self.assertGreater(len(path), 0)
        length = sum(
            math.sqrt((path[i][0] - path[i-1][0])**2 +
                      (path[i][1] - path[i-1][1])**2)
            for i in range(1, len(path))
        )
        # Distance optimale diagonale = 8*sqrt(2) ≈ 11.31
        optimal = 8.0 * math.sqrt(2)
        self.assertLess(length, optimal * 1.05)


class TestAStarPerformance(unittest.TestCase):
    """Tests de performance sur la taille réelle du projet (200×150)."""

    def test_grille_pleine_200x150(self):
        """A* doit fonctionner sur la grille 20m×15m (200×150 cellules).

        Le temps de calcul initial doit être raisonnable (< 1 s).
        """
        # Grille vide 20m × 15m
        grid = create_test_grid(
            width_m=config.WORLD_WIDTH,
            height_m=config.WORLD_HEIGHT,
            resolution=config.GRID_RESOLUTION,
        )

        t0 = time.perf_counter()
        planner = AStarPlanner(
            grid,
            resolution=config.GRID_RESOLUTION,
            robot_radius=config.ROBOT_RADIUS,
        )
        init_time = (time.perf_counter() - t0) * 1000

        # Planification
        path = planner.plan(start=(1.0, 1.0), goal=(18.0, 13.0))
        plan_time = planner.last_plan_time_ms

        self.assertGreater(len(path), 0)
        self.assertLess(init_time, 2000.0,
                         f"Initialisation trop lente : {init_time:.0f} ms")
        self.assertLess(plan_time, 1000.0,
                         f"Planification trop lente : {plan_time:.0f} ms")

    def test_replanification_rapide(self):
        """La replanification (appel répété de plan()) doit être rapide.

        Le document de cadrage exige de comparer la vitesse de
        replanification d'A* vs RRT.
        """
        obstacles = [
            {"type": "rect", "x": 8.0, "y": 0.0, "w": 0.3, "h": 10.0},
            {"type": "rect", "x": 4.0, "y": 5.0, "w": 5.0, "h": 0.3},
        ]
        grid = create_test_grid(
            width_m=config.WORLD_WIDTH,
            height_m=config.WORLD_HEIGHT,
            resolution=config.GRID_RESOLUTION,
            obstacles=obstacles,
        )
        planner = AStarPlanner(
            grid,
            resolution=config.GRID_RESOLUTION,
            robot_radius=config.ROBOT_RADIUS,
        )

        # 10 replanifications
        times = []
        for i in range(10):
            goal = (15.0 + (i % 3), 3.0 + (i % 4))
            planner.plan(start=(1.0, 1.0), goal=goal)
            times.append(planner.last_plan_time_ms)

        avg_time = sum(times) / len(times)
        self.assertLess(avg_time, 500.0,
                         f"Replanification moyenne trop lente : {avg_time:.1f} ms")


class TestAStarInflation(unittest.TestCase):
    """Tests de l'inflation des obstacles."""

    def test_robot_ne_passe_pas_a_travers_mur(self):
        """Le robot (rayon 0.18m) ne doit pas planifier un chemin
        qui le fait frôler un mur de trop près.
        """
        # Mur fin
        obstacles = [
            {"type": "rect", "x": 5.0, "y": 0.0, "w": 0.1, "h": 10.0}
        ]
        grid = create_test_grid(10.0, 10.0, resolution=0.1, obstacles=obstacles)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path = planner.plan(start=(2.0, 5.0), goal=(8.0, 5.0))

        self.assertGreater(len(path), 0)

        # Vérifier que chaque point du chemin est à > robot_radius du mur
        # Le mur est à x=5.0, donc le robot doit passer à x < 4.82 ou x > 5.28
        for (x, y) in path:
            dist_to_wall = abs(x - 5.0)
            self.assertGreater(
                dist_to_wall, config.ROBOT_RADIUS - 0.05,
                f"Point ({x:.2f}, {y:.2f}) trop proche du mur : {dist_to_wall:.3f} m"
            )


class TestCreateTestGrid(unittest.TestCase):
    """Tests de la fonction utilitaire create_test_grid."""

    def test_grille_vide(self):
        grid = create_test_grid(10.0, 10.0, resolution=0.1)
        self.assertEqual(grid.shape, (100, 100))
        self.assertEqual(np.sum(grid), 0)

    def test_grille_avec_obstacle_rect(self):
        obstacles = [{"type": "rect", "x": 0.0, "y": 0.0, "w": 1.0, "h": 1.0}]
        grid = create_test_grid(10.0, 10.0, resolution=0.1, obstacles=obstacles)
        # 1.0m / 0.1m = 10 cellules
        self.assertEqual(np.sum(grid), 100)

    def test_grille_dimensions_cadrage(self):
        grid = create_test_grid(
            width_m=config.WORLD_WIDTH,
            height_m=config.WORLD_HEIGHT,
            resolution=config.GRID_RESOLUTION,
        )
        self.assertEqual(grid.shape, (150, 200))
