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
from planning.rrt import RRTPlanner, grid_to_is_free
from control.pure_pursuit import PurePursuitController


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


class TestRRTBase(unittest.TestCase):
    """Tests fondamentaux du RRT."""

    def _make_planner(self, obstacles=None, seed=42):
        """Créer un RRTPlanner sur une grille de test."""
        grid = create_test_grid(10.0, 10.0, resolution=0.1, obstacles=obstacles)
        is_free = grid_to_is_free(grid, 0.1, 0.18)
        return RRTPlanner(
            is_free=is_free,
            bounds=(0, 0, 10, 10),
            robot_radius=0.18,
            step_size=0.3,
            max_iter=2000,
            goal_bias=0.10,
            goal_tolerance=0.3,
            seed=seed,
        )

    def test_chemin_en_espace_libre(self):
        """Un chemin doit être trouvé en espace vide."""
        planner = self._make_planner()
        path = planner.plan(start=(1.0, 1.0), goal=(8.0, 8.0))

        self.assertGreater(len(path), 0, "Un chemin doit être trouvé")
        self.assertAlmostEqual(path[0][0], 1.0, places=1)
        self.assertAlmostEqual(path[-1][0], 8.0, places=1)
        self.assertAlmostEqual(path[-1][1], 8.0, places=1)

    def test_chemin_contourne_obstacle(self):
        """Le robot doit contourner un obstacle."""
        obstacles = [{"type": "rect", "x": 3.0, "y": 0.0, "w": 0.3, "h": 6.0}]
        planner = self._make_planner(obstacles=obstacles)
        path = planner.plan(start=(1.0, 3.0), goal=(8.0, 3.0))

        self.assertGreater(len(path), 0, "Un chemin doit contourner l'obstacle")

    def test_pas_de_chemin_si_start_bloque(self):
        """Aucun chemin si le start est dans un obstacle."""
        # Mur couvrant tout le coin bas-gauche
        obstacles = [{"type": "rect", "x": 0.0, "y": 0.0, "w": 3.0, "h": 3.0}]
        planner = self._make_planner(obstacles=obstacles)
        path = planner.plan(start=(1.0, 1.0), goal=(8.0, 8.0))

        self.assertEqual(len(path), 0, "Aucun chemin depuis un obstacle")

    def test_start_equals_goal(self):
        """Si start == goal, le chemin est un seul point."""
        planner = self._make_planner()
        path = planner.plan(start=(5.0, 5.0), goal=(5.0, 5.0))
        self.assertEqual(len(path), 1)

    def test_temps_calcul_mesure(self):
        """Le temps de calcul doit être enregistré."""
        planner = self._make_planner()
        planner.plan(start=(1.0, 1.0), goal=(8.0, 8.0))

        self.assertGreater(planner.last_plan_time_ms, 0.0)
        self.assertLess(planner.last_plan_time_ms, 5000.0)

    def test_chemin_lisse(self):
        """Le chemin lissé doit avoir peu de points en ligne droite."""
        planner = self._make_planner()
        path = planner.plan(start=(1.0, 1.0), goal=(9.0, 1.0))
        self.assertGreater(len(path), 0)
        # Le lissage doit réduire les points
        self.assertLessEqual(len(path), 10)


class TestRRTPerformance(unittest.TestCase):
    """Tests de performance du RRT."""

    def test_replanification_rapide(self):
        """Replanification répétée doit être rapide (< 500 ms en moyenne)."""
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
        is_free = grid_to_is_free(
            grid, config.GRID_RESOLUTION, config.ROBOT_RADIUS
        )
        planner = RRTPlanner(
            is_free=is_free,
            bounds=(0, 0, config.WORLD_WIDTH, config.WORLD_HEIGHT),
            robot_radius=config.ROBOT_RADIUS,
            step_size=config.RRT_STEP_SIZE,
            max_iter=config.RRT_MAX_ITER,
            goal_bias=config.RRT_GOAL_BIAS,
            goal_tolerance=config.RRT_GOAL_TOLERANCE,
            seed=42,
        )

        times = []
        for i in range(10):
            goal = (15.0 + (i % 3), 3.0 + (i % 4))
            path = planner.plan(start=(1.0, 1.0), goal=goal)
            times.append(planner.last_plan_time_ms)

        avg_time = sum(times) / len(times)
        self.assertLess(avg_time, 500.0,
                         f"Replanification RRT trop lente : {avg_time:.1f} ms")


class TestAstarVsRRT(unittest.TestCase):
    """Comparaison directe A* vs RRT (pour le rapport, section 5)."""

    def test_deux_algorithmes_trouvent_chemin(self):
        """Les deux algorithmes doivent trouver un chemin sur la même carte."""
        obstacles = [
            {"type": "rect", "x": 5.0, "y": 0.0, "w": 0.3, "h": 7.0},
        ]
        grid = create_test_grid(10.0, 10.0, resolution=0.1, obstacles=obstacles)

        # A*
        astar = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path_astar = astar.plan(start=(1.0, 3.0), goal=(8.0, 3.0))

        # RRT
        is_free = grid_to_is_free(grid, 0.1, 0.18)
        rrt = RRTPlanner(is_free, bounds=(0, 0, 10, 10), seed=42)
        path_rrt = rrt.plan(start=(1.0, 3.0), goal=(8.0, 3.0))

        self.assertGreater(len(path_astar), 0, "A* doit trouver un chemin")
        self.assertGreater(len(path_rrt), 0, "RRT doit trouver un chemin")

    def test_astar_plus_court_que_rrt(self):
        """A* doit produire un chemin plus court (optimal) que RRT."""
        grid = create_test_grid(10.0, 10.0, resolution=0.1)

        astar = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path_astar = astar.plan(start=(1.0, 1.0), goal=(9.0, 9.0))

        is_free = grid_to_is_free(grid, 0.1, 0.18)
        rrt = RRTPlanner(is_free, bounds=(0, 0, 10, 10), seed=42)
        path_rrt = rrt.plan(start=(1.0, 1.0), goal=(9.0, 9.0))

        def path_length(p):
            return sum(
                math.sqrt((p[i][0]-p[i-1][0])**2 + (p[i][1]-p[i-1][1])**2)
                for i in range(1, len(p))
            )

        len_astar = path_length(path_astar)
        len_rrt = path_length(path_rrt)

        # A* doit être au moins aussi bon (optimalité garantie)
        self.assertLessEqual(len_astar, len_rrt * 1.1,
            f"A* ({len_astar:.2f}m) devrait etre <= RRT ({len_rrt:.2f}m)")


class TestPurePursuit(unittest.TestCase):
    """Tests du controleur Pure Pursuit."""

    def test_chemin_vide_arrete(self):
        """Chemin vide -> commande (0, 0)."""
        ctrl = PurePursuitController()
        v, omega = ctrl.compute_command(pose=(0, 0, 0), path=[])
        self.assertEqual(v, 0.0)
        self.assertEqual(omega, 0.0)

    def test_deja_au_but_arrete(self):
        """Si le robot est deja au but (< 10 cm), commande (0, 0)."""
        ctrl = PurePursuitController(goal_tolerance=0.10)
        # Robot a (5.05, 5.05), but a (5.0, 5.0) -> distance = 0.07 m < 0.10
        v, omega = ctrl.compute_command(
            pose=(5.05, 5.05, 0),
            path=[(0, 0), (3, 0), (5.0, 5.0)]
        )
        self.assertEqual(v, 0.0)
        self.assertEqual(omega, 0.0)

    def test_chemin_droit_omega_nul(self):
        """Sur un chemin droit dans la direction du robot, omega = 0."""
        ctrl = PurePursuitController(lookahead_distance=0.5, v_cruise=0.3)
        # Robot oriente vers +x, chemin droit vers +x
        v, omega = ctrl.compute_command(
            pose=(0, 0, 0),
            path=[(1, 0), (2, 0), (3, 0), (4, 0), (5, 0)]
        )
        self.assertAlmostEqual(omega, 0.0, places=3,
                               msg="Sur un chemin droit aligne, omega doit etre ~0")
        self.assertGreater(v, 0, "Le robot doit avancer")

    def test_virage_omega_non_nul(self):
        """Sur un chemin qui tourne, omega doit etre non nul."""
        ctrl = PurePursuitController(lookahead_distance=0.5, v_cruise=0.3)
        # Robot oriente vers +x, mais le lookahead point est en haut a droite
        # Le point (0.3, 0.4) est a distance ~0.5 et angle ~53 deg
        v, omega = ctrl.compute_command(
            pose=(0, 0, 0),
            path=[(0.3, 0.4), (1, 1), (2, 3)]
        )
        self.assertNotAlmostEqual(omega, 0.0, places=1,
                                  msg="Le robot doit tourner")

    def test_tolerance_but_respectee(self):
        """Le robot s'arrete a <= 0.10 m du but.

        Simule une poursuite pas a pas et verifie la precision.
        """
        ctrl = PurePursuitController(
            lookahead_distance=0.5,
            v_cruise=0.5,
            goal_tolerance=0.10,
        )
        path = [(5.0, 0.0)]

        # Simuler avec Euler a DT=0.05
        x, y, theta = 0.0, 0.0, 0.0
        for _ in range(500):  # max 25 secondes
            v, omega = ctrl.compute_command(pose=(x, y, theta), path=path)
            if v == 0.0 and omega == 0.0:
                break
            # Integration Euler
            x += v * math.cos(theta) * config.DT
            y += v * math.sin(theta) * config.DT
            theta += omega * config.DT

        dist = math.sqrt((x - 5.0)**2 + (y - 0.0)**2)
        self.assertLessEqual(dist, config.GOAL_TOLERANCE,
            f"Robot s'est arrete a {dist:.4f} m du but (> 0.10 m)")

    def test_reset_recommence_debut(self):
        """reset() doit remettre l'index a 0."""
        ctrl = PurePursuitController()
        ctrl._current_waypoint_idx = 5
        ctrl.reset()
        self.assertEqual(ctrl._current_waypoint_idx, 0)

    def test_deceleration_pres_du_but(self):
        """Le robot ralentit quand il est proche du but."""
        ctrl = PurePursuitController(
            lookahead_distance=0.5,
            v_cruise=0.3,
            goal_tolerance=0.10,
        )
        # Loin du but -> vitesse = v_cruise
        v1, _ = ctrl.compute_command(
            pose=(0, 0, 0),
            path=[(10, 0)]
        )
        # Pres du but (< 2*lookahead = 1.0 m)
        v2, _ = ctrl.compute_command(
            pose=(9.5, 0, 0),
            path=[(10, 0)]
        )
        self.assertLess(v2, v1, "Le robot doit ralentir pres du but")


class TestPurePursuitIntegration(unittest.TestCase):
    """Integration : A* + Pure Pursuit sur le Robot."""

    def test_astar_pure_pursuit_atteint_le_but(self):
        """Le robot suit un chemin A* et atteint le but a < 15 cm."""
        from robot.robot import Robot
        from simulation.simulator import Simulator

        grid = create_test_grid(10.0, 10.0, resolution=0.1)
        planner = AStarPlanner(grid, resolution=0.1, robot_radius=0.18)
        path = planner.plan(start=(0.0, 0.0), goal=(8.0, 8.0))

        self.assertGreater(len(path), 0)

        robot = Robot()
        sim = Simulator(robot)
        ctrl = PurePursuitController(
            lookahead_distance=0.5,
            v_cruise=0.5,
            goal_tolerance=0.10,
        )

        def command_fn(r, t):
            px, py, pth = r.get_true_pose()
            v, omega = ctrl.compute_command(
                pose=(px, py, pth),
                path=path,
            )
            r.set_velocity(v, omega)

        sim.run(duration=60.0, command_fn=command_fn, verbose=False)

        px, py, _ = robot.get_true_pose()
        dist = math.sqrt((px - 8.0)**2 + (py - 8.0)**2)
        self.assertLessEqual(dist, 0.15,
            f"Robot a {dist:.3f} m du but apres 60s")

    def test_rrt_pure_pursuit_atteint_le_but(self):
        """Le robot suit un chemin RRT et atteint le but."""
        from robot.robot import Robot
        from simulation.simulator import Simulator

        grid = create_test_grid(10.0, 10.0, resolution=0.1)
        is_free = grid_to_is_free(grid, 0.1, 0.18)
        rrt = RRTPlanner(is_free, bounds=(0, 0, 10, 10), seed=42)
        path = rrt.plan(start=(0.0, 0.0), goal=(8.0, 8.0))

        self.assertGreater(len(path), 0)

        robot = Robot()
        sim = Simulator(robot)
        ctrl = PurePursuitController(
            lookahead_distance=0.5,
            v_cruise=0.5,
            goal_tolerance=0.10,
        )

        def command_fn(r, t):
            px, py, pth = r.get_true_pose()
            v, omega = ctrl.compute_command(
                pose=(px, py, pth),
                path=path,
            )
            r.set_velocity(v, omega)

        sim.run(duration=60.0, command_fn=command_fn, verbose=False)

        px, py, _ = robot.get_true_pose()
        dist = math.sqrt((px - 8.0)**2 + (py - 8.0)**2)
        self.assertLessEqual(dist, 0.20,
            f"Robot a {dist:.3f} m du but apres 60s (RRT)")
