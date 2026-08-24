"""
planning/rrt.py - Planificateur RRT (Rapidly-exploring Random Tree).

Role 3 - Koja
References:
    - Cadrage, section 4 : RRT (echantillonnage aleatoire, plus rapide en
      grand espace mais trajectoire moins directe)
    - Cadrage : comparaison sur la vitesse de replanification lors d'une intrusion
    - Slide 24 du cours Dr Randria
    - LaValle, Planning Algorithms, chap. 5

Interface :
    planner = RRTPlanner(is_free, bounds)
    path = planner.plan(start=(x, y), goal=(x, y))
    # path = [(x1,y1), ...] en metres, ou [] si impossible
    # planner.last_plan_time_ms = temps de calcul en millisecondes

Note : contrairement a A* qui travaille sur une grille discrete, RRT
travaille dans l'espace continu. Il est donc complementaire :
    - A* = optimal, deterministe, sur grille
    - RRT = rapide, aleatoire, espace continu
"""

import math
import random
import time
from typing import Callable, List, Optional, Tuple

import config


Coord = Tuple[float, float]


class RRTPlanner:
    """Planificateur RRT dans un espace 2D continu.

    L'arbre explore aleatoirement l'espace libre en s'etendant
    par pas de `step_size`. Une probabilite `goal_bias` de tirer
    directement vers le but accelere la convergence.
    """

    def __init__(
        self,
        is_free: Callable[[float, float], bool],
        bounds: Tuple[float, float, float, float],
        robot_radius: float = 0.18,
        step_size: float = 0.3,
        max_iter: int = 2000,
        goal_bias: float = 0.10,
        goal_tolerance: float = 0.3,
        seed: Optional[int] = None,
    ):
        """Initialiser le planificateur RRT.

        Args:
            is_free: Fonction (x, y) -> bool. Retourne True si le
                     point est libre (pas d'obstacle, pas en dehors du monde).
            bounds: (x_min, y_min, x_max, y_max) limites du monde en metres.
            robot_radius: Rayon du robot pour la verification de collision.
            step_size: Longueur maximale d'une extension en metres.
            max_iter: Nombre maximal d'iterations.
            goal_bias: Probabilite (0-1) de tirer un echantillon vers le but.
            goal_tolerance: Distance au but pour considerer qu'on l'atteint.
            seed: Graine aleatoire pour la reproductibilite.
        """
        self.is_free = is_free
        self.x_min, self.y_min, self.x_max, self.y_max = bounds
        self.robot_radius = robot_radius
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_bias = goal_bias
        self.goal_tolerance = goal_tolerance
        self.last_plan_time_ms: float = 0.0

        if seed is not None:
            random.seed(seed)

        # Arbre : liste de noeuds [(x, y), ...]
        # Parents : parent[i] = index du noeud parent dans l'arbre
        self.tree: List[Coord] = []
        self.parent: List[int] = []

    def _distance(self, p1: Coord, p2: Coord) -> float:
        """Distance euclidienne entre deux points."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)

    def _nearest(self, point: Coord) -> int:
        """Trouver l'index du noeud le plus proche dans l'arbre."""
        best_idx = 0
        best_dist = float("inf")
        for i, node in enumerate(self.tree):
            d = self._distance(node, point)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx

    def _steer(self, from_point: Coord, to_point: Coord) -> Coord:
        """Avancer de step_size depuis from_point vers to_point.

        Si la distance est < step_size, retourne to_point directement.
        """
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 1e-9:
            return from_point

        if dist <= self.step_size:
            return to_point

        ratio = self.step_size / dist
        return (
            from_point[0] + dx * ratio,
            from_point[1] + dy * ratio,
        )

    def _segment_free(self, p1: Coord, p2: Coord) -> bool:
        """Verifier qu'un segment entre deux points est libre.

        On echantillonne le segment et verifie chaque point
        avec la fonction is_free.
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 1e-9:
            return self.is_free(p1[0], p1[1])

        n_samples = max(2, int(dist / 0.05))

        for k in range(n_samples + 1):
            t = k / n_samples
            x = p1[0] + t * dx
            y = p1[1] + t * dy
            if not self.is_free(x, y):
                return False

        return True

    def _random_sample(self, goal: Coord) -> Coord:
        """Tirer un echantillon aleatoire avec goal bias."""
        if random.random() < self.goal_bias:
            return goal
        return (
            random.uniform(self.x_min, self.x_max),
            random.uniform(self.y_min, self.y_max),
        )

    def _extract_path(self, goal_idx: int) -> List[Coord]:
        """Extraire le chemin depuis la racine jusqu'au noeud goal_idx."""
        path = []
        idx = goal_idx
        while idx != -1:
            path.append(self.tree[idx])
            idx = self.parent[idx]
        path.reverse()
        return path

    def _smooth_path(self, path: List[Coord]) -> List[Coord]:
        """Lisser le chemin par suppression de points inutiles."""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        i = 0
        while i < len(path) - 1:
            best_j = i + 1
            for j in range(len(path) - 1, i + 1, -1):
                if self._segment_free(path[i], path[j]):
                    best_j = j
                    break
            smoothed.append(path[best_j])
            i = best_j

        return smoothed

    def plan(self, start: Coord, goal: Coord) -> List[Coord]:
        """Planifier un chemin de start vers goal avec RRT.

        Args:
            start: (x, y) en metres.
            goal: (x, y) en metres.

        Returns:
            Liste de points [(x, y), ...] en metres, lissee,
            ou liste vide si aucun chemin trouve.
        """
        t0 = time.perf_counter()

        if not self.is_free(start[0], start[1]):
            self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
            return []
        if not self.is_free(goal[0], goal[1]):
            self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
            return []

        if self._distance(start, goal) < 1e-6:
            self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
            return [start]

        # Reinitialiser l'arbre
        self.tree = [start]
        self.parent = [-1]

        for _ in range(self.max_iter):
            sample = self._random_sample(goal)
            nearest_idx = self._nearest(sample)
            nearest = self.tree[nearest_idx]
            new_point = self._steer(nearest, sample)

            if not self._segment_free(nearest, new_point):
                continue

            new_idx = len(self.tree)
            self.tree.append(new_point)
            self.parent.append(nearest_idx)

            if self._distance(new_point, goal) <= self.goal_tolerance:
                if self._segment_free(new_point, goal):
                    self.tree.append(goal)
                    self.parent.append(new_idx)
                    raw_path = self._extract_path(len(self.tree) - 1)
                    smoothed = self._smooth_path(raw_path)
                    self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
                    return smoothed

        self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
        return []


def grid_to_is_free(
    grid, resolution: float, robot_radius: float, inflate: bool = True
) -> Callable[[float, float], bool]:
    """Convertir une grille d'occupation en fonction is_free.

    Args:
        grid: numpy array 2D (rows, cols), 0=libre, 1=obstacle.
        resolution: metres par cellule.
        robot_radius: rayon du robot.
        inflate: si True, inflat les obstacles du rayon du robot.

    Returns:
        Fonction (x, y) -> bool.
    """
    import numpy as np

    rows, cols = grid.shape
    inflate_cells = max(1, int(math.ceil(robot_radius / resolution))) if inflate else 0

    if inflate and inflate_cells > 0:
        inflated = grid.copy()
        occupied = np.argwhere(grid == 1)
        for (r, c) in occupied:
            for dr in range(-inflate_cells, inflate_cells + 1):
                for dc in range(-inflate_cells, inflate_cells + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        if math.sqrt(dr * dr + dc * dc) <= inflate_cells:
                            inflated[nr, nc] = 1
    else:
        inflated = grid

    def is_free(x: float, y: float) -> bool:
        col = int(x / resolution)
        row = int(y / resolution)
        if row < 0 or row >= rows or col < 0 or col >= cols:
            return False
        return inflated[row, col] == 0

    return is_free
