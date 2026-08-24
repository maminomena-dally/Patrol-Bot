"""
planning/astar.py — Planificateur A* sur grille d'occupation.

Rôle 3 — Koja
Références :
    - Cadrage, section 4 : « Deux algorithmes de planification seront implémentés
      et comparés : A* (recherche sur grille, optimal, déterministe) »
    - Slide 25 du cours Dr Randria : f(n) = g(n) + h(n), h ne surestime jamais
    - LaValle, Planning Algorithms, chap. 2

Interface :
    planner = AStarPlanner(grid, resolution, robot_radius)
    path = planner.plan(start=(x, y), goal=(x, y))

    # path = [(x1,y1), (x2,y2), ...] en mètres, ou [] si impossible
    # planner.last_plan_time_ms = temps de calcul en millisecondes
"""

import heapq
import math
import time
from typing import Callable, List, Optional, Tuple

import numpy as np

import config


# Types alias pour la lisibilité
Coord = Tuple[float, float]
Cell = Tuple[int, int]


class AStarPlanner:
    """Planificateur A* sur grille d'occupation.

    Le constructeur pré-traite la grille (inflation des obstacles
    pour tenir compte du rayon du robot) afin que plan() soit
    rapide — important pour la replanification.
    """

    def __init__(
        self,
        grid: np.ndarray,
        resolution: float = 0.1,
        robot_radius: float = 0.18,
        eight_connected: bool = True,
    ):
        """Initialiser le planificateur.

        Args:
            grid: Tableau 2D (H, W), 0 = libre, 1 = obstacle.
                   L'origine (0,0) de la grille correspond au coin
                   bas-gauche du monde. grid[row][col] où
                   row = y discretisé (haut), col = x discretisé (droite).
            resolution: Taille d'une cellule en mètres.
            robot_radius: Rayon du robot — les obstacles sont
                          élargis de ce rayon pour la planification.
            eight_connected: Si True, 8 directions (diagonales coût √2).
                           Sinon, 4 directions uniquement.
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.eight_connected = eight_connected
        self.last_plan_time_ms: float = 0.0

        # Dimensions de la grille
        self.rows, self.cols = grid.shape

        # Pré-traitement : inflation des obstacles (Minkowski)
        # On marque comme occupée toute cellule à distance < robot_radius
        # d'un obstacle original.
        inflate_cells = max(1, int(math.ceil(robot_radius / resolution)))
        self.inflated = self._inflate(grid, inflate_cells)

        # Voisinage
        if self.eight_connected:
            # 8 directions : (drow, dcol, coût)
            self.neighbors = [
                (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
            ]
        else:
            # 4 directions
            self.neighbors = [
                (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            ]

    def _inflate(self, grid: np.ndarray, radius_cells: int) -> np.ndarray:
        """Infler les obstacles de `radius_cells` cellules.

        Utilise une convolution binaire simple : si une cellule
        est occupée, toutes les cellules dans un carré de
        côté 2*radius_cells+1 autour deviennent occupées.
        """
        inflated = grid.copy()
        rows, cols = grid.shape
        occupied = np.argwhere(grid == 1)

        for (r, c) in occupied:
            for dr in range(-radius_cells, radius_cells + 1):
                for dc in range(-radius_cells, radius_cells + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        # Vérifier distance euclidienne (pas juste le carré)
                        if math.sqrt(dr * dr + dc * dc) <= radius_cells:
                            inflated[nr, nc] = 1

        return inflated

    def _world_to_cell(self, x: float, y: float) -> Cell:
        """Convertir coordonnées monde (mètres) → cellule grille.

        Convention : x = colonne (gauche→droite), y = ligne (bas→haut).
        Le point (0, 0) monde = cellule (0, 0).
        """
        col = int(round(x / self.resolution))
        row = int(round(y / self.resolution))
        # Clamp aux limites de la grille
        col = max(0, min(col, self.cols - 1))
        row = max(0, min(row, self.rows - 1))
        return (row, col)

    def _cell_to_world(self, row: int, col: int) -> Coord:
        """Convertir cellule grille → coordonnées monde (mètres)."""
        return (col * self.resolution, row * self.resolution)

    def _heuristic(self, cell: Cell, goal: Cell) -> float:
        """Heuristique admissible : distance euclidienne.

        Ne surestime jamais le coût réel, donc A* reste optimal
        (slide 25 du cours : « Si h ne surestime jamais, A* peut
        conserver l'optimalité »).
        """
        dr = cell[0] - goal[0]
        dc = cell[1] - goal[1]
        return math.sqrt(dr * dr + dc * dc)

    def _reconstruct_path(
        self, came_from: dict, start: Cell, goal: Cell
    ) -> List[Cell]:
        """Reconstruire le chemin depuis goal jusqu'à start."""
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from.get(current)
            if current is None:
                return []  # ne devrait pas arriver
        path.append(start)
        path.reverse()
        return path

    def _smooth_path(
        self, cell_path: List[Cell], grid: np.ndarray
    ) -> List[Coord]:
        """Lisser le chemin : supprimer les points intermédiaires inutiles.

        Algorithme « line-of-sight smoothing » : on garde le premier
        point, puis on saute les points intermédiaires tant qu'il
        y a un segment libre (sans obstacle) jusqu'à un point plus
        loin sur le chemin.
        """
        if len(cell_path) <= 2:
            # Chemin trop court, juste convertir en monde
            return [self._cell_to_world(r, c) for r, c in cell_path]

        # Convertir en coordonnées monde
        world_path = [self._cell_to_world(r, c) for r, c in cell_path]

        smoothed = [world_path[0]]
        i = 0
        while i < len(world_path) - 1:
            # Chercher le point le plus lointain j > i tel que
            # le segment world_path[i] → world_path[j] est libre
            best_j = i + 1
            for j in range(len(world_path) - 1, i + 1, -1):
                if self._line_free(world_path[i], world_path[j], grid):
                    best_j = j
                    break
            smoothed.append(world_path[best_j])
            i = best_j

        return smoothed

    def _line_free(
        self, p1: Coord, p2: Coord, grid: np.ndarray
    ) -> bool:
        """Vérifier qu'un segment entre deux points est libre d'obstacles.

        On échantillonne le segment et vérifie chaque point
        dans la grille inflatée.
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 1e-9:
            # Point unique
            cell = self._world_to_cell(p1[0], p1[1])
            return self.inflated[cell[0], cell[1]] == 0

        # Nombre d'échantillons : un tous les resolution/2
        n_samples = max(2, int(dist / (self.resolution * 0.5)))
        for k in range(n_samples + 1):
            t = k / n_samples
            x = p1[0] + t * dx
            y = p1[1] + t * dy
            row, col = self._world_to_cell(x, y)
            if self.inflated[row, col] == 1:
                return False
        return True

    def plan(
        self,
        start: Coord,
        goal: Coord,
    ) -> List[Coord]:
        """Planifier un chemin de start vers goal avec A*.

        Args:
            start: (x, y) en mètres.
            goal: (x, y) en mètres.

        Returns:
            Liste de points [(x, y), ...] en mètres, lissée,
            ou liste vide si aucun chemin n'existe.
        """
        t0 = time.perf_counter()

        start_cell = self._world_to_cell(start[0], start[1])
        goal_cell = self._world_to_cell(goal[0], goal[1])

        # Si start ou goal est dans un obstacle inflaté → pas de chemin
        if self.inflated[start_cell[0], start_cell[1]] == 1:
            self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
            return []
        if self.inflated[goal_cell[0], goal_cell[1]] == 1:
            self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
            return []

        # Si start == goal
        if start_cell == goal_cell:
            self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
            return [self._cell_to_world(*start_cell)]

        # A* ---
        # Priority queue : (f_score, compteur, cell)
        # Le compteur sert de tie-break pour éviter de comparer des tuples
        counter = 0
        open_set: list = []
        heapq.heappush(
            open_set,
            (self._heuristic(start_cell, goal_cell), counter, start_cell),
        )

        came_from: dict[Cell, Cell] = {}
        g_score: dict[Cell, float] = {start_cell: 0.0}
        closed_set: set[Cell] = set()

        while open_set:
            f_current, _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue
            closed_set.add(current)

            # But atteint
            if current == goal_cell:
                cell_path = self._reconstruct_path(came_from, start_cell, goal_cell)
                world_path = self._smooth_path(cell_path, self.inflated)
                self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
                return world_path

            # Explorer les voisins
            for dr, dc, cost in self.neighbors:
                nr = current[0] + dr
                nc = current[1] + dc

                # Limites de la grille
                if nr < 0 or nr >= self.rows or nc < 0 or nc >= self.cols:
                    continue
                # Déjà visité ou obstacle
                neighbor = (nr, nc)
                if neighbor in closed_set:
                    continue
                if self.inflated[nr, nc] == 1:
                    continue

                # Coût tentatif
                tentative_g = g_score[current] + cost

                if tentative_g < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self._heuristic(neighbor, goal_cell)
                    counter += 1
                    heapq.heappush(open_set, (f, counter, neighbor))

        # Aucun chemin trouvé
        self.last_plan_time_ms = (time.perf_counter() - t0) * 1000
        return []


# ----------------------------------------------------------------------
# Fonction utilitaire pour créer une grille de test
# ----------------------------------------------------------------------
def create_test_grid(
    width_m: float = 20.0,
    height_m: float = 15.0,
    resolution: float = 0.1,
    obstacles: Optional[list] = None,
) -> np.ndarray:
    """Créer une grille d'occupation vide (0 partout) avec obstacles optionnels.

    Args:
        width_m, height_m: Dimensions du monde en mètres.
        resolution: Taille d'une cellule.
        obstacles: Liste d'obstacles, chacun étant un dict :
            {"type": "rect", "x": 5, "y": 3, "w": 2, "h": 0.3}
            ou {"type": "circle", "cx": 10, "cy": 7, "r": 1}
            (toutes les coordonnées en mètres)

    Returns:
        Grille numpy (rows × cols), 0 = libre, 1 = obstacle.
    """
    cols = int(width_m / resolution)
    rows = int(height_m / resolution)
    grid = np.zeros((rows, cols), dtype=np.int8)

    if obstacles is None:
        return grid

    for obs in obstacles:
        if obs["type"] == "rect":
            # Remplir les cellules du rectangle
            c0 = max(0, int(obs["x"] / resolution))
            r0 = max(0, int(obs["y"] / resolution))
            c1 = min(cols, int((obs["x"] + obs["w"]) / resolution))
            r1 = min(rows, int((obs["y"] + obs["h"]) / resolution))
            grid[r0:r1, c0:c1] = 1

        elif obs["type"] == "circle":
            cx = obs["cx"] / resolution
            cy = obs["cy"] / resolution
            cr = obs["r"] / resolution
            for r in range(rows):
                for c in range(cols):
                    if (r - cy) ** 2 + (c - cx) ** 2 <= cr ** 2:
                        grid[r, c] = 1

    return grid
