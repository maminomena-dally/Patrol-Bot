import math
from typing import List, Optional, Tuple

import config


Coord = Tuple[float, float]
Pose = Tuple[float, float, float]


class PurePursuitController:
    """Controleur Pure Pursuit pour le suivi de trajectoire.

    Principe (slide 26 du cours Dr Randria) :
        1. Trouver le lookahead point sur le chemin.
        2. Calculer alpha = angle entre cap robot et direction cible.
        3. omega = (v / Ld) * sin(alpha) ou Ld = distance au point cible.
        4. Renvoyer (v_cruise, omega).
    """

    def __init__(
        self,
        lookahead_distance: float = 0.5,
        v_cruise: float = 0.3,
        goal_tolerance: float = 0.10,
    ):
        self.lookahead_distance = lookahead_distance
        self.v_cruise = v_cruise
        self.goal_tolerance = goal_tolerance
        self._current_waypoint_idx: int = 0

    def _distance(self, p1: Coord, p2: Coord) -> float:
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.sqrt(dx * dx + dy * dy)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _find_lookahead_point(
        self, robot_pos: Coord, path: List[Coord],
    ) -> Optional[Coord]:
        if not path:
            return None

        idx = max(0, self._current_waypoint_idx)

        for i in range(idx, len(path)):
            dist = self._distance(robot_pos, path[i])
            if dist >= self.lookahead_distance:
                if i > 0:
                    d_prev = self._distance(robot_pos, path[i - 1])
                    d_curr = dist
                    if d_prev < self.lookahead_distance <= d_curr and d_curr - d_prev > 1e-6:
                        t = (self.lookahead_distance - d_prev) / (d_curr - d_prev)
                        lx = path[i - 1][0] + t * (path[i][0] - path[i - 1][0])
                        ly = path[i - 1][1] + t * (path[i][1] - path[i - 1][1])
                        self._current_waypoint_idx = i
                        return (lx, ly)

                self._current_waypoint_idx = i
                return path[i]

        self._current_waypoint_idx = len(path) - 1
        return path[-1]

    def goal_reached(self, robot_pos: Coord, path: List[Coord]) -> bool:
        if not path:
            return True
        return self._distance(robot_pos, path[-1]) <= self.goal_tolerance

    def compute_command(self, pose: Pose, path: List[Coord]) -> Tuple[float, float]:
        robot_pos = (pose[0], pose[1])
        robot_theta = pose[2]

        if not path:
            return (0.0, 0.0)

        if self.goal_reached(robot_pos, path):
            return (0.0, 0.0)

        target = self._find_lookahead_point(robot_pos, path)
        if target is None:
            return (0.0, 0.0)

        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        Ld = math.sqrt(dx * dx + dy * dy)

        if Ld < 1e-9:
            return (0.0, 0.0)

        angle_to_target = math.atan2(dy, dx)
        alpha = self._normalize_angle(angle_to_target - robot_theta)
        omega = (self.v_cruise / Ld) * math.sin(alpha)

        dist_to_goal = self._distance(robot_pos, path[-1])
        if dist_to_goal < self.lookahead_distance * 2:
            ratio = max(0.1, dist_to_goal / (self.lookahead_distance * 2))
            v = self.v_cruise * ratio
        else:
            v = self.v_cruise

        return (v, omega)

    def reset(self):
        self._current_waypoint_idx = 0
