"""
gui/robot_view.py — Dessin 2D du robot dans des axes Matplotlib.

Ce module contient UNIQUEMENT la logique de rendu (aucune dépendance à
Tkinter), afin de pouvoir être testé et réutilisé aussi bien par
l'interface interactive (gui/app.py) que par le rejeu de logs
(gui/replay.py), ou plus tard par le rendu d'obstacles/chemin planifié
ajouté par les autres binômes.

Point d'extension pour les autres modules : ajoutez vos propres méthodes
de dessin ici (ex: `draw_obstacles`, `draw_path`, `draw_camera_fov`) sans
modifier `redraw()` — appelez-les simplement depuis `gui/app.py`.
"""

import math

import matplotlib.patches as patches

import config


class RobotView:
    """Dessine le robot (corps + orientation) et sa trace sur un axes Matplotlib."""

    def __init__(self, ax, robot, world_size: float = 6.0):
        self.ax = ax
        self.robot = robot
        self.world_size = world_size
        self.trail_x = []
        self.trail_y = []

        self.ax.set_xlim(-world_size, world_size)
        self.ax.set_ylim(-world_size, world_size)
        self.ax.set_aspect("equal")
        self.ax.grid(True, linestyle="--", alpha=0.3)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title("Robot de patrouille — vue 2D")

        (self.trail_line,) = self.ax.plot([], [], "-", linewidth=1,
                                           color="#4C72B0", alpha=0.6, label="trajectoire")
        self.body = patches.Circle((0, 0), robot.radius, facecolor="#55A868",
                                    edgecolor="black", zorder=3, label="robot")
        self.ax.add_patch(self.body)
        (self.heading_line,) = self.ax.plot([], [], "-", linewidth=2, color="black", zorder=4)

    def redraw(self):
        """Met à jour le dessin à partir de la pose courante du robot."""
        x, y, theta = self.robot.get_true_pose()
        self.trail_x.append(x)
        self.trail_y.append(y)
        self.trail_line.set_data(self.trail_x, self.trail_y)

        self.body.center = (x, y)
        # Rouge si arrêt sûr actif, vert sinon — cohérent avec robot.stopped.
        stopped = getattr(self.robot, "stopped", False)
        self.body.set_facecolor("#C44E52" if stopped else "#55A868")

        heading_len = self.robot.radius * 1.6
        hx = x + heading_len * math.cos(theta)
        hy = y + heading_len * math.sin(theta)
        self.heading_line.set_data([x, hx], [y, hy])

    def reset_trail(self):
        self.trail_x = []
        self.trail_y = []
        self.trail_line.set_data([], [])

    def draw_pose(self, x: float, y: float, theta: float, stopped: bool = False):
        """
        Variante sans dépendre de `self.robot` : utile pour le rejeu de logs
        (gui/replay.py) où l'on ne dispose que des valeurs (x, y, theta)
        lues dans un fichier CSV.
        """
        self.trail_x.append(x)
        self.trail_y.append(y)
        self.trail_line.set_data(self.trail_x, self.trail_y)

        self.body.center = (x, y)
        self.body.set_facecolor("#C44E52" if stopped else "#55A868")

        heading_len = self.robot.radius * 1.6
        hx = x + heading_len * math.cos(theta)
        hy = y + heading_len * math.sin(theta)
        self.heading_line.set_data([x, hx], [y, hy])
