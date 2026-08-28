"""
gui/auto_patrol.py — Pilotage AUTOMATIQUE en direct (Tkinter + Matplotlib).

Contrairement à gui/app.py et gui/safety_app.py (pilotage manuel), cette
interface fait patrouiller le robot de façon AUTONOME dans l'entrepôt, en
utilisant réellement la boucle complète des six modules du projet
(perception, localisation EKF, sécurité, sûreté, planification, commande)
— la même classe `_IntegrationLoop` et le même `simulation.Simulator` que
`experiments/integration_finale.py`, mais affichés en direct pas à pas
plutôt qu'exécutés hors-ligne.

Lancer avec :
    python -m gui.auto_patrol
    python -m gui.auto_patrol --planner rrt
"""

import argparse
import os
import random
import sys
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.patches as patches

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import config
from robot.robot import Robot
from simulation.simulator import Simulator
from gui.robot_view import RobotView
from experiments.integration_finale import _IntegrationLoop, INTRUDER_SCHEDULE
from experiments.entrepot_patrouille import (
    WAREHOUSE_OBSTACLES, WAREHOUSE_WAYPOINTS, WAREHOUSE_LANDMARKS,
)


COULEURS_ETAT = {"NOMINAL": "#2A7A2A", "ALERTE": "#E8A33D", "ARRET_SUR": "#C44E52"}


class AutoPatrolGUI(tk.Tk):
    def __init__(self, planner_name="astar"):
        super().__init__()
        self.title(f"Robot de patrouille — Pilotage automatique ({planner_name.upper()})")
        self.geometry("1080x700")
        self.minsize(900, 600)

        self.planner_name = planner_name
        self.running = False
        self.dt_ms = max(1, int(config.DT * 1000))

        self._setup_simulation()
        self._build_layout()
        self._draw_static_map()
        self._update_labels()

    # ------------------------------------------------------------------
    # Simulation (memes modules que experiments/integration_finale.py)
    # ------------------------------------------------------------------
    def _setup_simulation(self):
        start = WAREHOUSE_WAYPOINTS[0]
        self.robot = Robot(initial_pose=(start[0], start[1], 0.0))
        self.loop = _IntegrationLoop(
            self.planner_name, self.robot, WAREHOUSE_WAYPOINTS[1:],
            WAREHOUSE_LANDMARKS, WAREHOUSE_OBSTACLES, verbose=False,
        )
        # Seed pour reproductibilite, coherent avec experiments/integration_finale.py
        random.seed(42 if self.planner_name == "astar" else 43)

        self.sim = Simulator(self.robot, dt=config.DT)
        self.sim.on_perceive = self.loop.on_perceive
        self.sim.on_localize = self.loop.on_localize
        self.sim.on_detect = self.loop.on_detect
        self.sim.on_plan = self.loop.on_plan
        self.sim.on_safety = self.loop.on_safety

    # ------------------------------------------------------------------
    # Construction de l'interface
    # ------------------------------------------------------------------
    def _build_layout(self):
        main = ttk.Frame(self)
        main.pack(fill=tk.BOTH, expand=True)

        plot_frame = ttk.Frame(main)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(8, 6.5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.view = RobotView(self.ax, self.robot, world_size=config.WORLD_WIDTH)
        # RobotView recentre les axes sur un carre (-world_size, world_size) ;
        # la carte de l'entrepot est rectangulaire (0..20 x 0..15), on recorrige.
        self.ax.set_xlim(-0.5, config.WORLD_WIDTH + 0.5)
        self.ax.set_ylim(-0.5, config.WORLD_HEIGHT + 0.5)
        self.ax.set_title(f"Patrouille automatique — {self.planner_name.upper()}")

        (self.est_marker,) = self.ax.plot([], [], "o", color="#F44336", markersize=5,
                                           alpha=0.7, label="Pose estimée (EKF)")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        panel = ttk.Frame(main, padding=12)
        panel.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Label(panel, text="Pilotage automatique",
                  font=("TkDefaultFont", 12, "bold")).pack(anchor="w", pady=(0, 4))
        ttk.Label(panel, text="Perception + Localisation EKF + Sécurité\n"
                              "+ Sûreté + Planification + Commande",
                  foreground="#666").pack(anchor="w", pady=(0, 12))

        btns = ttk.Frame(panel)
        btns.pack(fill=tk.X, pady=(0, 14))
        self.start_btn = ttk.Button(btns, text="▶ Démarrer", command=self.start)
        self.start_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.pause_btn = ttk.Button(btns, text="⏸ Pause", command=self.pause)
        self.pause_btn.grid(row=0, column=1, sticky="ew")
        btns.columnconfigure((0, 1), weight=1)

        ttk.Separator(panel).pack(fill=tk.X, pady=8)

        ttk.Label(panel, text="État", font=("TkDefaultFont", 12, "bold")).pack(anchor="w", pady=(0, 6))
        self.state_labels = {}
        for key, label in [
            ("time", "Temps (s)"), ("waypoints", "Waypoints"),
            ("uncertainty", "Incertitude EKF (m)"), ("safety", "État de sûreté"),
            ("alert", "Niveau d'alerte max"), ("intrusions", "Pas avec intrusion"),
        ]:
            row = ttk.Frame(panel)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=label + " :").pack(side=tk.LEFT)
            val = ttk.Label(row, text="-", font=("TkDefaultFont", 10, "bold"))
            val.pack(side=tk.RIGHT)
            self.state_labels[key] = val

        self.status_bar = ttk.Label(self, text="Prêt. Clique sur Démarrer.",
                                     relief=tk.SUNKEN, anchor="w")
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def _draw_static_map(self):
        for obs in WAREHOUSE_OBSTACLES:
            rect = patches.Rectangle((obs["x"], obs["y"]), obs["w"], obs["h"],
                                      linewidth=1.2, edgecolor="#333", facecolor="#999", alpha=0.8)
            self.ax.add_patch(rect)

        lm_x = [lm["x"] for lm in WAREHOUSE_LANDMARKS]
        lm_y = [lm["y"] for lm in WAREHOUSE_LANDMARKS]
        self.ax.plot(lm_x, lm_y, "s", color="orange", markersize=6, label="Balises", zorder=5)

        wp_x = [w[0] for w in WAREHOUSE_WAYPOINTS]
        wp_y = [w[1] for w in WAREHOUSE_WAYPOINTS]
        self.ax.plot(wp_x, wp_y, "^", color="green", markersize=10, label="Waypoints", zorder=6)

        for i, (t_app, (ix, iy)) in enumerate(INTRUDER_SCHEDULE):
            self.ax.plot(ix, iy, "P", color="black", markersize=12, zorder=7,
                         label="Intrus simulé" if i == 0 else None)

        self.ax.legend(loc="upper left", fontsize=8)

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------
    def start(self):
        if not self.running:
            self.running = True
            self.status_bar.config(text="Simulation en cours...")
            self._loop()

    def pause(self):
        self.running = False
        self.status_bar.config(text="En pause.")

    # ------------------------------------------------------------------
    # Boucle de rafraichissement (basee sur Tkinter .after, non bloquante)
    # Un pas de simulation.Simulator par frame : meme classe, meme ordre
    # d'appel (on_perceive/on_localize/on_detect/on_plan/on_safety) que
    # experiments/integration_finale.py, juste execute un pas a la fois.
    # ------------------------------------------------------------------
    def _loop(self):
        if not self.running:
            return

        if self.loop.stop_fn(self.robot, self.robot.time):
            self.running = False
            m = self.loop.finalize()
            statut = "Succès" if m["success"] else f"Arrêt ({m['safety_final_state']})"
            self.status_bar.config(text=f"Terminé : {statut}")
            self._update_labels()
            return

        self.sim.run(duration=config.DT, command_fn=self.loop.command_fn)

        self.view.redraw()
        if self.loop.est_trajectory:
            ex, ey = self.loop.est_trajectory[-1]
            self.est_marker.set_data([ex], [ey])
        self.canvas.draw_idle()
        self._update_labels()
        self.after(self.dt_ms, self._loop)

    def _update_labels(self):
        m = self.loop.metrics
        self.state_labels["time"].config(text=f"{self.robot.time:.1f}")
        self.state_labels["waypoints"].config(
            text=f"{m['waypoints_reached']}/{m['waypoints_target']}")
        self.state_labels["uncertainty"].config(text=f"{self.loop.localizer.uncertainty:.3f}")
        etat = self.loop.safety_manager.etat.name
        self.state_labels["safety"].config(text=etat, foreground=COULEURS_ETAT.get(etat, "black"))
        self.state_labels["alert"].config(text=m["max_alert_level"])
        self.state_labels["intrusions"].config(text=str(m["intrusions_detected"]))


def main():
    parser = argparse.ArgumentParser(
        description="Pilotage automatique en direct du robot de patrouille (6 modules).")
    parser.add_argument("--planner", choices=["astar", "rrt"], default="astar",
                         help="Algorithme de planification (défaut : astar)")
    args = parser.parse_args()
    app = AutoPatrolGUI(planner_name=args.planner)
    app.mainloop()


if __name__ == "__main__":
    main()
