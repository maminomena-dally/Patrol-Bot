"""
gui/safety_app.py — Interface graphique interactive du SafetyManager (Role 5 - Tino).

Permet de piloter le robot a la main (comme gui/app.py de Malala) tout en
observant EN DIRECT la reaction du SafetyManager :
  - un obstacle imprevu qu'on fait apparaitre en un clic (lidar reel, cf. sensors/lidar.py),
  - une case pour couper les balises et voir l'incertitude de localisation
    deriver en direct (localisation reelle : Odometry + LandmarkDetector +
    Localizer, cf. localization/localization.py),
  - une case "aucun chemin trouve" (simule un echec de replanification),
  - une case "capteur indisponible",
  - l'etat de surete affiche en direct (NOMINAL / ALERTE / ARRET_SUR),
  - le journal des transitions, visible dans la fenetre.

Reutilise gui/robot_view.py (deja colore selon robot.stopped) sans le
modifier, conformement a la convention du projet. Reutilise aussi les
balises definies dans experiments/campagne_localisation.py (memes
positions, coherence entre la demo GUI et les campagnes en script).

Lancer depuis la racine du projet :
    python -m gui.safety_app

Necessite matplotlib et tkinter (voir INSTALLATION.md).
"""

import os
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
from gui.robot_view import RobotView
from safety.safety_manager import SafetyManager, EtatSurete
from sensors.lidar import LidarSensor
from sensors.odometry import Odometry
from sensors.landmarks import LandmarkDetector
from localization.localization import Localizer
from experiments.campagne_localisation import LANDMARKS


def clamp(value, vmin, vmax):
    return max(vmin, min(vmax, value))


# Meme geometrie que le cas limite "couloir bloque" (experiments/campagne_essais.py)
OBSTACLE_FIXE = {"type": "rect", "x": 5.0, "y": 0.0, "w": 0.3, "h": 5.0}
OBSTACLE_IMPREVU = {"type": "rect", "x": 10.0, "y": 0.0, "w": 0.4, "h": config.WORLD_HEIGHT}

COULEURS_ETAT = {
    "NOMINAL": "#2A7A2A",
    "ALERTE": "#E8A33D",
    "ARRET_SUR": "#C44E52",
}


class SafetyDemoGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("SafetyManager — Interface de test interactive (Rôle 5)")
        self.geometry("1150x700")
        self.minsize(980, 620)

        self.robot = Robot(initial_pose=(2.0, 7.5, 0.0))
        self.sm = SafetyManager(tentatives_max_replanification=3)
        self.lidar = LidarSensor(self.robot, obstacles=[OBSTACLE_FIXE])  # LIAISON : vrai capteur
        self.odometry = Odometry(self.robot)
        self.landmarks_detector = LandmarkDetector(self.robot, LANDMARKS)  # memes balises que campagne_localisation.py
        self.localizer = Localizer(initial_pose=(2.0, 7.5, 0.0))  # LIAISON : vraie localisation
        self.running = False
        self.dt_ms = max(1, int(config.DT * 1000))

        self.obstacle_visible = False

        self._build_layout()
        self._bind_keys()
        self._update_labels()

    # ------------------------------------------------------------------
    def _build_layout(self):
        main = ttk.Frame(self)
        main.pack(fill=tk.BOTH, expand=True)

        # --- Graphique (gauche) ---
        plot_frame = ttk.Frame(main)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(8, 6.5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.view = RobotView(self.ax, self.robot, world_size=10.0)
        self.ax.set_xlim(-0.5, config.WORLD_WIDTH + 0.5)
        self.ax.set_ylim(-0.5, config.WORLD_HEIGHT + 0.5)
        self.ax.set_title("Test interactif — SafetyManager")

        rect = patches.Rectangle(
            (OBSTACLE_FIXE["x"], OBSTACLE_FIXE["y"]), OBSTACLE_FIXE["w"], OBSTACLE_FIXE["h"],
            linewidth=1.5, edgecolor="#333", facecolor="#999")
        self.ax.add_patch(rect)

        self.obstacle_patch = patches.Rectangle(
            (OBSTACLE_IMPREVU["x"], OBSTACLE_IMPREVU["y"]),
            OBSTACLE_IMPREVU["w"], OBSTACLE_IMPREVU["h"],
            linewidth=2, edgecolor="red", facecolor="#ff9999",
            linestyle="--", visible=False)
        self.ax.add_patch(self.obstacle_patch)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # --- Panneau de commande (droite) ---
        panel = ttk.Frame(main, padding=12)
        panel.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Label(panel, text="Piloter le robot",
                  font=("TkDefaultFont", 12, "bold")).pack(anchor="w", pady=(0, 6))
        ttk.Label(panel, text=f"v (m/s) — max {config.V_MAX}").pack(anchor="w")
        self.v_var = tk.DoubleVar(value=0.0)
        ttk.Scale(panel, from_=-config.V_MAX, to=config.V_MAX, orient=tk.HORIZONTAL,
                  variable=self.v_var, command=lambda e: self._on_command_change()
                  ).pack(fill=tk.X, pady=(0, 8))
        ttk.Label(panel, text=f"ω (rad/s) — max {config.OMEGA_MAX}").pack(anchor="w")
        self.omega_var = tk.DoubleVar(value=0.0)
        ttk.Scale(panel, from_=-config.OMEGA_MAX, to=config.OMEGA_MAX, orient=tk.HORIZONTAL,
                  variable=self.omega_var, command=lambda e: self._on_command_change()
                  ).pack(fill=tk.X, pady=(0, 6))
        ttk.Label(panel, text="Flèches ↑ ↓ ← → pour piloter", foreground="#666"
                  ).pack(anchor="w", pady=(0, 10))

        btns = ttk.Frame(panel)
        btns.pack(fill=tk.X, pady=(0, 10))
        self.start_btn = ttk.Button(btns, text="▶ Démarrer", command=self.start)
        self.start_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(btns, text="⏸ Pause", command=self.pause).grid(row=0, column=1, sticky="ew")
        btns.columnconfigure((0, 1), weight=1)
        ttk.Button(panel, text="↺ Réinitialiser", command=self.reset).pack(fill=tk.X, pady=(0, 12))

        ttk.Separator(panel).pack(fill=tk.X, pady=6)

        ttk.Label(panel, text="Provoquer une situation critique",
                  font=("TkDefaultFont", 12, "bold")).pack(anchor="w", pady=(6, 6))

        self.obstacle_btn = ttk.Button(
            panel, text="🚧 Faire apparaître l'obstacle imprévu",
            command=self.toggle_obstacle)
        self.obstacle_btn.pack(fill=tk.X, pady=(0, 8))

        self.path_found_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            panel, text="Aucun chemin trouvé (échec replanification)",
            variable=self.path_found_var,
            command=lambda: None).pack(anchor="w", pady=(0, 4))

        self.capteur_indispo_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            panel, text="Capteur critique indisponible",
            variable=self.capteur_indispo_var).pack(anchor="w", pady=(0, 8))

        ttk.Label(panel, text="Incertitude de localisation (m) — en direct"
                  f" — seuil = {config.LOCALIZATION_UNCERTAINTY_MAX}").pack(anchor="w")
        self.incertitude_label = ttk.Label(panel, text="0.000 m", font=("TkDefaultFont", 11, "bold"))
        self.incertitude_label.pack(anchor="w", pady=(0, 4))

        self.balise_coupee_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            panel, text="Couper les balises (simule une perte de balise)",
            variable=self.balise_coupee_var).pack(anchor="w", pady=(0, 10))

        ttk.Button(panel, text="Tenter une reprise (resume_si_possible)",
                   command=self.tenter_reprise).pack(fill=tk.X, pady=(0, 10))

        ttk.Separator(panel).pack(fill=tk.X, pady=6)

        ttk.Label(panel, text="État de sûreté (SafetyManager)",
                  font=("TkDefaultFont", 12, "bold")).pack(anchor="w", pady=(6, 4))
        self.etat_label = ttk.Label(panel, text="NOMINAL", font=("TkDefaultFont", 14, "bold"))
        self.etat_label.pack(anchor="w", pady=(0, 8))

        ttk.Label(panel, text="Journal des transitions :").pack(anchor="w")
        self.journal_box = tk.Listbox(panel, height=8, font=("TkDefaultFont", 8))
        self.journal_box.pack(fill=tk.BOTH, expand=True, pady=(2, 6))

        self.status_bar = ttk.Label(self, text="Prêt.", relief=tk.SUNKEN, anchor="w")
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def _bind_keys(self):
        self.bind("<Up>", lambda e: self._nudge(v=0.1))
        self.bind("<Down>", lambda e: self._nudge(v=-0.1))
        self.bind("<Left>", lambda e: self._nudge(omega=0.3))
        self.bind("<Right>", lambda e: self._nudge(omega=-0.3))

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------
    def _nudge(self, v=0.0, omega=0.0):
        self.v_var.set(clamp(self.v_var.get() + v, -config.V_MAX, config.V_MAX))
        self.omega_var.set(clamp(self.omega_var.get() + omega, -config.OMEGA_MAX, config.OMEGA_MAX))
        self._on_command_change()

    def _on_command_change(self):
        if not self.robot.stopped:
            self.robot.set_velocity(self.v_var.get(), self.omega_var.get())

    def toggle_obstacle(self):
        self.obstacle_visible = not self.obstacle_visible
        self.obstacle_patch.set_visible(self.obstacle_visible)
        if self.obstacle_visible:
            self.path_found_var.set(False)  # coherent : l'obstacle bloque tout le couloir
            self.lidar.update_obstacles([OBSTACLE_FIXE, OBSTACLE_IMPREVU])  # LIAISON : lidar voit l'obstacle
            self.status_bar.config(text="Obstacle imprévu apparu — le couloir est totalement bloqué.")
        else:
            self.path_found_var.set(True)
            self.lidar.update_obstacles([OBSTACLE_FIXE])
            self.status_bar.config(text="Obstacle imprévu retiré.")
        self.canvas.draw_idle()

    def tenter_reprise(self):
        ok = self.sm.resume_si_possible(self.robot)
        self.status_bar.config(
            text="Reprise effectuée." if ok else
            "Reprise refusée : la situation est encore critique.")

    def start(self):
        if not self.running:
            self.running = True
            self.status_bar.config(text="Simulation en cours...")
            self._loop()

    def pause(self):
        self.running = False
        self.status_bar.config(text="En pause.")

    def reset(self):
        self.pause()
        self.robot.reset(initial_pose=(2.0, 7.5, 0.0))
        self.sm.reinitialiser()
        self.v_var.set(0.0)
        self.omega_var.set(0.0)
        self.incertitude_var.set(0.05)
        self.path_found_var.set(True)
        self.capteur_indispo_var.set(False)
        self.balise_coupee_var.set(False)
        self.obstacle_visible = False
        self.obstacle_patch.set_visible(False)
        self.lidar.update_obstacles([OBSTACLE_FIXE])
        self.localizer = Localizer(initial_pose=(2.0, 7.5, 0.0))  # reinitialise la localisation aussi
        self.incertitude_label.config(text="0.000 m")
        self.view.reset_trail()
        self.journal_box.delete(0, tk.END)
        self.canvas.draw_idle()
        self._update_labels()
        self.status_bar.config(text="Réinitialisé.")

    # ------------------------------------------------------------------
    # Boucle
    # ------------------------------------------------------------------
    def _loop(self):
        if not self.running:
            return

        self.robot.step(dt=config.DT)

        # -- Localisation reelle : predict (odometrie) + correct (balises) --
        d_left, d_right = self.odometry.read(config.DT)
        self.localizer.predict(d_left, d_right)
        if not self.balise_coupee_var.get():
            detections = self.landmarks_detector.detect()
            self.localizer.correct(detections)
        self.incertitude_label.config(text=f"{self.localizer.uncertainty:.3f} m")

        if self.capteur_indispo_var.get():
            incertitude, distance = None, None
        else:
            incertitude = self.localizer.uncertainty  # LIAISON : vraie incertitude, plus un curseur manuel
            distance = self.lidar.min_distance()  # LIAISON : vraie mesure, plus une heuristique

        n_avant = len(self.sm.journal)
        etat = self.sm.check(
            self.robot,
            localization_uncertainty=incertitude,
            obstacle_distance=distance,
            path_found=self.path_found_var.get(),
        )
        if len(self.sm.journal) > n_avant:
            ev = self.sm.journal[-1]
            self.journal_box.insert(tk.END, f"t={ev.t:.1f}s  {ev.transition}  ({ev.raison})")
            self.journal_box.see(tk.END)

        self.view.redraw()
        self.canvas.draw_idle()
        self._update_labels(etat)
        self.after(self.dt_ms, self._loop)

    def _update_labels(self, etat: EtatSurete = EtatSurete.NOMINAL):
        self.etat_label.config(text=etat.name, foreground=COULEURS_ETAT[etat.name])
        self.status_bar.config(
            text=f"t={self.robot.time:.1f}s — v={self.robot.v:.2f} ω={self.robot.omega:.2f} "
                 f"— stopped={self.robot.stopped}"
        )


def main():
    app = SafetyDemoGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
