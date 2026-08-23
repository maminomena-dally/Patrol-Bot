"""
gui/app.py — Interface graphique 2D interactive (Tkinter + Matplotlib).

Permet de :
  - visualiser le robot (position, orientation, rayon) et sa trajectoire
    en temps réel dans un repère 2D,
  - le piloter avec deux curseurs (vitesse linéaire v, vitesse angulaire
    omega) ou au clavier (flèches directionnelles),
  - déclencher / lever un arrêt sûr (bouton "ARRÊT D'URGENCE" / "Reprendre"),
  - réinitialiser le robot,
  - exporter le log de la session en un clic (voir robot.export_log()).

Cette interface visualise le module Système/Cinématique déjà fonctionnel.
Les autres binômes peuvent enrichir l'affichage (obstacles, balises,
chemin planifié, zone de vision caméra...) en ajoutant des méthodes de
dessin dans gui/robot_view.py, sans modifier la structure de ce fichier.

Lancer depuis la racine du projet :
    python -m gui.app

Nécessite matplotlib et tkinter (voir INSTALLATION.md — tkinter est inclus
avec la plupart des distributions Python ; sous Linux il peut nécessiter
le paquet système `python3-tk`).
"""

import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import config
from robot.robot import Robot
from gui.robot_view import RobotView


def clamp(value, vmin, vmax):
    return max(vmin, min(vmax, value))


class RobotGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Robot de patrouille — Interface graphique 2D")
        self.geometry("1000x650")
        self.minsize(860, 560)

        self.robot = Robot()
        self.running = False
        self.dt_ms = max(1, int(config.DT * 1000))

        self._build_layout()
        self._bind_keys()
        self._update_state_labels()

    # ------------------------------------------------------------------
    # Construction de l'interface
    # ------------------------------------------------------------------
    def _build_layout(self):
        main = ttk.Frame(self)
        main.pack(fill=tk.BOTH, expand=True)

        # --- Panneau graphique (gauche) ---
        plot_frame = ttk.Frame(main)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(6, 6), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.view = RobotView(self.ax, self.robot)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # --- Panneau de commande (droite) ---
        panel = ttk.Frame(main, padding=12)
        panel.pack(side=tk.RIGHT, fill=tk.Y)

        ttk.Label(panel, text="Commande du robot",
                  font=("TkDefaultFont", 12, "bold")).pack(anchor="w", pady=(0, 8))

        ttk.Label(panel, text=f"Vitesse linéaire v (m/s) — max {config.V_MAX}").pack(anchor="w")
        self.v_var = tk.DoubleVar(value=0.0)
        self.v_slider = ttk.Scale(panel, from_=-config.V_MAX, to=config.V_MAX,
                                   orient=tk.HORIZONTAL, variable=self.v_var,
                                   command=lambda e: self._on_command_change())
        self.v_slider.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(panel, text=f"Vitesse angulaire ω (rad/s) — max {config.OMEGA_MAX}").pack(anchor="w")
        self.omega_var = tk.DoubleVar(value=0.0)
        self.omega_slider = ttk.Scale(panel, from_=-config.OMEGA_MAX, to=config.OMEGA_MAX,
                                       orient=tk.HORIZONTAL, variable=self.omega_var,
                                       command=lambda e: self._on_command_change())
        self.omega_slider.pack(fill=tk.X, pady=(0, 14))

        ttk.Label(panel, text="Astuce : flèches ↑ ↓ ← →   |   Espace = arrêt d'urgence",
                  foreground="#666").pack(anchor="w", pady=(0, 14))

        btns = ttk.Frame(panel)
        btns.pack(fill=tk.X, pady=(0, 14))
        self.start_btn = ttk.Button(btns, text="▶ Démarrer", command=self.start)
        self.start_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.stop_btn = ttk.Button(btns, text="⏸ Pause", command=self.pause)
        self.stop_btn.grid(row=0, column=1, sticky="ew")
        btns.columnconfigure((0, 1), weight=1)

        self.estop_btn = ttk.Button(panel, text="🛑 ARRÊT D'URGENCE", command=self.emergency_stop)
        self.estop_btn.pack(fill=tk.X, pady=(0, 4))
        self.resume_btn = ttk.Button(panel, text="Reprendre", command=self.resume)
        self.resume_btn.pack(fill=tk.X, pady=(0, 14))

        reset_btn = ttk.Button(panel, text="↺ Réinitialiser le robot", command=self.reset)
        reset_btn.pack(fill=tk.X, pady=(0, 4))
        export_btn = ttk.Button(panel, text="💾 Exporter le log (CSV)", command=self.export_log)
        export_btn.pack(fill=tk.X, pady=(0, 14))

        ttk.Separator(panel).pack(fill=tk.X, pady=8)

        ttk.Label(panel, text="État du robot",
                  font=("TkDefaultFont", 12, "bold")).pack(anchor="w", pady=(0, 6))
        self.state_labels = {}
        for key, label in [
            ("time", "Temps (s)"), ("x", "x (m)"), ("y", "y (m)"),
            ("theta", "θ (rad)"), ("v", "v appliqué (m/s)"),
            ("omega", "ω appliqué (rad/s)"), ("stopped", "Arrêt sûr"),
        ]:
            row = ttk.Frame(panel)
            row.pack(fill=tk.X)
            ttk.Label(row, text=label + " :").pack(side=tk.LEFT)
            val = ttk.Label(row, text="0", font=("TkDefaultFont", 10, "bold"))
            val.pack(side=tk.RIGHT)
            self.state_labels[key] = val

        self.status_bar = ttk.Label(self, text="Prêt.", relief=tk.SUNKEN, anchor="w")
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def _bind_keys(self):
        self.bind("<Up>", lambda e: self._nudge(v=0.1))
        self.bind("<Down>", lambda e: self._nudge(v=-0.1))
        self.bind("<Left>", lambda e: self._nudge(omega=0.3))
        self.bind("<Right>", lambda e: self._nudge(omega=-0.3))
        self.bind("<space>", lambda e: self.emergency_stop())

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------
    def _nudge(self, v=0.0, omega=0.0):
        self.v_var.set(clamp(self.v_var.get() + v, -config.V_MAX, config.V_MAX))
        self.omega_var.set(clamp(self.omega_var.get() + omega, -config.OMEGA_MAX, config.OMEGA_MAX))
        self._on_command_change()

    def _on_command_change(self):
        self.robot.set_velocity(self.v_var.get(), self.omega_var.get())

    def start(self):
        if not self.running:
            self.running = True
            self.status_bar.config(text="Simulation en cours...")
            self._loop()

    def pause(self):
        self.running = False
        self.status_bar.config(text="En pause.")

    def emergency_stop(self):
        self.robot.emergency_stop()
        self.v_var.set(0.0)
        self.omega_var.set(0.0)
        self.status_bar.config(text="ARRÊT D'URGENCE déclenché.")

    def resume(self):
        self.robot.resume()
        self.status_bar.config(text="Reprise après arrêt sûr.")

    def reset(self):
        self.pause()
        self.robot.reset()
        self.v_var.set(0.0)
        self.omega_var.set(0.0)
        self.view.reset_trail()
        self.canvas.draw_idle()
        self._update_state_labels()
        self.status_bar.config(text="Robot réinitialisé.")

    def export_log(self):
        path = self.robot.export_log()
        n = len(self.robot.history)
        self.status_bar.config(text=f"Log exporté : {path} ({n} pas enregistrés).")
        messagebox.showinfo("Export réussi", f"{n} pas enregistrés dans :\n{path}")

    # ------------------------------------------------------------------
    # Boucle de rafraîchissement (basée sur Tkinter .after, non bloquante)
    # ------------------------------------------------------------------
    def _loop(self):
        if not self.running:
            return
        self.robot.step(dt=config.DT)
        self.view.redraw()
        self.canvas.draw_idle()
        self._update_state_labels()
        self.after(self.dt_ms, self._loop)

    def _update_state_labels(self):
        state = self.robot.get_state()
        for key, label in self.state_labels.items():
            value = state[key]
            if key == "stopped":
                label.config(text="OUI" if value else "non",
                             foreground="#C44E52" if value else "#2A7A2A")
            else:
                label.config(text=f"{value:.3f}" if isinstance(value, float) else str(value))


def main():
    app = RobotGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
