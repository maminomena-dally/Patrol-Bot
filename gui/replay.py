"""
gui/replay.py — Rejeu graphique 2D d'un log CSV exporté par robot.export_log().

Utile pour la traçabilité (section 19 : "Rejeu : état et événements
reconstructibles depuis les logs") : après une simulation (via main.py,
experiments/run_experiments.py ou gui/app.py), on peut ré-animer la
trajectoire enregistrée sans avoir à relancer le robot.

Utilisation :
    python -m gui.replay logs/robot_state_log.csv
    python -m gui.replay logs/robot_state_log.csv --speed 4
"""

import argparse
import csv
import os
import sys

import matplotlib
try:
    import tkinter  # noqa: F401  — juste pour tester la disponibilité
    matplotlib.use("TkAgg")
except ImportError:
    # Tkinter indisponible (ex: environnement de test headless) : on laisse
    # Matplotlib choisir un backend par défaut. L'animation interactive
    # (plt.show()) ne fonctionnera pas dans ce cas, mais le chargement du
    # log et la logique de dessin restent testables.
    pass
import matplotlib.pyplot as plt
import matplotlib.animation as animation

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import config
from gui.robot_view import RobotView


class _RobotStub:
    """Objet minimal exposant `.radius`, requis par RobotView.__init__."""
    def __init__(self, radius=config.ROBOT_RADIUS):
        self.radius = radius


def load_log(path):
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "time": float(row["time"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "theta": float(row["theta"]),
                "v": float(row["v"]),
                "omega": float(row["omega"]),
                "stopped": row["stopped"] in ("True", "true", "1"),
            })
    return rows


def replay(path, speed=1.0):
    rows = load_log(path)
    if not rows:
        print(f"Log vide ou introuvable : {path}")
        return

    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    margin = config.ROBOT_RADIUS * 3 + 0.5
    world_size = max(1.0, max(max(abs(x) for x in xs), max(abs(y) for y in ys)) + margin)

    fig, ax = plt.subplots(figsize=(6, 6))
    view = RobotView(ax, _RobotStub(), world_size=world_size)
    ax.set_title(f"Rejeu : {os.path.basename(path)}")
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    interval_ms = max(1, int(config.DT * 1000 / speed))

    def update(frame_idx):
        row = rows[frame_idx]
        view.draw_pose(row["x"], row["y"], row["theta"], stopped=row["stopped"])
        time_text.set_text(f"t = {row['time']:.2f} s   v={row['v']:.2f}  ω={row['omega']:.2f}")
        return view.body, view.heading_line, view.trail_line, time_text

    anim = animation.FuncAnimation(fig, update, frames=len(rows),
                                    interval=interval_ms, blit=False, repeat=False)
    plt.show()
    return anim  # garder une référence pour éviter le garbage collection prématuré


def main():
    parser = argparse.ArgumentParser(description="Rejeu graphique d'un log robot (CSV).")
    parser.add_argument("log_path", help="Chemin du fichier CSV exporté par robot.export_log()")
    parser.add_argument("--speed", type=float, default=1.0,
                         help="Facteur d'accélération du rejeu (2 = deux fois plus vite)")
    args = parser.parse_args()
    replay(args.log_path, speed=args.speed)


if __name__ == "__main__":
    main()
