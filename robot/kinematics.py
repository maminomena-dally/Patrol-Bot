"""
robot/kinematics.py — Modèle cinématique du robot différentiel (non-holonome).

Implémente exactement les équations du cadrage (Robot_Patrouille_Securite_Complet.pdf,
sections 7, 8, 10) et du support de cours (Robotique_mobile, slides 13-15) :

    v      = (vR + vL) / 2
    omega  = (vR - vL) / L

    x_dot     = v * cos(theta)
    y_dot     = v * sin(theta)
    theta_dot = omega

    x(k+1)     = x(k) + v(k) * cos(theta(k)) * dt
    y(k+1)     = y(k) + v(k) * sin(theta(k)) * dt
    theta(k+1) = theta(k) + omega(k) * dt

Ce module ne dépend d'aucun autre module du projet : il est pur et testable
indépendamment (voir tests/test_kinematics.py).
"""

import math
from dataclasses import dataclass


@dataclass
class Pose:
    """Pose 2D du robot : q = [x, y, theta] (section 6 du cahier des charges)."""
    x: float
    y: float
    theta: float


# ---------------------------------------------------------------------
# Utilitaires
# ---------------------------------------------------------------------
def clamp(value, vmin, vmax):
    """Sature `value` dans l'intervalle [vmin, vmax]."""
    return max(vmin, min(vmax, value))


def normalize_angle(theta):
    """Ramène un angle (rad) dans l'intervalle [-pi, pi]."""
    return (theta + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------
# Commande : saturation et conversion corps <-> roues
# ---------------------------------------------------------------------
def saturate_command(v, omega, v_max, omega_max):
    """
    Sature la commande (v, omega) selon les limites du robot (section 9) :
        |v|     <= v_max
        |omega| <= omega_max
    """
    v_sat = clamp(v, -v_max, v_max)
    omega_sat = clamp(omega, -omega_max, omega_max)
    return v_sat, omega_sat


def wheel_speeds_to_body(vL, vR, wheel_base):
    """
    Convertit les vitesses de roues (vL, vR, en m/s) en vitesse robot (v, omega).
    v     = (vR + vL) / 2
    omega = (vR - vL) / L
    """
    v = (vR + vL) / 2.0
    omega = (vR - vL) / wheel_base
    return v, omega


def body_to_wheel_speeds(v, omega, wheel_base):
    """
    Convertit une commande robot (v, omega) en vitesses de roues (vL, vR, en m/s).
    Relation inverse de wheel_speeds_to_body.
    """
    vR = v + (omega * wheel_base) / 2.0
    vL = v - (omega * wheel_base) / 2.0
    return vL, vR


def wheel_angular_speeds_to_body(omega_R, omega_L, wheel_radius, wheel_base):
    """
    Variante utilisant les vitesses angulaires des roues (rad/s), comme dans
    le support de cours (slides 13-14) :
        v     = r * (omega_R + omega_L) / 2
        omega = r * (omega_R - omega_L) / L
    """
    v = wheel_radius * (omega_R + omega_L) / 2.0
    omega = wheel_radius * (omega_R - omega_L) / wheel_base
    return v, omega


# ---------------------------------------------------------------------
# Intégration de la pose
# ---------------------------------------------------------------------
def pose_derivative(pose: Pose, v: float, omega: float):
    """Retourne (x_dot, y_dot, theta_dot) pour la pose et la commande données."""
    dx = v * math.cos(pose.theta)
    dy = v * math.sin(pose.theta)
    dtheta = omega
    return dx, dy, dtheta


def integrate_euler(pose: Pose, v: float, omega: float, dt: float) -> Pose:
    """
    Mise à jour discrète de la pose par intégration d'Euler explicite
    (section 10 du cahier des charges). Retourne une NOUVELLE Pose
    (le module reste sans effet de bord).
    """
    dx, dy, dtheta = pose_derivative(pose, v, omega)
    new_x = pose.x + dx * dt
    new_y = pose.y + dy * dt
    new_theta = normalize_angle(pose.theta + dtheta * dt)
    return Pose(new_x, new_y, new_theta)
