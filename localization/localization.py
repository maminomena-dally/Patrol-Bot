"""
localization/localization.py — Filtre de Kalman Etendu (EKF) pour la localisation.

Role 2 (Kojy) — Ameliore par Role 3 (Koja) avec matrice de covariance 3x3.

Implemente un EKF standard (ref: Thrun, Burgard, Fox - Probabilistic Robotics, Chap. 7)
pour un robot differentiel avec observations range/bearing vers des balises.

Etat : [x, y, theta]
Covariance : matrice 3x3 P

Ameliorations par rapport a la version precedente :
    - Matrice de covariance 3x3 (au lieu d’un scalaire unique)
    - Jacobien F du modele de mouvement (couple position/cap)
    - Jacobien H du modele d’observation (range/bearing)
    - Gain de Kalman K propre a chaque composante (x, y, theta)
    - Mise a jour sequentielle des observations (plus stable)

Interface (preservee pour compatibilite) :
    localizer = Localizer(initial_pose, wheel_base, process_noise, measurement_noise)
    localizer.predict(d_left, d_right)
    localizer.correct(landmark_measurements)  # list of dicts avec “x”,”y”,“distance”,”angle”
    localizer.estimated_pose  # -> Pose(x, y, theta)
    localizer.uncertainty    # -> float (max(std_x, std_y), en m)
"""

import math

import numpy as np

import config
from robot.kinematics import Pose, normalize_angle


class Localizer:
    """Filtre de Kalman Etendu pour la localisation d’un robot differentiel.

    Etat : x = [x, y, theta]^T
    Covariance : P (3x3, symetrique definie positive)
    """

    def __init__(self, initial_pose, wheel_base=config.WHEEL_BASE,
                 process_noise=config.LOCALIZATION_PROCESS_NOISE,
                 measurement_noise=config.LOCALIZATION_MEASUREMENT_NOISE):
        x, y, theta = initial_pose
        self.estimated_pose = Pose(x, y, theta)
        self.wheel_base = wheel_base

        # --- Bruit de processus (par unite de mouvement) ---
        # Ref: Thrun et al., Probabilistic Robotics, Table 7.2
        # Calibres pour le robot du projet (v=0.3m/s, dt=0.05s)
        self.alpha1 = process_noise * 0.1     # translation par m
        self.alpha2 = process_noise * 0.1     # translation par rad
        self.alpha3 = process_noise * 0.4     # rotation par m
        self.alpha4 = process_noise * 0.2     # rotation par rad

        # --- Bruit de mesure ---
        self.sigma_r = config.LANDMARK_NOISE_STD_DISTANCE   # 0.05 m
        self.sigma_phi = config.LANDMARK_NOISE_STD_ANGLE    # 0.03 rad
        self.sigma_fix = measurement_noise                     # 0.1 m (confiance de base)

        # --- Covariance 3x3 ---
        # Incertitude initiale : position 1cm, cap 2 deg
        # (non nulle pour eviter un gain de Kalman nul au debut)
        self.P = np.diag([1e-4, 1e-4, (math.radians(2.0))**2])

        # --- Pour compatibilite ---
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    @property
    def uncertainty(self):
        """Incertitude de position : max(std_x, std_y).

        Retourne un scalaire (en metres) pour compatibilite avec
        SafetyManager. N’inclut PAS l’incertitude de cap (unites
        differentes — rad vs m).
        """
        std_x = math.sqrt(max(0, self.P[0, 0]))
        std_y = math.sqrt(max(0, self.P[1, 1]))
        return float(max(std_x, std_y))

    # ==================================================================
    # PREDICT
    # ==================================================================

    def predict(self, d_left, d_right):
        """Etape de prediction : integre le modele de mouvement + propage P.

        Modele de mouvement (differential drive) :
            d_center = (d_left + d_right) / 2
            d_theta  = (d_right - d_left) / L
            x’ = x + d_center * cos(theta + d_theta/2)
            y’ = y + d_center * sin(theta + d_theta/2)
            theta’ = theta + d_theta

        Jacobien F :
            F = [1,  0,  -dc * sin(mt)]
                [0,  1,   dc * cos(mt)]
                [0,  0,   1          ]

        Bruit de processus Q :
            Q = diag(alpha1*|dc|+alpha2*|dt|, alpha1*|dc|+alpha2*|dt|,
                    alpha3*|dc|+alpha4*|dt|)
        """
        d_center = (d_left + d_right) / 2.0
        d_theta = (d_right - d_left) / self.wheel_base

        theta = self.estimated_pose.theta
        mid_theta = theta + d_theta / 2.0

        # Mise a jour de l’etat
        new_x = self.estimated_pose.x + d_center * math.cos(mid_theta)
        new_y = self.estimated_pose.y + d_center * math.sin(mid_theta)
        new_theta = normalize_angle(theta + d_theta)

        # Jacobien F (3x3)
        F = np.array([
            [1.0, 0.0, -d_center * math.sin(mid_theta)],
            [0.0, 1.0,  d_center * math.cos(mid_theta)],
            [0.0, 0.0,  1.0],
        ])

        # Bruit de processus Q (3x3 diagonal)
        trans_noise = self.alpha1 * abs(d_center) + self.alpha2 * abs(d_theta)
        rot_noise = self.alpha3 * abs(d_center) + self.alpha4 * abs(d_theta)
        Q = np.diag([trans_noise, trans_noise, rot_noise])

        # Propagation de la covariance
        self.P = F @ self.P @ F.T + Q

        # S’assurer que P reste symetrique (erreurs numeriques)
        self.P = (self.P + self.P.T) / 2.0

        self.estimated_pose = Pose(new_x, new_y, new_theta)
        return self.estimated_pose

    # ==================================================================
    # CORRECT
    # ==================================================================

    def correct(self, landmark_measurements):
        """Etape de correction : met a jour l’etat et P avec les observations.

        Chaque observation : {"x": lx, "y": ly, "distance": r, "angle": phi}
        ou phi est le bearing relatif au cap du robot.

        Modele d’observation :
            q       = (lx - x)^2 + (ly - y)^2
            r_hat   = sqrt(q)
            phi_hat = atan2(ly - y, lx - x) - theta

        Jacobien H (2x3) :
            H = [-(lx-x)/sqrt(q),  -(ly-y)/sqrt(q),  0  ]
                [ (ly-y)/q,        -(lx-x)/q,       -1  ]

        Mise a jour sequentielle (une observation a la fois) pour stabilite.
        """
        if not landmark_measurements:
            return self.estimated_pose

        x = self.estimated_pose.x
        y = self.estimated_pose.y
        theta = self.estimated_pose.theta

        for m in landmark_measurements:
            lx, ly = m["x"], m["y"]
            z_r = m["distance"]   # distance mesuree
            z_phi = m["angle"]     # bearing mesure (relatif au cap)

            # --- Observation predite ---
            dx = lx - x
            dy = ly - y
            q = dx * dx + dy * dy
            q_sqrt = math.sqrt(q)

            if q_sqrt < 1e-6:
                continue  # skip si trop proche

            r_hat = q_sqrt
            phi_hat = normalize_angle(math.atan2(dy, dx) - theta)

            # --- Innovation ---
            innov_r = z_r - r_hat
            innov_phi = normalize_angle(z_phi - phi_hat)
            innovation = np.array([innov_r, innov_phi])

            # --- Jacobien H (2x3) ---
            H = np.array([
                [-dx / q_sqrt, -dy / q_sqrt,  0.0],
                [ dy / q,      -dx / q,      -1.0],
            ])

            # --- Bruit de mesure R (2x2) ---
            R = np.diag([self.sigma_r ** 2, self.sigma_phi ** 2])

            # --- Gain de Kalman ---
            S = H @ self.P @ H.T + R      # (2x2)
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                continue  # skip si S non inversible

            K = self.P @ H.T @ S_inv      # (3x2)

            # --- Mise a jour de l’etat ---
            delta = K @ innovation         # (3,)
            x = x + delta[0]
            y = y + delta[1]
            theta = normalize_angle(theta + delta[2])

            # --- Mise a jour de la covariance (Joseph form pour stabilite) ---
            I3 = np.eye(3)
            IKH = I3 - K @ H
            self.P = IKH @ self.P @ IKH.T + K @ R @ K.T

            # Symetriser
            self.P = (self.P + self.P.T) / 2.0

        self.estimated_pose = Pose(x, y, theta)
        return self.estimated_pose
