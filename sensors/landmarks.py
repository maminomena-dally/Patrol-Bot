"""
sensors/landmarks.py — À IMPLÉMENTER par le binôme Localisation.

Rôle attendu (sections 5, 18 du cahier des charges) :
    Simuler des balises de position connue dans la carte, et détecter
    quand le robot en est suffisamment proche pour "recaler" sa position
    estimée (utile pour limiter la dérive de l'odométrie).

Interface à respecter :
    - Utiliser robot.get_true_pose() pour simuler la détection (avec bruit),
      exactement comme un vrai capteur de balise le ferait.
    - Fournir au module localization/localization.py une mesure du type
      (landmark_id, distance_mesurée, angle_mesuré) ou (x_balise, y_balise)
      selon la méthode de fusion choisie (EKF, recalage direct, etc.).

Exemple de squelette :

    class LandmarkDetector:
        def __init__(self, robot, landmarks, detection_radius=1.0):
            self.robot = robot
            self.landmarks = landmarks  # ex: [{"id": 0, "x": 2.0, "y": 3.0}, ...]
            self.detection_radius = detection_radius

        def detect(self):
            x, y, theta = self.robot.get_true_pose()
            # TODO: retourner la liste des balises visibles avec mesure bruitée
            raise NotImplementedError
"""

# TODO(binôme localisation) : implémenter la classe LandmarkDetector.
