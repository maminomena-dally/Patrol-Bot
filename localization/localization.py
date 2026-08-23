"""
localization/localization.py — À IMPLÉMENTER par le binôme Localisation.

Rôle attendu (sections 17, 18 du support de cours et cahier des charges) :
    Estimer la pose du robot (x_hat, y_hat, theta_hat) à partir des
    mesures des capteurs simulés (sensors/odometry.py, sensors/landmarks.py),
    SANS jamais lire robot.get_true_pose() en dehors des tests de
    validation. Boucle attendue : prédire (modèle) -> mesurer (capteurs)
    -> comparer -> corriger l'état (slide 17 du cours).

    Méthodes suggérées par le cours (slide 18) : EKF, AMCL, fusion
    odométrie + balises. Un simple recalage par balises suffit pour
    une première version.

Interface à respecter :
    - Ne jamais modifier robot.pose directement : la pose estimée doit
      être stockée séparément (par ex. self.estimated_pose), pour bien
      distinguer "vérité terrain" (robot.pose) et "estimation" (ce module).
    - Exposer une incertitude ou une covariance si possible, pour
      permettre à safety/safety_manager.py de déclencher un arrêt sûr en
      cas de localisation trop incertaine (voir config.LOCALIZATION_UNCERTAINTY_MAX).

Exemple de squelette :

    from robot.kinematics import Pose, integrate_euler

    class Localizer:
        def __init__(self, initial_pose):
            self.estimated_pose = initial_pose
            self.uncertainty = 0.0

        def predict(self, v, omega, dt):
            # TODO: prédire la nouvelle pose estimée avec le modèle cinématique
            raise NotImplementedError

        def correct(self, landmark_measurements):
            # TODO: recaler self.estimated_pose à partir des balises détectées
            raise NotImplementedError
"""

# TODO(binôme localisation) : implémenter la classe Localizer.
