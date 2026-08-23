"""
sensors/cameras.py — À IMPLÉMENTER par le binôme Vision / Sécurité.

Rôle attendu (sections 5, 13 du cahier des charges) :
    Modéliser la caméra frontale et la caméra de surveillance comme des
    champs de vision (angle, portée) attachés au robot, et produire une
    "observation simulée" utilisée ensuite par
    security/intrusion_detector.py pour décider s'il y a un événement
    suspect (voir section 13 : caméra -> observation -> détection ->
    validation -> alerte / faux positif).

Interface à respecter :
    - Utiliser robot.get_true_pose() pour positionner le champ de vision.
    - Rester cohérent avec config.CAMERA_FRONT_FOV_DEG / CAMERA_SURV_FOV_DEG.
    - Le projet étant hors ligne (section 13), l'observation peut d'abord
      être simulée par des "cibles" injectées dans la scène.

Exemple de squelette :

    import config

    class Camera:
        def __init__(self, robot, mount_angle_deg=0.0,
                     fov_deg=config.CAMERA_FRONT_FOV_DEG, max_range=5.0):
            self.robot = robot
            self.mount_angle_deg = mount_angle_deg
            self.fov_deg = fov_deg
            self.max_range = max_range

        def observe(self, targets):
            # targets: liste de points (x, y) représentant personnes/objets
            # TODO: retourner les cibles visibles dans le champ de vision
            raise NotImplementedError
"""

# TODO(binôme vision/sécurité) : implémenter la classe Camera.
