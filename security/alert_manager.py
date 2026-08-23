"""
security/alert_manager.py — À IMPLÉMENTER par le binôme Sécurité.

Rôle attendu (sections 14, 15 du cahier des charges) :
    Sur confirmation d'une intrusion (venant de intrusion_detector.py),
    déclencher le speaker (security/speaker.py) et générer un événement
    horodaté destiné à un "superviseur simulé" hors ligne, par exemple :

        {
            "event": "INTRUSION",
            "time": 12.40,
            "robot_x": 7.21,
            "robot_y": 4.63,
            "confidence": 0.92,
            "sound_alert": true
        }

    écrit dans un fichier JSON/CSV (voir config.RESULTS_DIR) ou affiché
    dans une console de supervision.

Interface attendue :
    - `notify(robot, confidence)` : construit l'événement à partir de
      `robot.get_state()` / `robot.get_true_pose()`, l'enregistre, et
      déclenche le speaker si disponible dans `robot.security["speaker"]`.

Exemple de squelette :

    import json, os, time
    import config

    class AlertManager:
        def __init__(self, output_path=None):
            self.output_path = output_path or os.path.join(
                config.RESULTS_DIR, "alerts.jsonl")

        def notify(self, robot, confidence, sound_alert=True):
            # TODO: construire l'événement, l'écrire en JSON Lines,
            # puis appeler robot.security["speaker"].play("ALERTE_INTRUSION")
            raise NotImplementedError
"""

# TODO(binôme sécurité) : implémenter la classe AlertManager.
