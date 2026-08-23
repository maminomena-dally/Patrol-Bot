"""
security/intrusion_detector.py — À IMPLÉMENTER par le binôme Sécurité.

Rôle attendu (section 13 du cahier des charges) :
    À partir de l'observation produite par sensors/cameras.py (et
    éventuellement sensors/lidar.py), décider si un événement est un
    FAUX POSITIF (à journaliser puis ignorer) ou une INTRUSION confirmée
    (à transmettre à security/alert_manager.py).

Interface attendue :
    - Une fonction/méthode `detect(observation) -> bool | dict` qui rend
      un verdict, avec un score de confiance si possible (voir le champ
      "confidence" du JSON d'exemple, section 15).
    - Ne doit jamais bloquer la boucle de simulation : retourner
      rapidement, quitte à affiner plus tard.

Exemple de squelette :

    class IntrusionDetector:
        def __init__(self, confidence_threshold=0.7):
            self.confidence_threshold = confidence_threshold

        def detect(self, observation):
            # TODO: analyser `observation` (issue de sensors/cameras.py)
            # et retourner par ex. {"intrusion": True, "confidence": 0.92}
            raise NotImplementedError
"""

# TODO(binôme sécurité) : implémenter la classe IntrusionDetector.
