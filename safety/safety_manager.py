"""
safety/safety_manager.py — À IMPLÉMENTER par le binôme Sûreté.

Rôle attendu (section 16 du cahier des charges) :
    Surveiller en continu l'état du système et déclencher robot.emergency_stop()
    quand une situation critique est détectée :

        Situation                          -> Action
        --------------------------------------------------------------
        Obstacle proche                    -> ralentir / évitement (planning)
        Obstacle bloquant le chemin         -> replanifier
        Aucun chemin valide                 -> arrêt sûr
        Localisation trop incertaine        -> arrêt sûr
        Capteur critique indisponible       -> mode dégradé ou arrêt
        Intrusion confirmée                 -> sirène + alerte + log (security/)
        Perte de supervision                -> continuer ou arrêt selon politique

Interface attendue :
    - `check(robot, localization_uncertainty=None, obstacle_distance=None)`
      appelée à chaque pas de la boucle de simulation (voir
      simulation/simulator.py -> sim.on_safety), qui appelle
      robot.emergency_stop() si nécessaire, sinon ne fait rien.

Exemple de squelette :

    import config

    class SafetyManager:
        def check(self, robot, localization_uncertainty=None,
                   obstacle_distance=None, path_found=True):
            if localization_uncertainty is not None and \\
               localization_uncertainty > config.LOCALIZATION_UNCERTAINTY_MAX:
                robot.emergency_stop()
                return
            if not path_found:
                robot.emergency_stop()
                return
            # TODO: compléter avec les autres règles du tableau ci-dessus
"""

# TODO(binôme sûreté) : implémenter la classe SafetyManager.
