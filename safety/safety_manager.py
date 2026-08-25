from dataclasses import dataclass, field
from enum import Enum, auto

import config


class EtatSurete(Enum):
    NOMINAL = auto()     # tout va bien
    ALERTE = auto()       # risque détecté, tolérance transitoire en cours
    ARRET_SUR = auto()    # robot.emergency_stop() déclenché


@dataclass
class EvenementSurete:
    """Une ligne du journal de sûreté (utile pour le rapport / le rejeu)."""
    t: float
    transition: str
    raison: str
    localization_uncertainty: float | None
    obstacle_distance: float | None
    path_found: bool


class SafetyManager:
    def __init__(
        self,
        obstacle_safe_distance: float = config.OBSTACLE_SAFE_DISTANCE,
        localization_uncertainty_max: float = config.LOCALIZATION_UNCERTAINTY_MAX,
        tentatives_max_replanification: int = 3,
    ):
        self.obstacle_safe_distance = obstacle_safe_distance
        self.localization_uncertainty_max = localization_uncertainty_max
        self.tentatives_max_replanification = tentatives_max_replanification

        self.etat = EtatSurete.NOMINAL
        self._echecs_chemin_consecutifs = 0
        self.journal: list[EvenementSurete] = []

    # ------------------------------------------------------------------
    # Interface principale — appelée à chaque pas via sim.on_safety
    # ------------------------------------------------------------------
    def check(
        self,
        robot,
        localization_uncertainty: float | None = None,
        obstacle_distance: float | None = None,
        path_found: bool = True,
        intrusion_confirmed: bool = False,
    ) -> EtatSurete:
        """
        À appeler à CHAQUE pas de la boucle de simulation.

        Args:
            robot: instance de robot.robot.Robot (pour emergency_stop()/resume())
            localization_uncertainty: incertitude courante (m), typiquement
                `localizer.uncertainty` (localization/localization.py)
            obstacle_distance: distance au plus proche obstacle (m), issue
                d'un capteur (ex: sensors/lidar.py, une fois disponible)
            path_found: False si le planificateur n'a pas trouvé de chemin
                (ex: `AStarPlanner.plan()` a retourné `[]`)
            intrusion_confirmed: True si security/intrusion_detector.py a
                confirmé une intrusion (déclenche l'alerte, pas l'arrêt du
                robot en soi -- une patrouille de sécurité doit pouvoir
                continuer à surveiller après une intrusion confirmée)

        Returns:
            L'état de sûreté courant.
        """
        t = robot.time

        # -- Capteur critique indisponible : pas de valeur d'incertitude
        #    ni de distance d'obstacle -> on ne peut pas garantir la sûreté,
        #    mode dégradé = arrêt (politique prudente par défaut).
        capteur_indisponible = (
            localization_uncertainty is None and obstacle_distance is None
        )

        # -- Suivi des échecs de replanification consécutifs --
        if path_found:
            self._echecs_chemin_consecutifs = 0
        else:
            self._echecs_chemin_consecutifs += 1

        echec_chemin_persistant = (
            self._echecs_chemin_consecutifs >= self.tentatives_max_replanification
        )

        localisation_incertaine = (
            localization_uncertainty is not None
            and localization_uncertainty > self.localization_uncertainty_max
        )

        # -- Décision --
        ancien_etat = self.etat
        raison = None

        if echec_chemin_persistant:
            self.etat = EtatSurete.ARRET_SUR
            raison = "aucun_chemin_valide"
        elif localisation_incertaine:
            self.etat = EtatSurete.ARRET_SUR
            raison = "localisation_trop_incertaine"
        elif capteur_indisponible:
            self.etat = EtatSurete.ARRET_SUR
            raison = "capteur_critique_indisponible"
        elif not path_found:
            # échec isolé, pas encore persistant : on tolère (replanification
            # en cours côté planning), on journalise en ALERTE
            self.etat = EtatSurete.ALERTE
            raison = "echec_replanification_transitoire"
        else:
            self.etat = EtatSurete.NOMINAL

        # -- Application de la décision sur le robot --
        if self.etat == EtatSurete.ARRET_SUR:
            robot.emergency_stop()

        # -- Intrusion confirmée : alerte, indépendante de l'arrêt du robot --
        if intrusion_confirmed:
            self._declencher_alerte(robot, t)

        # -- Journalisation des transitions --
        if self.etat != ancien_etat:
            self.journal.append(EvenementSurete(
                t=round(t, 3),
                transition=f"{ancien_etat.name} -> {self.etat.name}",
                raison=raison or "retour_nominal",
                localization_uncertainty=localization_uncertainty,
                obstacle_distance=obstacle_distance,
                path_found=path_found,
            ))

        return self.etat

    def _declencher_alerte(self, robot, t: float):
        """
        Relaie une intrusion confirmée vers security/alert_manager.py si
        disponible (voir robot.security["alert_manager"]), sans bloquer si
        ce module n'est pas encore branché.
        """
        alert_manager = robot.security.get("alert_manager") if hasattr(robot, "security") else None
        if alert_manager is not None and hasattr(alert_manager, "notify"):
            alert_manager.notify(robot, confidence=1.0)

    def resume_si_possible(self, robot):
        """
        Lève l'arrêt sûr si les conditions redeviennent nominales. À
        appeler explicitement (pas de reprise automatique dans check()) :
        une reprise doit être une décision consciente de la boucle
        d'intégration, pas un comportement caché du safety manager.
        """
        if self.etat != EtatSurete.ARRET_SUR:
            robot.resume()
            return True
        return False

    def reinitialiser(self):
        """Remet le moniteur à l'état nominal (ex : nouvel essai)."""
        self.etat = EtatSurete.NOMINAL
        self._echecs_chemin_consecutifs = 0
        self.journal.clear()
