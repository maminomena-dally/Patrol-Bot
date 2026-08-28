"""
safety/safety_manager.py — Arrêt sûr, mode dégradé et supervision (Rôle Sûreté — Tino).

Surveille en continu l'état du système et déclenche robot.emergency_stop()
quand une situation critique est détectée (section 16 du cahier des charges) :

    Situation                          -> Action
    --------------------------------------------------------------
    Obstacle proche                    -> ralentir / évitement (planning, pas ici)
    Obstacle bloquant le chemin         -> replanifier (planning, pas ici)
    Aucun chemin valide                 -> arrêt sûr
    Localisation trop incertaine        -> arrêt sûr
    Capteur critique indisponible       -> mode dégradé (arrêt par prudence)
    Intrusion confirmée (INFO/WARNING)  -> surveillance (ALERTE), pas d'arrêt
    Intrusion confirmée (DANGER)        -> arrêt d'urgence
    Perte de supervision                -> continuer ou arrêt selon politique

Ce module ne pilote jamais le robot directement en dehors de
`robot.emergency_stop()` / `robot.resume()` : le ralentissement/évitement et
la replanification restent la responsabilité de planning/ et control/.
L'alarme sonore et la classification des intrusions restent la
responsabilité de security/ (Speaker, AlertManager).

Chaîne complète attendue (security/ implémenté par Koja) :

    IntrusionDetector.check(targets, t) -> (confirmed, alerts)
    AlertManager.update(alerts, t)      -> AlertEvent
        am.get_intrusion_confirmed()    -> intrusion_confirmed (ci-dessous)
        am.is_danger()                  -> intrusion_danger (ci-dessous)
    Speaker.update(am.should_alarm(), t)  -> alarme sonore (independant)

SafetyManager.check() est un consommateur PASSIF de ces informations : il
les REÇOIT en paramètres (comme localization_uncertainty ou obstacle_distance),
il ne va JAMAIS interroger ou notifier security/ lui-même. C'est à
l'appelant (boucle d'intégration, cf. main.py) de lire am.get_intrusion_confirmed()
et am.is_danger() puis de les passer à check().

Interface (branchée via `sim.on_safety` dans simulation/simulator.py) :

    safety_manager.check(robot, localization_uncertainty=None,
                          obstacle_distance=None, path_found=True,
                          intrusion_confirmed=False, intrusion_danger=False)
"""

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
        intrusion_danger: bool = False,
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
            intrusion_confirmed: True si security/alert_manager.py.AlertManager
                .get_intrusion_confirmed() est vrai (intrusion detectee,
                niveau INFO ou WARNING) -> surveillance renforcee (ALERTE),
                sans arreter le robot.
            intrusion_danger: True si AlertManager.is_danger() est vrai
                (intrusion critique, niveau DANGER) -> arret d'urgence,
                comme documente dans security/alert_manager.py.

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
        elif intrusion_danger:
            # Intrusion confirmee au niveau DANGER (security.AlertManager.is_danger())
            # : arret d'urgence, comme documente dans security/alert_manager.py
            # ("Interface avec SafetyManager : am.is_danger() -> bool (arret urgence)").
            self.etat = EtatSurete.ARRET_SUR
            raison = "intrusion_danger"
        elif localisation_incertaine:
            self.etat = EtatSurete.ARRET_SUR
            raison = "localisation_trop_incertaine"
        elif capteur_indisponible:
            self.etat = EtatSurete.ARRET_SUR
            raison = "capteur_critique_indisponible"
        elif intrusion_confirmed:
            # Intrusion detectee mais pas encore critique (INFO/WARNING) :
            # on reste vigilant sans arreter le robot -- Speaker/AlertManager
            # gerent l'alarme sonore independamment (security/speaker.py).
            self.etat = EtatSurete.ALERTE
            raison = "intrusion_surveillee"
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
