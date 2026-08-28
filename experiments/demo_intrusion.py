"""
experiments/demo_intrusion.py — Demonstration de la chaine complete de
securite (Role 5 - Tino). Personne n'avait encore construit de scenario
reliant les 3 modules de security/ (Koja) au SafetyManager (Tino) : ce
script est la premiere verification bout-en-bout de cette chaine.

Chaine testee (telle que documentee dans security/*.py) :

    IntrusionDetector.check(targets, t) -> (confirmed, alerts)
    AlertManager.update(alerts, t)      -> AlertEvent (NOMINAL/INFO/WARNING/DANGER)
        am.get_intrusion_confirmed()    -> intrusion_confirmed
        am.is_danger()                  -> intrusion_danger
    Speaker.update(am.should_alarm(), t)  -> alarme sonore (independant)
    SafetyManager.check(..., intrusion_confirmed=..., intrusion_danger=...)
        -> arret d'urgence si intrusion_danger

Scenario : le robot est immobile en poste de surveillance. Un intrus
simule s'approche progressivement (de 6m a 0.5m), traversant les 3
niveaux d'alerte (INFO -> WARNING -> DANGER). On verifie que :
  - l'alarme sonore (Speaker) se declenche au niveau DANGER (pas avant),
  - le SafetyManager passe en ARRET_SUR au meme moment,
  - le robot est bien arrete (robot.stopped == True).

LIAISON avec le reste du role : reutilise LidarSensor (obstacle_distance)
et Localizer (localization_uncertainty), memes modules que demo_safety.py
et campagne_localisation.py -- coherence complete de la chaine de capteurs.

Lancer avec :
    python -m experiments.demo_intrusion
"""

import os

import config
from robot.robot import Robot
from safety.safety_manager import SafetyManager, EtatSurete
from sensors.lidar import LidarSensor
from security.intrusion_detector import IntrusionDetector
from security.alert_manager import AlertManager
from security.speaker import Speaker
from experiments.run_experiments import _ensure_dir


# Robot en poste fixe, aucun obstacle physique alentour pour ce scenario
# (on isole volontairement le canal "intrusion" du canal "obstacle/planning").
POSTE_SURVEILLANCE = (10.0, 7.5, 0.0)


def rejouer_scenario_intrusion(verbose=True):
    robot = Robot(initial_pose=POSTE_SURVEILLANCE)
    sm = SafetyManager()
    lidar = LidarSensor(robot, obstacles=[])
    detector = IntrusionDetector(robot, known_obstacles=[])  # aucun obstacle connu a filtrer ici
    alert_manager = AlertManager()
    speaker = Speaker()

    dt = config.DT
    historique = []

    # L'intrus s'approche en ligne droite du robot, de 6m a 0.5m
    distance_depart, distance_fin = 6.0, 0.5
    duree = 12.0
    n_steps = int(duree / dt)

    for i in range(n_steps):
        t = i * dt
        progression = i / (n_steps - 1)
        distance_intrus = distance_depart + (distance_fin - distance_depart) * progression
        intrus_pos = (POSTE_SURVEILLANCE[0] + distance_intrus, POSTE_SURVEILLANCE[1])

        # -- Chaine de securite complete --
        confirmed, alerts = detector.check([intrus_pos], t)
        event = alert_manager.update(alerts, t)
        speaker_event = speaker.update(alert_manager.should_alarm(), t)

        # -- SafetyManager recoit (ne va JAMAIS chercher lui-meme) --
        etat = sm.check(
            robot,
            localization_uncertainty=0.05,
            obstacle_distance=lidar.min_distance(),
            path_found=True,
            intrusion_confirmed=alert_manager.get_intrusion_confirmed(),
            intrusion_danger=alert_manager.is_danger(),
        )

        historique.append({
            "t": round(t, 2),
            "distance_intrus": round(distance_intrus, 2),
            "niveau_alerte": event.level.value,
            "alarme_active": speaker_event["is_alarming"],
            "etat_surete": etat.name,
            "robot_stopped": robot.stopped,
        })

        if etat == EtatSurete.ARRET_SUR:
            if verbose:
                print(f"  t={t:.2f}s d_intrus={distance_intrus:.2f}m "
                      f"niveau={event.level.value} -> ARRET_SUR "
                      f"(alarme={speaker_event['is_alarming']})")
            break

        robot.step(dt)

    return historique, sm, alert_manager, speaker


def verifier_coherence(historique):
    """
    Verifie les proprietes attendues de la chaine (pas juste qu'elle
    tourne sans planter -- qu'elle fait la bonne chose) :
      1. L'alarme ne se declenche jamais avant le niveau DANGER
      2. Le SafetyManager n'arrete jamais le robot avant le niveau DANGER
      3. Au niveau DANGER, alarme ET arret sont bien actifs
    """
    problemes = []
    for ligne in historique:
        if ligne["alarme_active"] and ligne["niveau_alerte"] != "danger":
            problemes.append(f"t={ligne['t']}s : alarme active hors niveau DANGER "
                              f"({ligne['niveau_alerte']})")
        if ligne["robot_stopped"] and ligne["niveau_alerte"] not in ("danger",):
            problemes.append(f"t={ligne['t']}s : robot arrete hors niveau DANGER "
                              f"({ligne['niveau_alerte']})")

    derniere = historique[-1]
    if derniere["niveau_alerte"] == "danger":
        if not derniere["alarme_active"]:
            problemes.append("Niveau DANGER atteint mais alarme non active")
        if not derniere["robot_stopped"]:
            problemes.append("Niveau DANGER atteint mais robot non arrete")

    return problemes


def sauvegarder(historique, problemes, base_dir=None):
    if base_dir is None:
        base_dir = os.path.join(config.RESULTS_DIR, "features_experimentation")
    os.makedirs(base_dir, exist_ok=True)

    txt_path = os.path.join(base_dir, "resume_intrusion.txt")
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write("DEMO CHAINE DE SECURITE COMPLETE -- Role 5 (Tino)\n")
        f.write("IntrusionDetector -> AlertManager -> Speaker + SafetyManager\n")
        f.write("=" * 65 + "\n\n")
        f.write(f"Nombre de pas simules : {len(historique)}\n")
        f.write(f"Etat final : {historique[-1]['etat_surete']}\n")
        f.write(f"Niveau d'alerte final : {historique[-1]['niveau_alerte']}\n\n")

        f.write("Verification de coherence :\n")
        if problemes:
            for p in problemes:
                f.write(f"  [PROBLEME] {p}\n")
        else:
            f.write("  OK -- alarme et arret d'urgence declenches uniquement au niveau DANGER\n")

        f.write("\nTransitions de niveau d'alerte :\n")
        niveau_precedent = None
        for ligne in historique:
            if ligne["niveau_alerte"] != niveau_precedent:
                f.write(f"  t={ligne['t']:.2f}s  distance_intrus={ligne['distance_intrus']:.2f}m"
                        f"  -> niveau={ligne['niveau_alerte']}\n")
                niveau_precedent = ligne["niveau_alerte"]

    return txt_path


def main():
    print("Demo chaine de securite complete -- IntrusionDetector -> AlertManager -> "
          "Speaker + SafetyManager\n")
    historique, sm, alert_manager, speaker = rejouer_scenario_intrusion(verbose=True)
    problemes = verifier_coherence(historique)

    print()
    if problemes:
        print("PROBLEMES DE COHERENCE DETECTES :")
        for p in problemes:
            print(f"  - {p}")
    else:
        print("Chaine coherente : alarme et arret d'urgence declenches uniquement "
              "au niveau DANGER.")

    txt_path = sauvegarder(historique, problemes)
    print(f"\nResultats sauvegardes : {txt_path}")


if __name__ == "__main__":
    main()
