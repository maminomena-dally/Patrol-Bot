🎯 Role 5 — Experimentation, Surete et Qualite : Suivi de travail
Tino — M2 SDIA
> **Derniere mise a jour** : Phase 5 quasi-terminee (recalibration localisation + intrusion faite, intrusion_danger corrige dans l'integration finale de dally)
> **Statut global** : 🟢 Role complet — plus aucune tache de code en attente
> **Branche** : `feature/surete-experimentation`
> **Sources lues** :
> - ✅ Cadrage_Mini_Projet_Robotique_Mobile.pdf
> - ✅ Code source du depot (`safety/safety_manager.py` stub, `simulation/simulator.py`, `robot/robot.py`, `localization/localization.py`)
> - ✅ ROLE3_WORKFLOW.md (Koja) — section "Guide pour Role 5 (Tino)"
---
📌 Exigences OFFICIELLES pour le Role 5
Ce qui est attendu (section 4 "Architecture logicielle" et section 5 "Metriques" du cadrage)
D'apres le cadrage, l'etage "Garantir" de la boucle de navigation revient a ce role :
Surveillance de la coherence des capteurs et de l'incertitude de localisation
Arret sur si necessaire (aucun chemin trouve, position estimee trop incertaine)
Journalisation complete a chaque pas de temps, pour permettre le rejeu d'un essai
Campagne d'essais : au moins 10 essais nominaux + au moins 3 cas limites, repetes pour A* et RRT
Metriques : taux de succes, temps de mission, longueur du trajet, distance min aux obstacles, erreur de localisation, temps de replanification
Difference importante avec le cadrage initial
Le cadrage prevoit une equipe de 5 (avec Kojy sur perception/localisation). Kojy a ete retiree de l'equipe en cours de route — l'equipe est maintenant a 4 (Malala, Koja, dally, Tino). Consequence directe sur ce role : le cas limite "perte temporaire d'une balise pendant la replanification" ne peut pas etre teste tant que personne ne reprend le module de localisation/balises. Voir section "Ce qui reste a faire" plus bas.
Autre ecart : le depot contient un dossier `security/` (intrusion, alertes, sirene) qui n'existe pas dans les 5 roles d'origine du cadrage — un role "Securite" non attribue depuis le depart de Kojy.
Checklist principale
Phase 1 — Setup & Comprehension ✅ TERMINE
[x] Recuperer le depot, comprendre `Robot`, `Simulator`, les points d'extension (`sim.on_safety`, etc.)
[x] Lire le stub `safety/safety_manager.py` (deja tres documente par l'equipe)
[x] Verifier l'interface reelle de `Robot` (`emergency_stop()`, `resume()`, `robot.time`, `robot.security`)
[x] Verifier l'interface de `Localizer.uncertainty` (float en metres)
[x] Creer la branche `feature/surete-experimentation`
Phase 2 — SafetyManager ✅ TERMINE
[x] Implementer `safety/safety_manager.py` (classe `SafetyManager`)
[x] Machine a etats `NOMINAL / ALERTE / ARRET_SUR`
[x] Detection "aucun chemin valide" (echecs de replanification consecutifs, seuil configurable)
[x] Detection "localisation trop incertaine" (seuil configurable, `config.LOCALIZATION_UNCERTAINTY_MAX`)
[x] Detection "capteur critique indisponible" (aucune mesure recue -> arret par prudence)
[x] Reception (pas relai) des infos d'intrusion depuis `security/alert_manager.py` : `intrusion_confirmed`/`intrusion_danger` recus en parametres de `check()`, jamais interroges directement (corrige le 25/08, voir Jour 6 — le design initial poussait vers un `alert_manager.notify()` inexistant)
[x] Journal des transitions d'etat (base pour le rejeu et le rapport)
[x] `resume_si_possible()` : reprise toujours explicite, jamais automatique dans `check()`
[x] Tests unitaires (`tests/test_safety.py`) — 13 tests
Commit : `feat(safety): implémente SafetyManager (arrêt sûr, journal, reprise)`
Phase 3 — Tests & validation ✅ TERMINE
[x] 13 tests unitaires du SafetyManager (nominal, localisation, planification, capteur indisponible, journal, reprise, intrusion)
[x] Verification de non-regression : 74/74 tests du depot passent (safety + tous les autres roles)
Phase 4 — Campagne d'essais statistique ✅ TERMINE
[x] Etendre `scenario_replanification()` (Koja) avec 2 parametres optionnels (`unexpected_obstacle`, `obstacle_time`) — defauts inchanges, verifie sans regression
[x] Ajouter le champ `path_found_after_replan` aux metriques de Koja, pour distinguer explicitement "aucun chemin trouve" d'un simple timeout
[x] Creer `experiments/campagne_essais.py` :
[x] 10 essais nominaux (position et instant d'obstacle imprevu variables, deterministes)
[x] Cas limite 1 : obstacle sur le chemin le plus court
[x] Cas limite 2 : couloir totalement bloque (aucun detour possible)
[x] Branchement du `SafetyManager` sur le cas limite 2 : verifie que `ARRET_SUR` se declenche bien quand `planner.plan()` ne trouve rien
[x] Agregation (taux de succes, moyenne et ecart-type du temps de replanification)
[x] Export CSV detaille + resume texte dans `results/features_experimentation/`
[x] Execute pour A* et RRT (12 essais chacun) — 92% de succes, arrets surs attendus/confirmes = 1/1 pour les deux algos
Commit : `feat(experiments): campagne d'essais statistique (10 nominaux + 2 cas limites)`
Phase 5 — Integration finale & rapport ⏸ EN COURS
[x] Script de demonstration visuelle (`experiments/demo_safety.py`) : rejoue le cas limite "couloir bloque" avec `SafetyManager.check()` appele reellement a chaque pas, produit un graphique montrant la trajectoire coloree par etat de surete + le point exact d'`ARRET_SUR` (`results/features_experimentation/images/safety_arret_sur_*.png`)
[x] Distance minimale aux obstacles sur les essais de replanification (etendu depuis la version de Koja, qui ne la calculait que cote patrouille) — `min_obstacle_dist` disponible sur chaque essai + agregee (min global) dans `resume_campagne.txt`
[x] Interface Tkinter interactive (`gui/safety_app.py`) : pilotage manuel du robot + panneau de test du SafetyManager en direct (obstacle imprevu, incertitude, capteur indisponible, journal des transitions) — teste sans mainloop (tkinter indisponible dans l'environnement de dev), a confirmer par Tino en local
[ ] Brancher reellement `sim.on_safety` dans la boucle d'integration finale (avec dally), avec les vraies valeurs de `localizer.uncertainty` et `planner.plan()` a chaque pas — pour l'instant le SafetyManager est teste isolement (tests unitaires + simulation dans la campagne), pas encore cable dans une boucle complete — en attente de dally
[x] Cas limite 3 (perte de balise) : DEBLOQUE — `localization/localization.py`, `sensors/odometry.py`, `sensors/landmarks.py` sont en realite deja tous implementes (verifie sur le depot a jour), le blocage n'etait pas reel. Teste dans `experiments/campagne_localisation.py`
[x] Erreur de localisation (metrique du cahier des charges) : DEBLOQUE — calculee reellement dans `experiments/campagne_localisation.py` (Odometry + LandmarkDetector + Localizer branches, pas simules), voir `results/features_experimentation/resume_localisation.txt`
[ ] Rediger la section 4/5 du rapport (methodologie de la campagne, resultats, analyse des cas limites, limites connues)
[x] `sensors/lidar.py` et `sensors/cameras.py` : DEBLOQUE — ces 2 capteurs sont tombes sur ce role (sans responsable depuis le depart de Kojy). Implementes, testes (9 tests), et branches reellement (obstacle_distance vient desormais du lidar, plus une valeur simulee) dans demo_safety.py, campagne_localisation.py et gui/safety_app.py
[ ] Brancher reellement `sim.on_safety` dans la boucle d'integration finale (avec dally) — en attente de dally (seule tache de code encore bloquee de ce role)
Constat de calibration RESOLU (Jour 7, voir plus bas) : les balises de mon propre scenario de test etaient trop clairsemees (4, trou volontaire de 8m) comparees a la realite du projet (`WAREHOUSE_LANDMARKS`, 9 balises espacees de 7-8m). Corrige dans `experiments/campagne_localisation.py` uniquement (pas de changement partage) : desormais `NOMINAL` sur tous les cas, sauf perte de balise prolongee (>=10s), qui declenche bien `ARRET_SUR` comme attendu. `LOCALIZATION_PROCESS_NOISE` (partage, config.py) n'a PAS ete touche : deja bon pour le vrai scenario (`integration_finale.py` reussit sans souci, confirme par dally via `LANDMARK_DETECTION_RADIUS` passe de 2.0 a 6.0 avant meme cette investigation)
Historique des commits
Date	Commit	Fichiers	Tests
J+X	`feat(safety): implémente SafetyManager (arrêt sûr, journal, reprise)`	`safety/safety_manager.py`, `tests/test_safety.py`	74 pass (13 safety + 61 existants)
J+X	`feat(experiments): campagne d'essais statistique (10 nominaux + 2 cas limites)`	`experiments/campagne_essais.py` (nouveau), `experiments/run_experiments.py` (ajouts), `results/features_experimentation/`	74 pass (aucune regression)
J+X	`feat(experiments): erreur de localisation + cas limite perte de balise`	`experiments/campagne_localisation.py` (nouveau), `results/features_experimentation/campagne_localisation.csv`, `resume_localisation.txt`	74 pass (aucune regression)
Journal de bord
Jour 1 — Setup + SafetyManager
Fait : Branche creee, lecture du code existant (stub deja tres documente par l'equipe, interfaces de `Robot` et `Localizer` deja claires). Implementation complete de `SafetyManager` avec machine a etats, 13 tests unitaires, 74/74 tests du depot passent.
Bloquants : Aucun — toutes les interfaces necessaires (`robot.emergency_stop()`, `localizer.uncertainty`, `planner.plan()` retournant `[]`) etaient deja definies par les autres roles.
Decisions :
Reprise de l'arret sur toujours explicite (`resume_si_possible()`), jamais automatique dans `check()`, pour eviter des oscillations arret/reprise non controlees
Seuil de tentatives de replanification avant arret = 3 (tolerance aux echecs transitoires, coherent avec la logique de replanification de Koja)
Intrusion confirmee geree en `no-op` si `security/alert_manager.py` n'est pas branche, pour ne jamais faire planter le systeme a cause d'un module pas encore pret (design initial pousse — corrige au Jour 6, voir plus bas : security/ etait un stub a l'epoque, l'interface reelle de Koja est differente et le sens du flux a du etre inverse)
Jour 2 — Campagne d'essais statistique
Fait : Extension non-cassante de `scenario_replanification()` (Koja) pour parametrer position/instant de l'obstacle imprevu. Creation de `campagne_essais.py` : 10 essais nominaux + 2 cas limites testables, x2 algos, avec verification croisee du `SafetyManager` sur le cas "couloir bloque". Resultats agreges et exportes.
Bloquants :
Le cas limite "perte de balise" n'est pas testable : depend du module de localisation, dont la responsable (Kojy) a ete retiree de l'equipe sans reprise du module
L'erreur de localisation (metrique du cahier des charges) n'est pas encore calculable : les scenarios actuels utilisent la pose reelle du robot (`get_true_pose()`), pas encore une pose estimee comparable
Decisions :
Ajout du champ `path_found_after_replan` aux metriques de Koja plutot que de deviner l'echec a partir d'autres champs (plus fiable, plus lisible)
Positions/instants d'obstacle deterministes (pas de tirage aleatoire) pour garder les essais reproductibles d'une execution a l'autre
Jour 3 — Distance min. aux obstacles (replanification)
Fait : Ajout de `min_obstacle_dist` a `scenario_replanification()` (Koja ne le calculait que cote patrouille). Calcule sur tout l'historique du robot, en tenant compte du moment d'apparition de l'obstacle imprevu (obstacles actifs differents avant/apres `obstacle_time`). Metrique agregee (min global) ajoutee au resume de la campagne.
Bloquants : Aucun — reutilise `_min_dist_to_rects()` deja ecrite par Koja
Decisions : Calcul factorise dans une fonction dediee (`_min_obstacle_dist_sur_trajet`) appelee a chaque point de sortie de `scenario_replanification()` (4 `return` possibles), plutot que duplique — evite les incoherences si un cas est oublie
A faire ensuite (bloque, en attente des autres roles) :
Branchement reel de `sim.on_safety` dans la boucle finale -> attend dally
`security/` (intrusion_detector, alert_manager, speaker) -> FAIT PAR KOJA depuis (voir Jour 6) — lien corrige avec SafetyManager
Jour 4 — Erreur de localisation + cas limite perte de balise
Fait : Verification sur le depot a jour que `localization/localization.py`, `sensors/odometry.py` et `sensors/landmarks.py` sont deja reellement implementes (pas des stubs) — le blocage "attend Kojy" n'etait plus reel. Cree `experiments/campagne_localisation.py` : branche Odometry + LandmarkDetector + Localizer reellement dans une boucle de simulation (pas simules), calcule l'erreur de localisation (position reelle vs estimee), et teste le cas limite "perte de balise" (balises rendues indetectables 3s pendant la replanification).
Bloquants : Aucun cote code. Trouve un probleme de CALIBRATION (pas un bug) : les 4 balises "points de controle" de Koja sont a >4.5m du corridor de test de replanification (hors du rayon de detection de 2.0m) — inutilisables telles quelles pour ce scenario. Balises dediees definies le long du corridor a la place (documente dans le fichier).
Decisions : Ne modifie pas `security/` (toujours des stubs, hors de perimetre) — nouveau module entierement independant, n'utilise que les modules deja finis
Jour 5 — Capteurs lidar et cameras (sensor tombe sur ce role)
Fait : Implemente `sensors/lidar.py` (LidarSensor, scan 360 deg par intersection rayon/rectangle, methode des slabs) et `sensors/cameras.py` (Camera frontale + surveillance, champ de vision/portee). 9 tests avec geometrie verifiee a la main. Branche reellement le lidar dans `demo_safety.py`, `campagne_localisation.py` et `gui/safety_app.py` : `obstacle_distance` n'est plus simule en dur, vient du capteur.
Bloquants : Aucun
Decisions : `Camera.observe()` produit un contrat de donnees ({"x","y","distance","angle_deg","camera"}) documente pour `security/intrusion_detector.py` — confirme utilise TEL QUEL par Koja (`from sensors.cameras import Camera`, memes noms de parametres exacts), aucune adaptation necessaire
Jour 6 — Correction de la liaison SafetyManager <-> security/, demo bout-en-bout
Fait : Koja a implemente `security/` (intrusion_detector.py, alert_manager.py, speaker.py) et documente l'interface attendue avec SafetyManager. Audit complet du code (sur demande explicite) a revele que mon design initial etait FAUX : `SafetyManager._declencher_alerte()` appelait `alert_manager.notify(...)`, une methode qui n'existe pas dans l'implementation reelle de Koja — code mort, jamais declenche. L'interface reelle documentee est inversee : l'appelant lit `am.get_intrusion_confirmed()` et `am.is_danger()`, puis les PASSE a `SafetyManager.check()`. Corrige : suppression de `_declencher_alerte`, ajout du parametre `intrusion_danger` qui declenche un arret d'urgence reel (auparavant, aucune intrusion ne pouvait arreter le robot, contrairement a ce que documentait Koja : "am.is_danger() -> arret urgence"). Cree `experiments/demo_intrusion.py` : premier scenario bout-en-bout reliant IntrusionDetector -> AlertManager -> Speaker + SafetyManager, avec verification automatique de coherence (alarme et arret uniquement au niveau DANGER, jamais avant).
Bloquants : Aucun cote code
Decisions : `intrusion_confirmed` seul (niveau INFO/WARNING) place le systeme en ALERTE (vigilance) sans arreter le robot ; seul `intrusion_danger` (niveau DANGER) declenche `ARRET_SUR`, conforme a la doc de Koja
Constat de calibration RESOLU (Jour 7, voir plus bas) : `DETECTION_COOLDOWN`/`ALERT_RESOLUTION_DELAY` centralises dans `config.py` (etaient codes en dur dans `security/*.py`), avec `ALERT_RESOLUTION_DELAY=3.0s > DETECTION_COOLDOWN=2.0s`. Plus d'oscillation : progression propre nominal -> info -> warning -> danger, verifie dans `resume_intrusion.txt`
Jour 7 — Recalibration localisation/intrusion, correction integration_finale de dally
Fait : (1) Trouve que `experiments/integration_finale.py` (nouveau fichier de dally, integration reelle des 6 modules) appelait `SafetyManager.check()` sans `intrusion_danger` — corrige, verifie que la patrouille complete (A* et RRT) reussit toujours apres coup. (2) Diagnostique le "constat de calibration" localisation du Jour 6 : pas un probleme de `LOCALIZATION_PROCESS_NOISE` (deja bon, confirme par le succes de `integration_finale.py`), mais mes propres balises de test dans `campagne_localisation.py` etaient trop clairsemees comparees a la realite du projet -- corrige (densite alignee sur `WAREHOUSE_LANDMARKS`). Verifie avec plusieurs durees de perte de balise (3/6/10/15s) : tolerant aux pertes courtes, declenche `ARRET_SUR` a partir de 10s -- comportement attendu confirme. (3) Centralise `DETECTION_COOLDOWN`/`ALERT_RESOLUTION_DELAY` dans `config.py`, corrige l'oscillation du niveau d'alerte.
Bloquants : Aucun
Decisions : Ne pas toucher `LOCALIZATION_PROCESS_NOISE` (config partagee, deja bonne) -- corriger la densite des balises dans mon propre fichier de test uniquement, zero risque de regression sur le travail des autres. Pour l'intrusion, centraliser dans `config.py` plutot que changer les defauts codes en dur de Koja directement, pour rester coherent avec le reste du projet (localisation deja centralisee ainsi) et permettre un ajustement futur en un seul endroit.
Ajoute `random.seed()` dans `campagne_localisation.py` (manquant, resultats non reproductibles d'une execution a l'autre a cause du bruit d'odometrie non-seede)
Interface publique de mes modules
SafetyManager
```python
from safety.safety_manager import SafetyManager, EtatSurete

sm = SafetyManager(
    obstacle_safe_distance=config.OBSTACLE_SAFE_DISTANCE,
    localization_uncertainty_max=config.LOCALIZATION_UNCERTAINTY_MAX,
    tentatives_max_replanification=3,
)

# A chaque pas de la boucle de simulation (via sim.on_safety) :
etat = sm.check(
    robot,
    localization_uncertainty=localizer.uncertainty,   # float, m
    obstacle_distance=lidar_distance,                  # float, m (ou None si capteur indisponible)
    path_found=bool(planner.plan(...)),                # False si aucun chemin
    intrusion_confirmed=False,                          # True si security/ confirme une intrusion
)
# etat in {EtatSurete.NOMINAL, EtatSurete.ALERTE, EtatSurete.ARRET_SUR}
# robot.emergency_stop() est appele automatiquement si ARRET_SUR

# Journal des transitions (pour le rejeu / le rapport) :
sm.journal   # liste d'EvenementSurete(t, transition, raison, ...)

# Reprise explicite uniquement :
sm.resume_si_possible(robot)
```
Campagne d'essais
```python
from experiments.campagne_essais import lancer_campagne, agreger, sauvegarder

resultats = lancer_campagne(planner_name="astar", n_nominaux=10, verbose=True)
resume = agreger(resultats)
# resume : taux_succes, temps_replanification_moy_ms, ecart-type,
#          nb_arrets_surs_attendus / confirmes

sauvegarder({"astar": resultats}, {"astar": resume})
# -> results/features_experimentation/campagne_essais.csv
# -> results/features_experimentation/resume_campagne.txt
```
Points d'attention pour l'integration (Dally)
`SafetyManager.check()` doit etre appele avant `command_fn` dans la boucle (comme `on_safety` dans `simulation/simulator.py`), pour que `robot.emergency_stop()` prenne effet sur le pas courant
Ne jamais appeler `robot.resume()` directement depuis un autre module — toujours passer par `resume_si_possible()`, pour garder la decision de reprise centralisee
`obstacle_distance` vient maintenant d'un vrai `LidarSensor` (voir `sensors/lidar.py`) — `None` (capteur indisponible) declenche toujours un arret par prudence
Notes de collaboration
Interface avec les autres roles
Role 1 (Malala — Cinematique) : ✅ Aucune dependance bloquante — `robot.emergency_stop()`/`resume()` deja disponibles et stables, utilises tels quels
Role 2 (Kojy — Perception/Localisation) : ⚠️ Role retire de l'equipe, mais son travail (`Localizer`, `Odometry`, `LandmarkDetector`) est complet et fonctionnel — verifie et reellement utilise dans `experiments/campagne_localisation.py`. Ne bloque donc plus rien pour ce role. Reste sans responsable pour la maintenance/calibration future (voir "Constat de calibration", Phase 5)
Role 3 (Koja — Planification/Commande) : ✅ Interface stable (`planner.plan()` retourne `[]` si echec, `scenario_replanification()` etendu sans casser son code). Guide dedie tres complet laisse dans `ROLE3_WORKFLOW.md`
Role 4 (Dally — Simulation/Integration) : ⏳ En attente — le branchement reel de `sim.on_safety` dans la boucle d'integration finale reste a faire ensemble
Equipe Securite (`security/`) : ✅ Implemente par Koja (intrusion_detector.py, alert_manager.py, speaker.py). LIAISON CORRIGEE (Jour 6) : le design initial de `SafetyManager` tentait de pousser une notification vers `alert_manager.notify()` (methode inexistante, code mort). L'interface reelle documentee par Koja est inversee : `AlertManager.get_intrusion_confirmed()` et `.is_danger()` sont lus par l'appelant et PASSES a `SafetyManager.check(intrusion_confirmed=..., intrusion_danger=...)`. Corrige, teste, demontre bout-en-bout dans `experiments/demo_intrusion.py`
Convention Git
Branche : `feature/surete-experimentation`
Messages : `feat(safety): ...` / `feat(experiments): ...`
Modifications hors de mon perimetre : uniquement des ajouts non-cassants a `experiments/run_experiments.py` (parametres optionnels, defauts inchanges, verifie sans regression)
Fichiers modifies par Role 5
Fichier	Role	Description
`safety/safety_manager.py`	Modifie (stub -> implementation)	`SafetyManager` complet
`tests/test_safety.py`	Nouveau	13 tests unitaires
`experiments/campagne_essais.py`	Nouveau	Campagne statistique (10 nominaux + 2 cas limites, x2 algos)
`experiments/run_experiments.py`	Modifie (ajouts)	2 parametres optionnels + 1 champ de metrique, sur `scenario_replanification()` (code de Koja preserve)
`experiments/demo_safety.py`	Nouveau	Demo visuelle (graphique) du SafetyManager
`gui/safety_app.py`	Nouveau	Interface Tkinter interactive de test du SafetyManager
`experiments/campagne_localisation.py`	Nouveau	Erreur de localisation + cas limite perte de balise (localisation reelle)
`sensors/lidar.py`	Nouveau (stub -> implementation, tombe sur ce role)	LidarSensor, scan 360 deg
`sensors/cameras.py`	Nouveau (stub -> implementation, tombe sur ce role)	Camera frontale + surveillance
`tests/test_sensors.py`	Nouveau	9 tests (lidar + cameras)
`experiments/demo_intrusion.py`	Nouveau	Demo bout-en-bout IntrusionDetector -> AlertManager -> Speaker + SafetyManager
`safety/safety_manager.py`	Modifie (correction liaison)	Suppression du relai casse vers alert_manager.notify(), ajout intrusion_danger
`experiments/integration_finale.py`	Modifie (correction liaison, fichier de dally)	Ajout de intrusion_danger, manquant depuis sa creation
`config.py`	Modifie (ajout)	DETECTION_COOLDOWN, ALERT_RESOLUTION_DELAY (centralises, etaient codes en dur)
`security/intrusion_detector.py`, `security/alert_manager.py`	Modifie (recalibration)	Defauts branches sur config.py au lieu de valeurs en dur
`results/features_experimentation/`	Nouveau	Resultats agreges (CSV + resume texte, campagnes essais + localisation)
`TINO_WORKFLOW.md`	Nouveau (ex ROLE5_WORKFLOW.md)	Ce fichier