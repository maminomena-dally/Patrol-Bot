# 🎯 Role 5 — Experimentation, Surete et Qualite : Suivi de travail
# Tino — M2 SDIA

> **Derniere mise a jour** : Phase 4 terminee (SafetyManager + campagne d'essais statistique)
> **Statut global** : 🟢 Phases 1-4 terminees, Phase 5 a faire (integration finale, en attente du reste de l'equipe)
> **Branche** : `feature/surete-experimentation`

> **Sources lues** :
> - ✅ Cadrage_Mini_Projet_Robotique_Mobile.pdf
> - ✅ Code source du depot (`safety/safety_manager.py` stub, `simulation/simulator.py`, `robot/robot.py`, `localization/localization.py`)
> - ✅ ROLE3_WORKFLOW.md (Koja) — section "Guide pour Role 5 (Tino)"

---

## 📌 Exigences OFFICIELLES pour le Role 5

### Ce qui est attendu (section 4 "Architecture logicielle" et section 5 "Metriques" du cadrage)

D'apres le cadrage, l'etage **"Garantir"** de la boucle de navigation revient a ce role :
1. **Surveillance de la coherence des capteurs et de l'incertitude de localisation**
2. **Arret sur si necessaire** (aucun chemin trouve, position estimee trop incertaine)
3. **Journalisation complete a chaque pas de temps**, pour permettre le rejeu d'un essai
4. **Campagne d'essais** : au moins 10 essais nominaux + au moins 3 cas limites, repetes pour A* et RRT
5. **Metriques** : taux de succes, temps de mission, longueur du trajet, distance min aux obstacles, erreur de localisation, temps de replanification

### Difference importante avec le cadrage initial

Le cadrage prevoit une equipe de 5 (avec Kojy sur perception/localisation). **Kojy a ete retiree de l'equipe** en cours de route — l'equipe est maintenant a 4 (Malala, Koja, dally, Tino). Consequence directe sur ce role : le cas limite **"perte temporaire d'une balise pendant la replanification"** ne peut pas etre teste tant que personne ne reprend le module de localisation/balises. Voir section "Ce qui reste a faire" plus bas.

Autre ecart : le depot contient un dossier `security/` (intrusion, alertes, sirene) qui n'existe pas dans les 5 roles d'origine du cadrage — un role "Securite" non attribue depuis le depart de Kojy.

---

## Checklist principale

### Phase 1 — Setup & Comprehension ✅ TERMINE
- [x] Recuperer le depot, comprendre `Robot`, `Simulator`, les points d'extension (`sim.on_safety`, etc.)
- [x] Lire le stub `safety/safety_manager.py` (deja tres documente par l'equipe)
- [x] Verifier l'interface reelle de `Robot` (`emergency_stop()`, `resume()`, `robot.time`, `robot.security`)
- [x] Verifier l'interface de `Localizer.uncertainty` (float en metres)
- [x] Creer la branche `feature/surete-experimentation`

### Phase 2 — SafetyManager ✅ TERMINE
- [x] Implementer `safety/safety_manager.py` (classe `SafetyManager`)
  - [x] Machine a etats `NOMINAL / ALERTE / ARRET_SUR`
  - [x] Detection "aucun chemin valide" (echecs de replanification consecutifs, seuil configurable)
  - [x] Detection "localisation trop incertaine" (seuil configurable, `config.LOCALIZATION_UNCERTAINTY_MAX`)
  - [x] Detection "capteur critique indisponible" (aucune mesure recue -> arret par prudence)
  - [x] Relai d'une intrusion confirmee vers `security/alert_manager.py` (optionnel, ne bloque pas si absent)
  - [x] Journal des transitions d'etat (base pour le rejeu et le rapport)
  - [x] `resume_si_possible()` : reprise **toujours explicite**, jamais automatique dans `check()`
- [x] Tests unitaires (`tests/test_safety.py`) — 13 tests
- **Commit** : `feat(safety): implémente SafetyManager (arrêt sûr, journal, reprise)`

### Phase 3 — Tests & validation ✅ TERMINE
- [x] 13 tests unitaires du SafetyManager (nominal, localisation, planification, capteur indisponible, journal, reprise, intrusion)
- [x] Verification de non-regression : 74/74 tests du depot passent (safety + tous les autres roles)

### Phase 4 — Campagne d'essais statistique ✅ TERMINE
- [x] Etendre `scenario_replanification()` (Koja) avec 2 parametres optionnels (`unexpected_obstacle`, `obstacle_time`) — defauts inchanges, verifie sans regression
- [x] Ajouter le champ `path_found_after_replan` aux metriques de Koja, pour distinguer explicitement "aucun chemin trouve" d'un simple timeout
- [x] Creer `experiments/campagne_essais.py` :
  - [x] 10 essais nominaux (position et instant d'obstacle imprevu variables, deterministes)
  - [x] Cas limite 1 : obstacle sur le chemin le plus court
  - [x] Cas limite 2 : couloir totalement bloque (aucun detour possible)
  - [x] Branchement du `SafetyManager` sur le cas limite 2 : verifie que `ARRET_SUR` se declenche bien quand `planner.plan()` ne trouve rien
  - [x] Agregation (taux de succes, moyenne et ecart-type du temps de replanification)
  - [x] Export CSV detaille + resume texte dans `results/features_experimentation/`
- [x] Execute pour A* et RRT (12 essais chacun) — 92% de succes, arrets surs attendus/confirmes = 1/1 pour les deux algos
- **Commit** : `feat(experiments): campagne d'essais statistique (10 nominaux + 2 cas limites)`

### Phase 5 — Integration finale & rapport ⏸ A FAIRE
- [ ] Brancher reellement `sim.on_safety` dans la boucle d'integration finale (avec dally), avec les vraies valeurs de `localizer.uncertainty` et `planner.plan()` a chaque pas — pour l'instant le SafetyManager est teste isolement (tests unitaires + simulation dans la campagne), pas encore cable dans une boucle complete
- [ ] Cas limite 3 (perte de balise) : bloque tant que le module de localisation/balises n'a pas de responsable (voir "Ce qui reste a faire")
- [ ] Erreur de localisation (metrique du cahier des charges) : pas encore calculee dans la campagne — necessite de comparer position reelle vs position estimee sur la duree, une fois la localisation branchee dans la boucle (aujourd'hui les scenarios de Koja utilisent `get_true_pose()`, pas encore la pose estimee)
- [ ] Distance minimale aux obstacles sur les essais de replanification (deja calculee cote patrouille par Koja, a etendre cote replanification)
- [ ] Rediger la section 4/5 du rapport (methodologie de la campagne, resultats, analyse des cas limites, limites connues)
- [ ] Decider avec le groupe qui reprend `sensors/lidar.py` (stub vide) — necessaire pour un `obstacle_distance` reel dans `SafetyManager.check()` (utilise pour l'instant uniquement dans les tests avec des valeurs simulees)

---

## Historique des commits

| Date | Commit | Fichiers | Tests |
|------|--------|---------|-------|
| J+X | `feat(safety): implémente SafetyManager (arrêt sûr, journal, reprise)` | `safety/safety_manager.py`, `tests/test_safety.py` | 74 pass (13 safety + 61 existants) |
| J+X | `feat(experiments): campagne d'essais statistique (10 nominaux + 2 cas limites)` | `experiments/campagne_essais.py` (nouveau), `experiments/run_experiments.py` (ajouts), `results/features_experimentation/` | 74 pass (aucune regression) |

---

## Journal de bord

### Jour 1 — Setup + SafetyManager
- **Fait** : Branche creee, lecture du code existant (stub deja tres documente par l'equipe, interfaces de `Robot` et `Localizer` deja claires). Implementation complete de `SafetyManager` avec machine a etats, 13 tests unitaires, 74/74 tests du depot passent.
- **Bloquants** : Aucun — toutes les interfaces necessaires (`robot.emergency_stop()`, `localizer.uncertainty`, `planner.plan()` retournant `[]`) etaient deja definies par les autres roles.
- **Decisions** :
  - Reprise de l'arret sur **toujours explicite** (`resume_si_possible()`), jamais automatique dans `check()`, pour eviter des oscillations arret/reprise non controlees
  - Seuil de tentatives de replanification avant arret = 3 (tolerance aux echecs transitoires, coherent avec la logique de replanification de Koja)
  - Intrusion confirmee geree en `no-op` si `security/alert_manager.py` n'est pas branche, pour ne jamais faire planter le systeme a cause d'un module pas encore pret

### Jour 2 — Campagne d'essais statistique
- **Fait** : Extension non-cassante de `scenario_replanification()` (Koja) pour parametrer position/instant de l'obstacle imprevu. Creation de `campagne_essais.py` : 10 essais nominaux + 2 cas limites testables, x2 algos, avec verification croisee du `SafetyManager` sur le cas "couloir bloque". Resultats agreges et exportes.
- **Bloquants** :
  - Le cas limite "perte de balise" n'est pas testable : depend du module de localisation, dont la responsable (Kojy) a ete retiree de l'equipe sans reprise du module
  - L'erreur de localisation (metrique du cahier des charges) n'est pas encore calculable : les scenarios actuels utilisent la pose reelle du robot (`get_true_pose()`), pas encore une pose estimee comparable
- **Decisions** :
  - Ajout du champ `path_found_after_replan` aux metriques de Koja plutot que de deviner l'echec a partir d'autres champs (plus fiable, plus lisible)
  - Positions/instants d'obstacle deterministes (pas de tirage aleatoire) pour garder les essais reproductibles d'une execution a l'autre

---

## Interface publique de mes modules

### SafetyManager

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

### Campagne d'essais

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

### Points d'attention pour l'integration (Dally)
- `SafetyManager.check()` doit etre appele **avant** `command_fn` dans la boucle (comme `on_safety` dans `simulation/simulator.py`), pour que `robot.emergency_stop()` prenne effet sur le pas courant
- Ne jamais appeler `robot.resume()` directement depuis un autre module — toujours passer par `resume_si_possible()`, pour garder la decision de reprise centralisee
- `obstacle_distance=None` declenche systematiquement un arret par prudence (capteur indisponible) — a garder en tete tant que `sensors/lidar.py` n'est pas implemente

---

## Notes de collaboration

### Interface avec les autres roles
- **Role 1 (Malala — Cinematique)** : ✅ Aucune dependance bloquante — `robot.emergency_stop()`/`resume()` deja disponibles et stables, utilises tels quels
- **Role 2 (Kojy — Perception/Localisation)** : ⚠️ Role retire de l'equipe. Son travail sur `Localizer.uncertainty` etait deja mergé avant son depart et reste utilisable, mais plus personne ne maintient ce module — bloque le cas limite "perte de balise" et le calcul de l'erreur de localisation
- **Role 3 (Koja — Planification/Commande)** : ✅ Interface stable (`planner.plan()` retourne `[]` si echec, `scenario_replanification()` etendu sans casser son code). Guide dedie tres complet laisse dans `ROLE3_WORKFLOW.md`
- **Role 4 (Dally — Simulation/Integration)** : ⏳ En attente — le branchement reel de `sim.on_safety` dans la boucle d'integration finale reste a faire ensemble
- **Equipe Securite (`security/`)** : ⏳ Sans responsable depuis le depart de Kojy. `SafetyManager` est concu pour fonctionner sans ce module (relai optionnel, `no-op` si absent), donc pas bloquant pour ce role, mais a trancher pour la livraison finale

### Convention Git
- Branche : `feature/surete-experimentation`
- Messages : `feat(safety): ...` / `feat(experiments): ...`
- Modifications hors de mon perimetre : uniquement des ajouts non-cassants a `experiments/run_experiments.py` (parametres optionnels, defauts inchanges, verifie sans regression)

### Fichiers modifies par Role 5
| Fichier | Role | Description |
|---------|------|--------------|
| `safety/safety_manager.py` | Modifie (stub -> implementation) | `SafetyManager` complet |
| `tests/test_safety.py` | Nouveau | 13 tests unitaires |
| `experiments/campagne_essais.py` | Nouveau | Campagne statistique (10 nominaux + 2 cas limites, x2 algos) |
| `experiments/run_experiments.py` | Modifie (ajouts) | 2 parametres optionnels + 1 champ de metrique, sur `scenario_replanification()` (code de Koja preserve) |
| `results/features_experimentation/` | Nouveau | Resultats agreges (CSV + resume texte) |
| `ROLE5_WORKFLOW.md` | Nouveau | Ce fichier |
