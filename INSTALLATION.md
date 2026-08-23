# Guide d'installation — Robot de patrouille de sécurité

Ce guide permet à n'importe quel membre de l'équipe de faire tourner le
projet en quelques minutes, sur Windows, macOS ou Linux.

## 1. Prérequis

- **Python 3.10 ou plus récent** (aucune autre dépendance obligatoire
  pour le module Système/Cinématique).
- Vérifier votre version :

```bash
python3 --version
```

Si Python n'est pas installé : https://www.python.org/downloads/
(cocher "Add Python to PATH" sous Windows lors de l'installation).

## 2. Récupérer le projet

Si le projet est sur un dépôt Git partagé par l'équipe :

```bash
git clone <url-du-depot>
cd robot_patrouille
```

Sinon, décompressez simplement l'archive `robot_patrouille.zip` fournie et
placez-vous dans le dossier :

```bash
cd robot_patrouille
```

## 3. Créer un environnement virtuel (recommandé)

Un environnement virtuel isole les dépendances du projet de votre Python
système — chaque membre de l'équipe doit en créer un.

**Linux / macOS :**
```bash
python3 -m venv venv
source venv/bin/activate
```

**Windows (PowerShell) :**
```powershell
python -m venv venv
venv\Scripts\Activate.ps1
```

**Windows (invite de commandes classique) :**
```cmd
python -m venv venv
venv\Scripts\activate.bat
```

Une fois activé, votre invite de commande doit afficher `(venv)` au début
de la ligne.

## 4. Installer les dépendances

```bash
pip install -r requirements.txt
```

Le module Système/Cinématique n'a besoin d'aucune bibliothèque externe
(seulement la bibliothèque standard Python). Le fichier `requirements.txt`
contient des dépendances *optionnelles*, commentées, pour les prochains
modules (numpy, matplotlib, pytest) — décommentez-les au fur et à mesure
que votre binôme en a besoin.

## 5. Vérifier que tout fonctionne

**a) Lancer la démonstration du module Système/Cinématique :**

```bash
python main.py
```

Vous devez voir s'afficher 4 scénarios (ligne droite, virage, saturation
des limites, arrêt sûr) et la confirmation qu'un fichier de log a été
exporté dans `logs/robot_state_log.csv`.

**b) Lancer les tests unitaires :**

```bash
python -m unittest discover -s tests -v
```

Résultat attendu : `Ran 17 tests ... OK` (aucun `FAILED`).

Si `pytest` est installé (voir `requirements.txt`), vous pouvez aussi
utiliser :

```bash
pytest tests/ -v
```

## 6. Lancer un scénario d'expérience

```bash
python experiments/run_experiments.py
```

Le résultat est exporté dans `results/scenario_avancer_tourner.csv`.

## 7. Lancer l'interface graphique 2D

```bash
python -m gui.app
```

Une fenêtre s'ouvre avec la visualisation du robot et les commandes
(curseurs v/ω, arrêt d'urgence, reprise, réinitialisation, export du log).
Fermez simplement la fenêtre pour quitter.

Pour rejouer un log déjà enregistré :

```bash
python -m gui.replay logs/robot_state_log.csv
```

**Sous Linux**, si vous obtenez `ModuleNotFoundError: No module named 'tkinter'`,
installez le paquet système correspondant à votre distribution, par exemple :

```bash
sudo apt install python3-tk        # Debian / Ubuntu
sudo dnf install python3-tkinter   # Fedora
```

Sous Windows et macOS, Tkinter est normalement déjà inclus avec Python.

**Si vous travaillez sur une machine sans écran** (ex : machine distante,
conteneur, WSL sans serveur X), l'interface graphique ne pourra pas
s'afficher — utilisez plutôt `main.py` (console) ou `experiments/run_experiments.py`,
qui fonctionnent partout.

## 8. Problèmes fréquents

| Symptôme | Cause probable | Solution |
|---|---|---|
| `command not found: python3` | Python non installé ou pas dans le PATH | Réinstaller Python en cochant "Add to PATH" |
| `ModuleNotFoundError: No module named 'config'` | Le script n'est pas lancé depuis la racine du projet | Se placer dans `robot_patrouille/` avant de lancer une commande |
| `(venv)` n'apparaît pas après activation | Environnement virtuel non activé | Relancer la commande d'activation de l'étape 3 |
| Les tests échouent après modification de `config.py` | Des valeurs comme `V_MAX`/`OMEGA_MAX` ont changé | Vérifier que les tests concernés utilisent bien `config.V_MAX` / `config.OMEGA_MAX` et non des valeurs codées en dur |

## 9. Pour la suite (autres binômes)

Une fois votre module implémenté dans son dossier (`sensors/`,
`localization/`, `planning/`, `control/`, `security/`, `safety/`) :

1. Ajoutez vos propres tests dans `tests/test_<votre_module>.py`.
2. Si votre module a besoin d'une nouvelle dépendance, ajoutez-la dans
   `requirements.txt` (décommentez ou ajoutez une ligne) et prévenez
   l'équipe.
3. Relancez `python -m unittest discover -s tests -v` pour vérifier que
   vous n'avez rien cassé dans les autres modules.
4. Voir `README.md`, section 4, pour brancher votre module dans la boucle
   de simulation (`simulation/simulator.py`) sans modifier le code
   existant.
