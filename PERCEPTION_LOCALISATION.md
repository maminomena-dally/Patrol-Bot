# Perception / Localisation — Documentation du module

Ce document décrit le travail réalisé pour le rôle **Perception / Localisation**
du mini-projet *Robot de patrouille de sécurité*. Il couvre :

- `sensors/odometry.py` — encodeurs de roues simulés
- `sensors/landmarks.py` — détecteur de balises simulé
- `localization/localization.py` — fusion odométrie + balises

`sensors/lidar.py` (binôme Perception/Navigation) et `sensors/cameras.py`
(binôme Vision/Sécurité) ne sont **pas** couverts ici : ce sont d'autres
sous-équipes d'après l'en-tête de ces fichiers.

Tout ce document est basé sur le code réellement écrit dans ce dépôt et sur
des exécutions réelles (tests unitaires + scénario manuel), pas sur des
hypothèses.

---

## 1. Vue d'ensemble du rôle

Le module "Système et Cinématique" (`robot/`, `simulation/`) simule le
déplacement **réel** du robot (`robot.get_true_pose()`). Le rôle
Perception/Localisation consiste à reconstituer une estimation de cette
pose **sans jamais lire cette vérité terrain**, exactement comme un robot
physique doit le faire : il ne "sait" pas où il est, il doit l'estimer à
partir de ses capteurs.

Deux capteurs simulés produisent les mesures :

- `Odometry` lit `robot.get_wheel_velocities()` et simule des encodeurs de
  roues bruités.
- `LandmarkDetector` lit `robot.get_true_pose()` (autorisé, car c'est un
  capteur — voir règle d'or dans `README.md`) et simule la détection
  bruitée de balises de position connue.

`Localizer` combine ces deux mesures dans une boucle **prédire → mesurer →
corriger** et expose `estimated_pose` (la pose estimée) et `uncertainty`
(l'incertitude accumulée), sans jamais toucher à `robot.pose`.

## 2. Concepts théoriques

### 2.1 Pourquoi l'odométrie seule dérive

L'odométrie intègre à chaque pas de temps un petit déplacement mesuré par
bruit des encodeurs :

```
x(k+1) = x(k) + d_center * cos(theta_moyen)
y(k+1) = y(k) + d_center * sin(theta_moyen)
theta(k+1) = theta(k) + d_theta
```

Chaque `d_left`/`d_right` porte une petite erreur de mesure (glissement,
calibration). Comme la pose est **intégrée** (chaque nouvelle estimation
part de la précédente), ces petites erreurs s'accumulent sans jamais se
corriger toutes seules — l'erreur de position croît avec la distance
parcourue, et une erreur sur `theta` fait dériver toute la trajectoire
future en `x, y`.

### 2.2 Pourquoi une seule mesure de distance ne suffit pas en 2D

Connaître uniquement une distance `d` à une balise de position connue
place le robot n'importe où sur un **cercle** de rayon `d` autour de la
balise : c'est sous-déterminé. Il faut une information supplémentaire pour
lever l'ambiguïté :

- soit une **deuxième balise** (intersection de deux cercles → 2 points
  possibles) puis une **troisième** (trilatération → point unique) ;
- soit un **angle** en plus de la distance (comme fait ici :
  `LandmarkDetector.detect()` retourne `distance` ET `angle`), ce qui fixe
  un point unique dès une seule balise.

### 2.3 Logique de correction utilisée dans ce projet

`Localizer` n'implémente pas un EKF complet mais un **recalage pondéré**
("complementary filter"), comme suggéré par le cours pour une première
version :

1. **predict()** avance la pose avec le modèle odométrique du robot
   différentiel, et **augmente** `uncertainty` proportionnellement à la
   distance/rotation parcourue (plus on avance sans recalage, moins on est
   sûr).
2. **correct()** convertit chaque mesure de balise en une estimation de
   position du robot, moyenne ces estimations si plusieurs balises sont
   visibles, puis fusionne ce résultat avec la position prédite par une
   moyenne pondérée. Le poids dépend du rapport entre l'incertitude
   accumulée (`uncertainty`) et le bruit de mesure des balises
   (`measurement_noise`) : plus l'odométrie a dérivé, plus on fait
   confiance aux balises. La correction **diminue** ensuite `uncertainty`.

### 2.4 Rôle de chaque capteur simulé et pourquoi il est bruité

| Capteur | Fichier | Ce qu'il simule | Pourquoi bruité |
|---|---|---|---|
| Encodeurs de roues | `sensors/odometry.py` | Distance parcourue par chaque roue sur un pas `dt` | Un vrai encodeur subit du glissement des roues et des erreurs de calibration — sans bruit, l'odométrie serait parfaite et il n'y aurait rien à corriger |
| Balises | `sensors/landmarks.py` | Détection d'un point fixe connu, avec mesure `(distance, angle)`, uniquement si à portée (`detection_radius`) | Un vrai capteur de balise a une précision de mesure limitée ; même la source de correction n'est pas exacte, seulement plus fiable dans la durée que l'odométrie |

## 3. Explication du code, fonction par fonction

### `sensors/odometry.py`

**`Odometry.__init__(self, robot, noise_std=config.ODOMETRY_NOISE_STD)`**
([sensors/odometry.py:23](sensors/odometry.py#L23)) — stocke une référence
au robot et l'écart-type du bruit gaussien à appliquer (par défaut lu dans
`config.py`, pas codé en dur).

**`Odometry.read(self, dt)`** ([sensors/odometry.py:27-40](sensors/odometry.py#L27-L40)) :
1. lit les vitesses de roues courantes via `robot.get_wheel_velocities()`
   (jamais `robot.pose`) ;
2. intègre chaque vitesse sur `dt` pour obtenir la distance parcourue par
   chaque roue (`vL * dt`, `vR * dt`) ;
3. ajoute un bruit gaussien indépendant sur chaque roue
   (`random.gauss(0.0, noise_std)`) ;
4. retourne `(d_left, d_right)` en mètres.

### `sensors/landmarks.py`

**`LandmarkDetector.__init__(...)`**
([sensors/landmarks.py:27-35](sensors/landmarks.py#L27-L35)) — stocke la
liste des balises (`{"id", "x", "y"}`), le rayon de détection et les
écarts-types de bruit sur distance et angle (par défaut depuis
`config.py`).

**`LandmarkDetector.detect(self)`**
([sensors/landmarks.py:37-68](sensors/landmarks.py#L37-L68)) :
1. lit la vraie pose du robot (`get_true_pose()`) — autorisé ici puisque
   c'est le capteur qui simule la mesure physique ;
2. pour chaque balise, calcule la distance et l'angle **vrais** via
   `math.hypot` et `math.atan2` (l'angle est calculé relatif au cap
   `theta` du robot, donc exprimé dans son propre repère) ;
3. ignore la balise si elle est hors de `detection_radius` (`continue`) ;
4. sinon, ajoute un bruit gaussien indépendant à la distance et à l'angle,
   et ajoute la mesure bruitée `{"id", "x", "y", "distance", "angle"}` à
   la liste retournée. `x`/`y` de la balise, elles, sont connues
   exactement (ce sont des points fixes de la carte, pas des mesures).

### `localization/localization.py`

**`Localizer.__init__(...)`**
([localization/localization.py:30-38](localization/localization.py#L30-L38))
— initialise `estimated_pose` à partir d'une pose de départ (fournie une
seule fois, pas lue depuis `robot.get_true_pose()` à chaque pas), et
`uncertainty = 0.0`.

**`Localizer.predict(self, d_left, d_right)`**
([localization/localization.py:40-64](localization/localization.py#L40-L64)) :
1. calcule le déplacement moyen `d_center` et la rotation `d_theta` à
   partir des deltas de roues (mêmes formules que le modèle du robot
   différentiel utilisé par `robot/kinematics.py`) ;
2. avance `x, y` en utilisant l'angle **moyen** entre avant et après
   rotation (`mid_theta`) plutôt que l'angle avant seul — plus précis
   quand le robot tourne en avançant ;
3. met à jour `estimated_pose` ;
4. **augmente** `uncertainty` proportionnellement à la distance et à la
   rotation parcourues (`process_noise * (|d_center| + |d_theta|)`) — modélise
   la dérive expliquée en 2.1.

**`Localizer.correct(self, landmark_measurements)`**
([localization/localization.py:66-102](localization/localization.py#L66-L102)) :
1. si aucune balise détectée, ne fait rien (le robot continue sur sa seule
   prédiction) ;
2. pour chaque mesure, calcule la position du robot qu'elle implique
   (`x_balise - distance * cos(theta_estimé + angle)`, idem en y) ;
3. moyenne ces estimations entre balises ;
4. calcule un poids de fusion `uncertainty / (uncertainty + measurement_noise)`,
   borné entre 0 et 1 ;
5. mélange la position prédite et la position issue des balises selon ce
   poids, et **diminue** `uncertainty` en conséquence.

## 4. Guide d'utilisation / test

### Lancer les tests

```bash
python -m unittest discover -s tests -v
```

**Résultat obtenu à l'exécution réelle : 29 tests, tous passants**
(17 tests déjà existants sur `robot/kinematics.py`/`robot/robot.py` +
12 nouveaux tests dans `tests/test_perception_localization.py` couvrant
`Odometry`, `LandmarkDetector`, `Localizer` et un scénario d'intégration
predict/correct sans bruit).

### Utiliser les modules dans un scénario

```python
import config
from robot.robot import Robot
from sensors.odometry import Odometry
from sensors.landmarks import LandmarkDetector
from localization.localization import Localizer

robot = Robot()
robot.set_velocity(0.3, 0.15)

odom = Odometry(robot)
landmarks = [{"id": 0, "x": 3.0, "y": 1.0}, {"id": 1, "x": 1.0, "y": -2.0}]
detector = LandmarkDetector(robot, landmarks)
localizer = Localizer(initial_pose=robot.get_true_pose())

dt = config.DT
for _ in range(200):
    d_left, d_right = odom.read(dt)
    localizer.predict(d_left, d_right)
    localizer.correct(detector.detect())
    robot.step(dt)
```

Ce scénario a été exécuté réellement (bruit par défaut de `config.py`,
10 secondes de simulation, 200 pas) :

```
Vérité terrain : x=2.002 y=1.851 theta=1.500
Estimation     : x=1.880 y=1.634 theta=1.704
Erreur de position : 0.2490 m
Incertitude finale (Localizer.uncertainty) : 0.0134
Corrections appliquées : 121 / 200 pas (balises détectées)
```

L'erreur finale (0.25 m) reste sous le seuil
`config.LOCALIZATION_UNCERTAINTY_MAX = 0.5` m qui déclencherait un arrêt
sûr côté `safety/safety_manager.py`. L'écart sur `theta` (1.704 rad estimé
contre 1.500 rad réel) est plus visible : voir limite connue en section 6.

### Brancher dans la boucle de simulation

`simulation/Simulator` expose `sim.on_perceive` et `sim.on_localize`
(voir `simulation/simulator.py`) pour connecter ces modules sans modifier
le cœur cinématique.

## 5. Paramètres de réglage disponibles (`config.py`)

| Paramètre | Valeur par défaut | Effet si on l'augmente |
|---|---|---|
| `ODOMETRY_NOISE_STD` | 0.01 m | Plus de bruit sur chaque mesure de roue → dérive plus rapide entre deux recalages |
| `LANDMARK_DETECTION_RADIUS` | 2.0 m | Balises détectées plus loin → recalages plus fréquents, mais aussi utilisables sur des cartes plus grandes/balises plus espacées |
| `LANDMARK_NOISE_STD_DISTANCE` | 0.05 m | Mesure de distance aux balises moins précise → correction moins fiable |
| `LANDMARK_NOISE_STD_ANGLE` | 0.03 rad | Mesure d'angle vers les balises moins précise → position corrigée moins précisément, surtout loin de la balise |
| `LOCALIZATION_PROCESS_NOISE` | 0.05 | `uncertainty` grandit plus vite à chaque `predict()` → le filtre fait confiance aux balises plus tôt |
| `LOCALIZATION_MEASUREMENT_NOISE` | 0.1 m | Plus grand → le filtre fait moins confiance aux balises même quand `uncertainty` est élevée (poids de fusion plus faible) |

Ces six paramètres n'existaient pas dans `config.py` avant ce travail : ils
ont été ajoutés (section "Perception / Localisation") pour respecter la
règle du projet de centraliser les constantes plutôt que de les coder en
dur dans les modules.

## 6. Limites connues

- **`theta` n'est pas corrigé par les balises.** `correct()` ne recale que
  `x` et `y` ; `estimated_pose.theta` reste uniquement piloté par
  l'intégration odométrique dans `predict()`, donc il continue à dériver
  même quand des balises sont détectées. C'est visible dans le scénario de
  la section 4 (erreur de ~0.2 rad sur theta en fin de run).
- **Pas un EKF complet.** La fusion est une moyenne pondérée par un
  scalaire d'incertitude, pas une covariance 3×3 avec gain de Kalman —
  volontairement plus simple, comme suggéré par le cours pour une première
  version, mais moins rigoureux qu'un vrai filtre de Kalman étendu.
- **Bruits supposés gaussiens indépendants.** Pas de biais systématique
  simulé (ex. roue légèrement mal calibrée en permanence), alors qu'un
  vrai robot a souvent ce type d'erreur.
- **Détection de balise en tout-ou-rien.** `detection_radius` est une
  coupure nette (0 % ou 100 % de chance de détection selon la distance) ;
  aucune occlusion par obstacle n'est modélisée (pas d'interaction avec
  `sensors/lidar.py` ou une carte d'obstacles).
- **`uncertainty` est un scalaire unique**, pas une incertitude séparée
  par axe (x, y, theta) — une simplification qui limite la précision de
  la décision d'arrêt sûr dans `safety/safety_manager.py`.

