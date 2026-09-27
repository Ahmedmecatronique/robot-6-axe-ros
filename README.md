# Bras robotique 6 axes — ROS 2 · STM32 · IHM jumeau numérique

Projet complet d'un bras à **6 degrés de liberté** (anthropomorphe, poignet sphérique) piloté par :

| Machine | Rôle |
|---|---|
| **Raspberry Pi 4** (Ubuntu 22.04 + ROS 2 Humble) | cerveau : cinématique, trajectoires, sécurité, vision par ordinateur |
| **STM32 Nucleo-F446RE** (FreeRTOS + micro-ROS) | bas niveau : 6 × NEMA 17 + A4988, encodeurs, fins de course, PID 1 kHz |
| **PC portable (Windows ou Ubuntu)** | IHM de supervision en temps réel (jumeau numérique 3D) |

> Fonctionnalités clés : IK analytique (8 configurations) + repli numérique, trajectoires quintiques, boucle fermée
> sur encodeurs, arrêt d'urgence verrouillé + watchdog, détection ArUco/couleur → saisie d'objet, calibration caméra
> et main-œil, IHM avec robot « fantôme » de la consigne, enregistrement/rejeu rosbag2, mode apprentissage.

Documentation détaillée : [Architecture](docs/ARCHITECTURE.md) · [Installation](docs/INSTALLATION.md) ·
[Calibrations](docs/CALIBRATION.md) · [Plan de tests](docs/TESTS.md) · [IHM sous Windows](windows/README_WINDOWS.md) ·
[Firmware](firmware/README.md) · [Journal des changements](CHANGELOG.md)

**Version actuelle : v1.1** (IHM plus performante, latence réelle mesurée — voir le journal des changements).

---

## 1. Démarrage rapide

### A. Voir l'IHM tout de suite (Windows, aucun robot, aucun ROS)

```bat
windows\install.bat                 :: une seule fois : Python 3.10-3.12 requis
windows\run_gui.bat --demo          :: robot simulé dans le programme
windows\run_gui.bat --demo-unhomed  :: idem, démarre désactivé (tester Activer puis Homing)
```

Un robot simulé (vraie cinématique inverse, vraies trajectoires, vraies limites) tourne dans le processus. Essayer :
sliders, pose cartésienne, arrêt d'urgence, « Aller saisir l'objet détecté », courbes, vue 3D avec fantôme.

### B. Simulation complète avec ROS 2 (une machine Ubuntu 22.04)

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch robot_bringup sim.launch.py          # cœur + STM32 simulé + IHM native
ros2 launch robot_bringup sim.launch.py gui:=false rviz:=true   # RViz2 avec robot mesuré + fantôme
```

Un PC Windows peut ensuite s'y connecter : `windows\run_gui.bat --host <IP de la machine>`.

### C. Robot réel

```bash
# Raspberry Pi
ros2 launch robot_bringup pi_real.launch.py serial_dev:=/dev/ttyACM0 camera_source:=opencv
```
```bat
:: PC Windows                                     (Ubuntu : ros2 launch robot_bringup pc_gui.launch.py)
windows\run_gui.bat --host 192.168.1.50
```

Dans l'IHM : choisir le mode **Réel** (confirmation), **Activer**, puis **Homing**. Suivre impérativement
[`docs/TESTS.md`](docs/TESTS.md) dans l'ordre (simulation → banc sans moteurs → axe par axe → vision → saisie).

---

## 2. Structure du dépôt

```
robot 6 axe ros/
├── README.md                     ce fichier
├── docs/
│   ├── ARCHITECTURE.md           architecture complète (nœuds, topics, firmware, IHM, sécurité, décisions)
│   ├── INSTALLATION.md           Pi, PC, micro-ROS, réseau, caméra
│   ├── CALIBRATION.md            calibration caméra + main-œil (hand-eye)
│   └── TESTS.md                  plan de tests, de la simulation à la saisie
├── CHANGELOG.md                  journal des changements et mesures de performance
├── ros2_ws/src/
│   ├── robot_interfaces/         services Home, SetEnable, SolveIK, BagCommand ; action PickObject
│   ├── robot_description/        robot.yaml (SOURCE UNIQUE), URDF/xacro, config RViz2
│   ├── robot_kinematics/         FK, IK, singularités, trajectoires ; nœuds fk / ik / trajectory
│   ├── robot_control/            safety_node, sim_robot_node (STM32 simulé), pick_node, bag_node
│   ├── robot_vision/             vision_node (ArUco / HSV), calibration caméra et main-œil
│   ├── robot_gui/                IHM PySide6 : ponts ROS 2 / rosbridge / démo, fenêtres, 3D
│   └── robot_bringup/            launch : core, pi_real, pc_gui, sim + params.yaml
├── firmware/                     STM32CubeIDE : FreeRTOS + micro-ROS, PID 1 kHz, homing, watchdog
├── windows/                      IHM sans ROS 2 : install.bat, run_gui.bat, run_gui.py, requirements.txt
└── tools/                        gen_firmware_config.py, check_urdf_vs_dh.py, bench_gui.py
```

---

## 3. Le robot en bref

**Cinématique (DH standard)** — `A_i = Rz(θi)·Tz(di)·Tx(ai)·Rx(αi)`

| i | θᵢ | dᵢ | aᵢ | αᵢ |
|---|---|---|---|---|
| 1 | q1 | d1 | 0 | +90° |
| 2 | q2 + 90° | 0 | a2 | 0 |
| 3 | q3 | 0 | a3 | 0 |
| 4 | q4 | 0 | 0 | +90° |
| 5 | q5 | 0 | 0 | −90° |
| 6 | q6 | d6 | 0 | 0 |

À `q = 0` le bras est **vertical**. Portée de position : `a2 + a3 = 0,60 m` autour de l'épaule.

**Valeurs par défaut** (toutes dans [`robot.yaml`](ros2_ws/src/robot_description/config/robot.yaml)) :

| Sujet | Valeur | Statut |
|---|---|---|
| Longueurs | `d1 = a2 = a3 = 0,30 m`, `d6 = 0,10 m` (poignet → outil) | 30 cm confirmés ; `d6` supposé |
| Moteurs / drivers | NEMA 17 + A4988, micro-pas 1/16 | confirmé |
| Encodeurs | 1000 impulsions/tour (×4 en quadrature), sur l'arbre moteur | confirmé — niveau logique 3,3 V / 5 V à vérifier |
| Réducteurs (tours moteur / tour articulation) | 30 · 30 · 30 · 20 · 20 · 10 | supposé |
| Limites articulaires | J1 ±170°, J2 ±90°, J3 ±150°, J4 ±170°, J5 ±120°, J6 ±170° | supposé |
| Vitesses max | 30 · 30 · 30 · 45 · 45 · 90 °/s | supposé |
| Microcontrôleur | Nucleo-F446RE | confirmé |
| Liaison Pi ↔ STM32 | USB (ST-LINK, `/dev/ttyACM0`), 921600 bauds | choix par défaut |
| Fins de course | 1 par axe, contact vers GND, pull-up interne | supposé |
| Pince | servo PWM 50 Hz (`/gripper_command` : 0 fermée … 1 ouverte) | supposé |
| Caméra | fixe (*eye-to-hand*), Pi Camera v3 ou webcam USB, ArUco `DICT_4X4_50` de 40 mm | supposé |
| PC | Windows (IHM par rosbridge, sans ROS 2) | confirmé |

### Adapter le projet à ton robot

Tout se règle dans **un seul fichier**, `robot.yaml` : longueurs, décalages d'angle, limites, réducteurs, PPR, sens des
moteurs et encodeurs, homing, PID. Puis :

```bash
python tools/gen_firmware_config.py   # régénère les constantes du firmware, puis recompiler/reflasher
python tools/check_urdf_vs_dh.py      # vérifie que l'URDF et la cinématique décrivent le même robot (pip install xacro)
colcon build --symlink-install        # côté ROS 2
```

---

## 4. Interface ROS 2 (résumé)

| Topic | Type | Sens | Fréq. | QoS |
|---|---|---|---|---|
| `/joint_states` | `JointState` | STM32 → tous | 100 Hz | best effort |
| `/joint_goal` | `JointTrajectory` | `ik_node`, IHM, `pick_node` → `trajectory_node` | évt | reliable |
| `/joint_commands` | `JointState` (pos + vitesse) | `trajectory_node` → STM32, IHM | 100 Hz | reliable |
| `/target_pose` | `PoseStamped` | `vision_node`, IHM → `ik_node` | 10 Hz / évt | reliable |
| `/end_effector_pose` | `PoseStamped` | `fk_node` → IHM | 50 Hz | best effort |
| `/emergency_stop` | `Bool` | IHM, `safety_node` → STM32 | évt + heartbeat 10 Hz | reliable |
| `/diagnostics` | `DiagnosticArray` | `safety_node` → IHM | 2 Hz | reliable |
| `/camera/image_annotated` | `CompressedImage` | `vision_node` → IHM | 15 Hz | best effort |
| `/gui/ping` → `/gui/pong` | `Float64` | IHM → `safety_node` → IHM | 2 Hz | reliable |
| `/stm32/status`, `/stm32/command` | `Int32MultiArray`, `Int32` | STM32 ↔ `safety_node` | 10 Hz / évt | best effort / reliable |

Services : `/robot/home`, `/robot/enable`, `/solve_ik`, `/pick/start`, `/pick/cancel`, `/bag/command` ; action
`/pick_object`. **Le seul écrivain de `/joint_commands` est `trajectory_node`.** Tableau complet, QoS et graphe :
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

---

## 5. L'IHM

* **Vue 3D temps réel** (pyqtgraph/OpenGL) : robot **mesuré** (positions encodeurs, plein), robot **fantôme** de la
  consigne (translucide), repères, trace de l'effecteur, cible caméra. Rendu 60 Hz, même cinématique que `fk_node`.
* **Contrôle** : 6 sliders (limites lues dans le YAML), pose cartésienne X Y Z R P Y avec aperçu IK
  (✔ atteignable / ⚠ proche d'une singularité / ✖ inatteignable), Activer/Désactiver, Homing, Repos, Arrêt doux, pince,
  vitesse, et un grand **ARRÊT D'URGENCE toujours visible**.
* **Tableau de bord** : consigne/mesure/erreur/vitesse/effort par axe, pose FK, courbes pour régler les PID, état de la
  liaison (Hz, **latence aller-retour mesurée par ping**, watchdog), défauts, alarmes. La barre d'état affiche aussi les
  images/s et le coût de la boucle d'affichage.
* **Caméra** : flux annoté, coordonnées 3D de la cible, « Aller saisir l'objet détecté ».
* **Enregistrement / apprentissage** : rosbag2 enregistré/rejoué côté robot (rejeu redirigé vers `/replay/…`, rien
  n'est envoyé au robot), poses mémorisées avec pince et pause, lecture en boucle, sauvegarde JSON.
* **Sécurité de l'interface** : commandes bloquées si le mode attendu (Simulation/Réel) ≠ mode réel du système, si le
  robot n'est pas PRÊT, ou pendant un rejeu ; l'arrêt d'urgence reste toujours actif.

Trois transports, un seul code d'interface : **rosbridge** (Windows), **ROS 2 natif** (Ubuntu), **démo** (local).

---

## 6. Sécurité — à lire avant d'allumer les moteurs

* Le firmware garde **plusieurs couches** : limites articulaires (IK, trajectoire, firmware, `safety_node`), écart de
  poursuite (moteur bloqué/décroché), **watchdog 300 ms** (plus de consigne ou de heartbeat → arrêt), arrêt d'urgence
  **verrouillé** jusqu'à réarmement explicite.
* L'arrêt d'urgence **logiciel garde le couple de maintien** (sinon le bras tombe). Il ne remplace pas un **bouton d'arrêt
  physique coupant l'alimentation des moteurs** : à installer.
* **rosbridge n'a aucune authentification** : limiter le port 9090 au PC (pare-feu du Pi), jamais sur Internet.
* Premiers essais : `speed_scale` réduit, zone dégagée, un axe à la fois, bras libre de toute collision.

---

## 7. État de validation

Vérifié dans l'environnement de développement (Windows, **sans ROS 2 ni matériel**) :

| Vérification | Résultat |
|---|---|
| Cinématique / trajectoires (`robot_kinematics/tests`) | 21 tests : `FK(IK(pose)) = pose` sur 200 poses × 8 configurations, choix de la plus proche, singularités, Jacobienne, profils |
| Vision (`robot_vision/tests`) | 10 tests : ArUco, HSV, projection, `solvePnP`, calibration main-œil retrouvant une pose imposée |
| IHM (`robot_gui/tests`) | 30 tests : fenêtre + blocages de sécurité (8) ; noyau des ponts — tampon anneau, ping (6) ; `WebBridge` contre un faux serveur rosbridge avec le vrai `roslibpy` (10) ; mode démo complet (6) |
| Performance de l'IHM (`tools/bench_gui.py`, hors écran) | boucle d'affichage 6,0–6,3 ms en moyenne (v1.0 : 8,1), p95 ≈ 15,9 ms (v1.0 : 20,9), CPU ≈ 45 % d'un cœur (v1.0 : 60 %) — le rendu GPU n'est pas mesuré |
| URDF ≡ modèle DH | écart ≈ 2·10⁻¹⁶ sur 7 repères × 200 configurations (`tools/check_urdf_vs_dh.py`) |
| Compilation Python | tous les fichiers |

**Non vérifié** (demande ROS 2 et/ou le matériel) : exécution réelle des nœuds ROS 2 et des launches, **rosbridge lui-même**
(seul un faux serveur a été testé), rendu OpenGL de la vue 3D (jamais observé : la capture hors écran est vide ; à
voir avec `windows\run_gui.bat --demo`), **compilation et exécution du firmware** (pas de chaîne ARM), liaison micro-ROS, Pi Camera v3 sous
Ubuntu 22.04, réglage des PID. Prévoir de petits ajustements à la première mise en route.

---

## 8. Dépannage rapide

| Symptôme | Piste |
|---|---|
| IHM : « MODE ? » | `safety_node` non lancé (publie `/robot_mode`) |
| IHM Windows : liaison rosbridge perdue | Pi injoignable, `rosbridge_websocket` non lancé, port 9090, pare-feu |
| Connecté mais aucune donnée | `ros2 topic hz /joint_states` sur le Pi ; agent micro-ROS/STM32 ? |
| Axe qui part dans le mauvais sens → défaut « écart de poursuite » | inverser `dir_sign` ou `encoder_sign` dans `robot.yaml`, régénérer le firmware |
| Homing en échec | moteur bloqué, encodeur débranché ou polarité inversée (contrôle de progression) |
| Positions 3D fausses | calibrations non faites (`calibrated: false` dans `camera.yaml` / `hand_eye.yaml`) |
| Vue 3D noire | pilote graphique sans OpenGL 3 |

Autres cas : [`windows/README_WINDOWS.md`](windows/README_WINDOWS.md), [`docs/TESTS.md`](docs/TESTS.md).

---

## 9. Feuille de route possible

* Compiler/valider le firmware sur la carte, régler les PID axe par axe.
* Passer `trajectory_node` en C++ si la gigue à 100 Hz gêne (le reste peut rester en Python).
* Étendre l'IK analytique aux bras avec décalage d'épaule/avant-bras (`a1`, `d4` ≠ 0) ; le repli numérique reste valable.
* Gazebo / `ros2_control` (la simulation actuelle est un modèle cinématique simple, sans dynamique).
* Vérifier les collisions (table, bras) avant exécution ; authentification pour rosbridge.

## 10. Licence et auteur

Apache-2.0 (déclarée dans les `package.xml`). Auteur : ahmed.
