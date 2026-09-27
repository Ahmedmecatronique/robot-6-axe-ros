# Architecture du projet

Ce document décrit **comment le système est construit et pourquoi** : les machines, les nœuds ROS 2, les messages, la
sécurité, la cinématique, le firmware, la vision, l'IHM et les choix de conception. Pour *utiliser* le projet, voir
le [README](../README.md).

## Sommaire

1. [Vue d'ensemble](#1-vue-densemble)
2. [Principes de conception](#2-principes-de-conception)
3. [Graphe ROS 2](#3-graphe-ros-2)
4. [Machine d'états du robot](#4-machine-détats-du-robot)
5. [Sécurité en couches](#5-sécurité-en-couches)
6. [Cinématique et trajectoires](#6-cinématique-et-trajectoires)
7. [Firmware STM32](#7-firmware-stm32)
8. [Vision et calibrations](#8-vision-et-calibrations)
9. [IHM](#9-ihm)
10. [Configuration : une source de vérité](#10-configuration--une-source-de-vérité)
11. [Déploiement et réseau](#11-déploiement-et-réseau)
12. [Carte des fichiers](#12-carte-des-fichiers)
13. [Décisions de conception](#13-décisions-de-conception)
14. [Limites connues et évolutions](#14-limites-connues-et-évolutions)

---

## 1. Vue d'ensemble

```
┌─────────────────────────────── PC (Windows ou Ubuntu) ────────────────────────────────┐
│  IHM PySide6 : vue 3D, contrôle, tableau de bord, caméra, rosbag, apprentissage        │
│  transport : rosbridge/WebSocket (Windows)  |  ROS 2 natif/DDS (Ubuntu)  |  démo local │
└───────────────────────────────────────────┬────────────────────────────────────────────┘
                                            │  Wi-Fi / Ethernet  (TCP 9090  ou  DDS)
┌──────────────────────────── Raspberry Pi 4 · Ubuntu 22.04 · ROS 2 Humble ─────────────┐
│  vision_node ─▶ ik_node ─▶ trajectory_node ─▶ /joint_commands (100 Hz)                  │
│  fk_node · robot_state_publisher · safety_node · pick_node · bag_node · rosbridge       │
│  micro_ros_agent  ◀── UART/USB 921600 bauds ──▶  STM32                                  │
└───────────────────────────────────────────┬────────────────────────────────────────────┘
                                            │
┌──────────────────────────── STM32F446RE · FreeRTOS · micro-ROS ────────────────────────┐
│  tâche contrôle 1 kHz (PID sur encodeurs, homing, watchdog, arrêt d'urgence)            │
│  ISR 20 kHz : générateur de pas  →  A4988 → NEMA 17 ×6   ;  encodeurs ×6  ;  fins ×6    │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

Trois « chaînes » de données :

| Chaîne | Chemin |
|---|---|
| **Commande** | IHM / vision → `/target_pose` → `ik_node` → `/joint_goal` → `trajectory_node` → `/joint_commands` → STM32 → moteurs |
| **Mesure** | encodeurs → STM32 → `/joint_states` → `fk_node` (`/end_effector_pose`, TF), `safety_node`, IHM |
| **Vision** | caméra → `vision_node` → `/target_pose` + `/camera/image_annotated` |

---

## 2. Principes de conception

1. **Une seule source de vérité** : `robot_description/config/robot.yaml` alimente l'URDF, la cinématique, la sécurité,
   l'IHM et — via un générateur — le firmware. Changer une longueur ne se fait qu'à un endroit.
2. **La boucle rapide ne dépend jamais du réseau ni de micro-ROS.** Le PID 1 kHz et le générateur de pas tournent sur
   le STM32 ; micro-ROS n'échange que des variables partagées. Si l'agent ou le Pi tombe, le watchdog arrête les moteurs.
3. **Un seul écrivain par consigne.** `/joint_commands` n'est publié que par `trajectory_node` ; tout le reste envoie un
   *but* (`/joint_goal`). Pas de conflit, une seule application des limites de vitesse/accélération.
4. **Échec sûr par défaut.** Tout état inconnu (pas de mode, robot non prêt, liaison perdue) *bloque* les mouvements ;
   seul l'arrêt d'urgence reste toujours actif.
5. **Un seul modèle partout.** L'IHM dessine le robot avec la même cinématique que `fk_node`, et l'URDF est vérifié
   identique au modèle DH (`tools/check_urdf_vs_dh.py`).
6. **La logique n'a pas besoin de ROS pour être testée.** Cinématique, trajectoires, vision, noyau de l'IHM : Python pur,
   testés sans ROS ni matériel.

---

## 3. Graphe ROS 2

### 3.1 Nœuds

| Paquet | Nœud | Machine | Rôle |
|---|---|---|---|
| robot_kinematics | `fk_node` | Pi | cinématique directe : `/end_effector_pose`, TF `dh_1..dh_6` + `tool_fk` |
| | `ik_node` | Pi | `/target_pose` → IK → `/joint_goal` ; service `/solve_ik` (aperçu sans mouvement) ; `/ik_result` |
| | `trajectory_node` | Pi | interpole (quintique/trapèze) à 100 Hz, respecte vitesse/accélération ; seul publisher de `/joint_commands` |
| robot_control | `safety_node` | Pi | arrêt d'urgence verrouillé + heartbeat, limites, écart de poursuite, watchdog de liaison, `/diagnostics`, services `home`/`enable`, écho du ping de l'IHM |
| | `sim_robot_node` | sim | émule le STM32 (mêmes topics, mêmes états, watchdog) |
| | `pick_node` | Pi | saisie : action `/pick_object` + services `/pick/start`, `/pick/cancel`, topic `/pick_status` |
| | `bag_node` | Pi | rosbag2 côté robot : service `/bag/command`, topic `/bag/status` |
| robot_vision | `vision_node` | Pi | détection ArUco ou HSV, pose 3D dans `base_link`, image annotée, TF caméra statique |
| — | `micro_ros_agent` | Pi | pont UART ↔ DDS |
| — | `rosbridge_websocket` | Pi | pont WebSocket pour l'IHM Windows |
| — | `robot_state_publisher` | Pi | TF des liens depuis l'URDF |
| robot_gui | `gui` / `main_web` | PC | IHM (voir §9) |
| — | firmware STM32 | STM32 | nœud micro-ROS `stm32_micro_ros` |

### 3.2 Topics

| Topic | Type | Publie → s'abonne | Fréq. | QoS |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | STM32 → `fk_node`, `safety_node`, IHM, `robot_state_publisher` | 100 Hz | best effort |
| `/joint_goal` | `trajectory_msgs/JointTrajectory` | `ik_node`, IHM, `pick_node` → `trajectory_node` | évt | reliable |
| `/joint_commands` | `sensor_msgs/JointState` (pos + vitesse) | `trajectory_node` → STM32, IHM, `safety_node` | 100 Hz | reliable |
| `/target_pose` | `geometry_msgs/PoseStamped` | `vision_node`, IHM → `ik_node`, `pick_node` | 10 Hz / évt | reliable |
| `/end_effector_pose` | `PoseStamped` | `fk_node` → IHM | 50 Hz | best effort |
| `/emergency_stop` | `std_msgs/Bool` | IHM, `safety_node` → STM32, `trajectory_node` | évt + heartbeat 10 Hz | reliable |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | `safety_node` → IHM | 2 Hz | reliable |
| `/stm32/status` | `std_msgs/Int32MultiArray` | STM32 → `safety_node`, `trajectory_node`, IHM | 10 Hz | best effort |
| `/stm32/command` | `std_msgs/Int32` | `safety_node` → STM32 | évt | reliable |
| `/gripper_command` | `std_msgs/Float32` | IHM, `pick_node` → STM32 | évt | reliable |
| `/robot_mode` | `std_msgs/String` (`real` / `simulation`) | `safety_node` → IHM | 1 Hz + latched | reliable |
| `/trajectory_active` | `std_msgs/Bool` | `trajectory_node` → IHM | 5 Hz + changement | reliable |
| `/camera/image_annotated` | `sensor_msgs/CompressedImage` (JPEG) | `vision_node` → IHM | 15 Hz | best effort |
| `/target_visible` | `std_msgs/Bool` | `vision_node` → IHM | chaque image | reliable |
| `/vision/marker_in_camera` | `PoseStamped` | `vision_node` → calibration main-œil | chaque image | best effort |
| `/ik_result` | `std_msgs/String` (JSON) | `ik_node` → IHM | évt | reliable |
| `/pick_status` | `std_msgs/String` (JSON) | `pick_node` → IHM | évt | reliable |
| `/bag/status` | `std_msgs/String` (JSON) | `bag_node` → IHM | 1 Hz | best effort |
| `/gui/ping`, `/gui/pong` | `std_msgs/Float64` (horodatage) | IHM → `safety_node` → IHM | 2 Hz | reliable |
| `/replay/*` | (mêmes types) | rosbag2 → IHM | — | — |

**Pourquoi ces QoS.** Mesures, vidéo → *best effort* : une valeur perdue est remplacée 10 ms plus tard, et cela évite
de bloquer sur des retransmissions. Consignes, arrêt d'urgence, services → *reliable* : aucune perte tolérée.

Services : `/robot/home`, `/robot/enable` (`safety_node`) · `/solve_ik` (`ik_node`) · `/pick/start`, `/pick/cancel`
(`pick_node`) · `/bag/command` (`bag_node`). Action : `/pick_object`.

### 3.3 Graphe attendu (rqt_graph)

```
 caméra ─▶ vision_node ─/target_pose──────────────┐
             └─/camera/image_annotated ─▶ (PC) IHM │
 (PC) IHM ─/target_pose──────────────────────────┤
                                                  ▼
                                               ik_node ─/joint_goal─┐
 (PC) IHM ─/joint_goal ──────────────────────────────────────────┤
 pick_node ─/joint_goal ─────────────────────────────────────────┤
                                                                  ▼
                                                        trajectory_node
                                                                  │ /joint_commands  100 Hz
                                                                  ▼
                                    micro_ros_agent ◀─ UART/USB ─▶ STM32  (PID 1 kHz)
                                                                  │ /joint_states  100 Hz
        ┌─────────────────────────────────────────────────────────┤
        ▼                          ▼                              ▼
     fk_node             robot_state_publisher             safety_node ─/emergency_stop (heartbeat)─▶ STM32
       │ /end_effector_pose, /tf                              │ /diagnostics, /robot_mode
       ▼                                                      ▼
    (PC) IHM ◀────────────────────────────────────────────────┘
```

---

## 4. Machine d'états du robot

Le STM32 (et `sim_robot_node`) publie l'état sur `/stm32/status` ; les codes sont définis à l'identique dans
`firmware/Core/Inc/app_types.h` et `robot_kinematics/status_codes.py` — **à modifier ensemble**.

```
 BOOT ─▶ DISABLED ──ENABLE──▶ IDLE (non référencé) ──HOME──▶ HOMING ──6 axes référencés──▶ READY
            ▲                    ▲   │ ZERO_HERE (mise en service, sans fins de course)      │
            │DISABLE             │   └────────────────────────────────────────────────────▶ │
            │                    └───────────────────── ENABLE (si référencé) ──────────────┘
 E-STOP  ◀── /emergency_stop = true (depuis IDLE, HOMING ou READY) ; sortie : ENABLE
 FAULT   ◀── watchdog, écart de poursuite, limite dépassée, homing échoué ; sortie : ENABLE
```

| `/stm32/status` (Int32MultiArray, 8 valeurs) | Signification |
|---|---|
| `[0]` état | 0 BOOT · 1 DISABLED · 2 IDLE · 3 HOMING · 4 READY · 5 ESTOP · 6 FAULT |
| `[1]` axes référencés | masque de bits |
| `[2]` fins de course actifs | masque de bits |
| `[3]` défauts | bits : 0 comm · 1 poursuite · 2 limite · 3 homing · 4 e-stop ; bits 8-13 = axe concerné |
| `[4]` écart max consigne/mesure | milli-radians |
| `[5]` fréquence de la boucle | Hz |
| `[6]` âge de la dernière consigne | ms |
| `[7]` uptime | s |

Commandes `/stm32/command` : `1` ENABLE (efface e-stop/défaut) · `2` DISABLE · `3` HOME · `4` ZERO_HERE.
Les mouvements ne sont acceptés que dans l'état **READY** ; `trajectory_node` colle passivement à la mesure dans les
autres états et refuse les buts.

---

## 5. Sécurité en couches

| Couche | Où | Protège contre |
|---|---|---|
| Bouton physique coupant l'alimentation moteurs | **à installer** | tout, y compris logiciel planté |
| Limites articulaires | IK (rejette), `trajectory_node` (rejette), firmware (borne + défaut), `safety_node` (e-stop) | dépassement de course |
| Plan de sécurité `workspace.z_min` | `ik_node` | cible sous la table |
| Écart de poursuite > 6° pendant 200 ms | firmware ; `safety_node` (10° / 0,5 s) | moteur bloqué, pas perdus, mauvais sens |
| **Watchdog 300 ms** | firmware | plus de consigne **ou** de heartbeat (Pi, réseau, micro-ROS) |
| Arrêt d'urgence **verrouillé** | firmware + `safety_node` | reprise involontaire ; un heartbeat `false` n'efface pas un `true` |
| Contrôle de progression du homing | firmware | encodeur inversé ou débranché, moteur bloqué |
| Blocage des commandes dans l'IHM | IHM | mauvais mode (sim/réel), robot non prêt, rejeu |
| Rejeu rosbag redirigé | `bag_node` | rejouer des commandes vers le vrai robot |

Le **heartbeat** : `safety_node` publie `/emergency_stop` à 10 Hz (`false` = tout va bien). Le STM32 s'arrête si le
heartbeat disparaît, ce qui protège aussi d'un plantage de `safety_node`. L'arrêt logiciel **garde le couple de
maintien** (`ESTOP_DISABLE_DRIVERS = 0`) : couper les drivers laisserait le bras tomber.

---

## 6. Cinématique et trajectoires

Code : `robot_kinematics/kinematics.py`, `trajectory.py`, `transforms.py` (NumPy pur, testés sans ROS).

**Modèle** — DH standard, `A_i = Rz(θ)·Tz(d)·Tx(a)·Rx(α)`, α = (+90°, 0, 0, +90°, −90°, 0), `θ = q + offset`
(offset₂ = 90° pour que `q = 0` soit le bras vertical). Les axes 4-5-6 se coupent : **poignet sphérique**.

**IK analytique** (découplage position/orientation) :
1. centre du poignet `p_c = p − d6·z6` ;
2. `q1 = atan2(y, x)` (épaule « face ») ou +180° (« dos ») ;
3. `q2, q3` : problème plan à deux segments, coude haut/bas ;
4. `q4, q5, q6` depuis `R36 = R03ᵀ·R`, poignet normal ou retourné.

→ jusqu'à **8 solutions** ; on garde celles qui respectent les limites *et* reproduisent la pose (garde-fou contre un
modèle non conforme), puis la **plus proche de l'état actuel** (somme des écarts angulaires). Si aucune :
**repli Levenberg-Marquardt** (Jacobienne amortie, redémarrages aléatoires), sinon message d'échec explicite
(« hors de portée : centre du poignet à x m, portée max y m » / « hors limites articulaires : joint_n »).
Hypothèse : pas de décalage d'épaule ni d'avant-bras (`a1 = d4 = 0`).

**Singularités** : poignet (|sin q5| faible), coude (bras tendu/replié), épaule (centre du poignet sur l'axe de la
base), conditionnement de la Jacobienne normalisée. Une pose singulière est *acceptée avec avertissement* par défaut
(`reject_near_singularity: false`) ; le mouvement est interpolé dans l'espace articulaire, donc les vitesses restent bornées.

**Trajectoires** : polynôme quintique synchronisé sur les 6 axes (position, vitesse, accélération continues), durée
choisie pour que *tous* les axes respectent `v_max`/`a_max` ; accepte une vitesse initiale non nulle (nouveau but pendant
un mouvement, sans à-coup). Profil trapézoïdal disponible (départ à l'arrêt). `speed_scale` réglable à chaud.
La vitesse est publiée avec la position : c'est la **préaction** du PID du firmware.

**Validation** : `FK(IK(pose)) = pose` (1 µm / 1 µrad) sur 200 poses aléatoires pour toutes les configurations ;
la Jacobienne est comparée aux différences finies ; l'URDF est comparé au modèle DH.

---

## 7. Firmware STM32

Code : `firmware/Core/Src` (voir aussi [`firmware/README.md`](../firmware/README.md)).

| Élément | Détail |
|---|---|
| Tâche **contrôle** | FreeRTOS, priorité maximale, période exacte 1 ms (`vTaskDelayUntil`) : encodeurs → PID → fréquences de pas → instantané |
| Tâche **micro-ROS** | priorité basse, nœud `stm32_micro_ros`, machine « attendre l'agent → créer les entités → servir → détecter la perte » |
| **ISR TIM7 à 20 kHz** | générateur de pas : accumulateur de phase 32 bits par axe ; retenue = impulsion `STEP` (≤ 10 kHz/axe, 8 kHz par défaut) ; ne fait aucun appel FreeRTOS (priorité supérieure au RTOS) |
| **Encodeurs** | TIM1, TIM2, TIM3, TIM4, TIM5, TIM8 en mode encodeur ×4 ; lecture 16 bits avec différence modulo (débordement géré) |
| Servo de pince | TIM10_CH1, PWM 50 Hz, 1-2 ms |
| PID | erreur de position (rad) → **vitesse** articulaire ; `ff + Kp·e + Ki·∫e − Kd·ḋ(mesure)` ; anti-windup ; zone morte ; limitation d'accélération |
| Conversion | `pas/s = vitesse · CFG_STEPS_PER_RAD` (réducteur et micro-pas inclus) ; angle articulaire = encodeurs / `CFG_ENC_COUNTS_PER_RAD` |
| **Homing** | par axe (ordre configurable, poignet d'abord) : approche du fin de course → recul lent → **référence au front de relâchement** ; timeout, contrôle de progression |
| Consigne | extrapolée linéairement (≤ 50 ms) entre deux messages à 100 Hz |
| Publications | `/joint_states` 100 Hz (`effort` = commande normalisée : l'A4988 ne mesure pas le courant), `/stm32/status` 10 Hz |

**Pourquoi « pas-à-pas en boucle fermée en vitesse » ?** Le moteur est un intégrateur (fréquence de pas → position) :
un P sur l'erreur mesurée par l'encodeur suffit à annuler l'erreur, rattrape les pas perdus et permet de **détecter un
décrochage** (écart de poursuite), ce qu'un pas-à-pas en boucle ouverte ne sait pas faire.

**Chaîne de configuration** : `robot.yaml` → `tools/gen_firmware_config.py` → `robot_config_generated.h`
(`CFG_STEPS_PER_RAD`, limites, PID, homing…). Câblage complet dans `firmware/Core/Inc/config.h`.

---

## 8. Vision et calibrations

Code : `robot_vision/` (`vision_core.py` sans ROS, `vision_node.py`, `frame_source.py`, scripts de calibration).

* **Sources d'image** : topic ROS, webcam/GStreamer (OpenCV), ou Pi Camera v3 via Picamera2 avec **mise au point
  verrouillée** (l'autofocus fausserait la calibration).
* **ArUco** : détection (compatible OpenCV 4.5 et ≥ 4.7) → `solvePnP` (IPPE_SQUARE) → pose du marqueur → pose dans
  `base_link` via `T_base_cam` ; orientation de saisie **par le dessus** (axe z de l'outil vers le bas, yaw du marqueur
  ramené dans ]−90°, 90°]).
* **HSV** : plus grand blob de la couleur cible ; position par **intersection du rayon optique avec le plan de la
  table** (`plane_z`), sans capteur de profondeur ; l'intervalle de teinte « enjambe » 0 pour le rouge.
* Lissage : médiane glissante de la position + moyenne circulaire du yaw.
* **Calibration intrinsèque** : échiquier, capture automatique sans écran sur le Pi, erreur de reprojection affichée.
* **Calibration main-œil (eye-to-hand)** : marqueur sur l'outil, `T_base_tool` par FK, `T_cam_marker` par vision,
  `cv2.calibrateHandEye` sur la pose *inverse* de l'outil → `T_base_cam` ; dispersion de `T_outil_marqueur` comme
  indicateur de qualité. Validée par un test synthétique. Procédure : [`CALIBRATION.md`](CALIBRATION.md).

---

## 9. IHM

Code : `ros2_ws/src/robot_gui/robot_gui/`.

### 9.1 Ponts (transports) — un seul code d'interface

```
                        ┌──────────── BridgeCore (sans ROS) ────────────┐
   widgets Qt  ◀────────│ tampons, snapshot(), history(), événements,   │
   (panels, 3D, …)      │ TOUTES les commandes (dict JSON ROS)          │
                        └──────▲──────────────▲──────────────▲──────────┘
                               │              │              │
                        RosBridge        WebBridge      DemoBridge
                        rclpy / DDS      roslibpy       robot simulé
                        (Ubuntu)         (Windows)      dans le processus
```

Chaque transport n'implémente que l'abonnement aux topics (→ `feed_*()`), `_publish()` et `_call()` (services).
Les commandes sont exprimées une seule fois, en dictionnaires au format JSON de ROS.

| | `RosBridge` | `WebBridge` | `DemoBridge` |
|---|---|---|---|
| Prérequis PC | ROS 2 Humble (Ubuntu) | Python + `roslibpy` | Python |
| Liaison | DDS | WebSocket TCP :9090 (rosbridge) | aucune |
| Usage | poste Ubuntu | **poste Windows** | découverte, tests |

### 9.2 Fenêtres

| Module | Rôle |
|---|---|
| `main_window.py` | barre d'état, séparateurs, boucle d'affichage (3D 60 Hz, panneaux 10 Hz, courbes 20 Hz, caméra 15 Hz) |
| `view3d.py` | pyqtgraph.opengl : robot mesuré, fantôme, repères, trace, cible ; même géométrie que l'URDF |
| `panels.py` | mode sim/réel, sliders, cartésien + IK, boutons, **arrêt d'urgence fixé hors de la zone défilante** |
| `dashboard.py` | table par axe, FK, courbes PID, liaison/watchdog, alarmes avec anti-répétition |
| `camera_tab.py` | flux annoté, coordonnées 3D, saisie |
| `teach_tab.py` | rosbag, poses mémorisées, lecture de séquence (machine d'états locale) |

### 9.3 Sécurité et latence côté IHM

* Blocages : mode attendu ≠ mode réel, mode inconnu, robot non PRÊT, rejeu → commandes désactivées ; arrêt d'urgence
  toujours actif (publié 3 fois).
* Les callbacks réseau ne font que ranger la dernière valeur (`feed_*`) ; le thread Qt lit un instantané cohérent et
  **ne bloque jamais** sur le réseau.
* **Performance (v1.1)** : historique des courbes en **tampon anneau NumPy** (écriture O(1), lecture sans copie de
  réordonnancement) ; **onglets masqués = aucun travail** (caméra, courbes, tableau ; les alarmes restent évaluées) ;
  vue 3D : cinématique directe calculée une fois par image et redessin seulement si la pose change ; cadences décalées
  (3D 60 Hz, courbes et caméra 15 Hz, tableau 10 Hz) ; rosbridge limité à 50 Hz sur les flux rapides (`--rate`).
  Mesures : [`CHANGELOG.md`](../CHANGELOG.md), banc `tools/bench_gui.py`.
* **Latence réelle** : ping horodaté `/gui/ping` → écho `/gui/pong` (`safety_node`), 2 Hz : l'aller-retour mesuré ne
  dépend pas de la synchronisation des horloges (contrairement à l'âge des messages ROS).
* Budget de latence visé < 50 ms : encodeur→STM32 1 ms, micro-ROS ≤ 10 ms, UART ~2 ms, DDS/Wi-Fi 2-20 ms,
  (JSON/WebSocket +quelques ms), rendu ≤ 16 ms. La latence affichée n'a de sens que si les horloges sont synchronisées.

---

## 10. Configuration : une source de vérité

```
                    robot_description/config/robot.yaml
                                   │
   ┌───────────────┬───────────────┼─────────────────┬────────────────────────┐
   ▼               ▼               ▼                 ▼                        ▼
 URDF (xacro)   robot_kinematics  safety_node   IHM (limites, 3D)   tools/gen_firmware_config.py
 (RViz2, TF)    (FK, IK, limites)                                    → robot_config_generated.h
```

Autres fichiers de paramètres : `robot_bringup/config/params.yaml` (paramètres des nœuds),
`robot_vision/config/camera.yaml` et `hand_eye.yaml` (résultats des calibrations).

---

## 11. Déploiement et réseau

| Topologie | Liaison PC ↔ Pi | Remarques |
|---|---|---|
| **PC Windows** (retenu) | WebSocket rosbridge, TCP 9090, connexion *sortante* du PC | pas de DDS, pas de multicast ; port à restreindre au PC (pas d'authentification) |
| PC Ubuntu | DDS (CycloneDDS, découverte unicast recommandée en Wi-Fi) | même `ROS_DOMAIN_ID` sur les deux machines |
| Simulation | une seule machine | `sim.launch.py` : `sim_robot_node` remplace agent + STM32 |

Le Pi doit tourner sous **Ubuntu 22.04** (paquets ROS 2 Humble officiels). Pi ↔ STM32 : câble USB (ST-LINK,
`/dev/ttyACM0`, 921600 bauds) ; liaison UART câblée possible (voir [`INSTALLATION.md`](INSTALLATION.md)).
Lancement : `core.launch.py` (cœur commun) inclus par `pi_real.launch.py` (+ agent micro-ROS + vision) et
`sim.launch.py` (+ simulateur, IHM, RViz2 optionnels) ; `pc_gui.launch.py` lance l'IHM native sur Ubuntu.

---

## 12. Carte des fichiers

```
ros2_ws/src/
├── robot_interfaces/   srv/{Home,SetEnable,SolveIK,BagCommand}.srv · action/PickObject.action
├── robot_description/  config/robot.yaml · urdf/arm6.urdf.xacro · launch/view_robot.launch.py · rviz/robot.rviz
├── robot_kinematics/   config.py · transforms.py · kinematics.py · trajectory.py · status_codes.py · ros_utils.py
│                       fk_node.py · ik_node.py · trajectory_node.py · tests/
├── robot_control/      safety_node.py · sim_robot_node.py · pick_node.py · bag_node.py
├── robot_vision/       vision_core.py · vision_node.py · frame_source.py · camera_calibration.py
│                       hand_eye_calibration.py · config/{camera,hand_eye}.yaml · tests/
├── robot_gui/          bridge_core.py · ros_bridge.py · web_bridge.py · demo_bridge.py · main.py · main_web.py
│                       main_window.py · view3d.py · panels.py · dashboard.py · camera_tab.py · teach_tab.py
│                       theme.py · tests/{test_gui_smoke,test_bridge_core,test_web_bridge,test_demo_bridge}.py
└── robot_bringup/      launch/{core,pi_real,pc_gui,sim}.launch.py · config/params.yaml
firmware/Core/Src/      main.c · app.c · control.c · board.c · pid.c · microros_app.c
firmware/Core/Inc/      config.h · app_types.h · board.h · control.h · pid.h · microros_app.h · app.h
                        robot_config_generated.h (GÉNÉRÉ)
firmware/microros/      colcon.meta
windows/                install.bat · run_gui.bat · run_gui.py · requirements.txt · README_WINDOWS.md
tools/                  gen_firmware_config.py · check_urdf_vs_dh.py · bench_gui.py
docs/                   ARCHITECTURE.md · INSTALLATION.md · CALIBRATION.md · TESTS.md
CHANGELOG.md            journal des changements + mesures de performance
```

---

## 13. Décisions de conception

| Décision | Choix | Alternative écartée | Raison |
|---|---|---|---|
| Liaison Pi ↔ STM32 | **micro-ROS** (UART/USB) | trames + CRC maison | le STM32 est un vrai nœud (topics, QoS, rosbag) ; le watchdog local compense sa dépendance à l'agent |
| IHM | **PySide6 + pyqtgraph** | interface web (Foxglove / roslibjs) | latence et débit sans JSON pour 100 Hz + vidéo ; IK, apprentissage et saisie natifs ; rendu réutilisant la cinématique |
| IHM sous Windows | **rosbridge + roslibpy** | ROS 2 Humble sur Windows | Python 3.8 imposé, DDS difficile en Wi-Fi ; un port TCP suffit |
| Consigne moteurs | **`/joint_goal` → `trajectory_node` → `/joint_commands`** | tous publient sur `/joint_commands` | un seul écrivain, limites appliquées à un seul endroit |
| Pas-à-pas | **boucle fermée en vitesse** sur encodeurs | boucle ouverte | rattrape les pas perdus, détecte un décrochage |
| Génération des pas | accumulateur de phase dans une ISR 20 kHz | un timer PWM par axe | 6 axes avec un seul timer, débit régulier, simple |
| Arrêt d'urgence logiciel | **garde le couple de maintien** | couper les drivers | le bras ne tombe pas ; l'arrêt physique reste nécessaire |
| Simulation | `sim_robot_node` (modèle cinématique) | Gazebo dès le départ | même interface que le STM32, exécutable partout ; Gazebo = évolution |
| Langage | Python (rclpy) | C++ | rapidité de développement ; `trajectory_node` en premier candidat au C++ si la gigue gêne |

---

## 14. Limites connues et évolutions

* Le **firmware n'a pas été compilé** ni exécuté ; les nœuds ROS 2, les launches et rosbridge réel n'ont pas tourné
  sous ROS 2 (voir l'état de validation du [README](../README.md#7-état-de-validation)).
* La simulation est **cinématique** (pas de dynamique, pas de collisions) ; l'IK n'évite pas les collisions.
* IK analytique limité aux bras sans décalage d'épaule/avant-bras ; le repli numérique couvre le reste.
* Pas de mesure de courant (A4988) : `effort` = commande normalisée ; pas de mesure de température.
* rosbridge sans authentification ; l'arrêt d'urgence de l'IHM passe par le réseau.
* Pi Camera v3 sous Ubuntu 22.04 : pile libcamera à compiler (non testé) ; webcam USB recommandée pour démarrer.
* Évolutions : validation matérielle et réglage des PID, `trajectory_node` en C++, Gazebo / `ros2_control`, évitement
  de collisions, authentification rosbridge, mesure de courant (drivers TMC), version ARM64 de l'IHM.
