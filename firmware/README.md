# Firmware STM32 — bras 6 axes

STM32F446RE (Nucleo-64), FreeRTOS, micro-ROS (Humble) sur UART/USB. Contrôle de 6 moteurs **NEMA 17 + A4988**
avec **encodeurs sur l'arbre moteur**.

```
Core/Inc, Core/Src
├─ main.c            horloge 180 MHz, init, démarrage FreeRTOS
├─ app.c             création des tâches (contrôle 1 kHz prioritaire, micro-ROS)
├─ control.c         machine d'états, PID 1 kHz, homing, watchdog, arrêt d'urgence
├─ board.c           GPIO, 6 timers en mode encodeur, générateur de pas (TIM7 20 kHz), PWM servo (TIM10)
├─ pid.c             PID position -> vitesse
├─ microros_app.c    nœud micro-ROS : /joint_states, /stm32/status, /joint_commands, /emergency_stop, ...
├─ config.h          câblage, sécurité, homing
└─ robot_config_generated.h   GÉNÉRÉ depuis robot.yaml (python tools/gen_firmware_config.py)
```

## Principe de commande

* **Les moteurs pas-à-pas sont pilotés en vitesse** : le PID (1 kHz) compare la consigne à la position **mesurée par
  l'encodeur** et produit une vitesse articulaire ; celle-ci devient une **fréquence de pas** (accumulateur de phase 32 bits
  dans l'ISR TIM7 à 20 kHz → jusqu'à 10 kHz par axe, 8 kHz par défaut). C'est un pas-à-pas en **boucle fermée** : les
  pas perdus sont rattrapés, et un décrochage est détecté (écart de poursuite).
* Réducteur et micro-pas sont pris en compte dans `CFG_STEPS_PER_RAD` ; l'angle articulaire vient de l'encodeur.
* **micro-ROS n'est jamais dans la boucle de contrôle.** S'il plante, le watchdog (300 ms sans consigne **ou** sans
  heartbeat `/emergency_stop`) arrête les moteurs.
* Arrêt d'urgence = arrêt immédiat des impulsions, **verrouillé** jusqu'à `CMD_ENABLE`. Par défaut les bobines restent
  alimentées (le bras ne tombe pas) — `ESTOP_DISABLE_DRIVERS` dans `config.h`. **Prévoir un arrêt physique** coupant
  l'alimentation moteurs.
* `effort` dans `/joint_states` = commande normalisée (−1..1) : l'A4988 ne mesure pas le courant.

## Câblage (Nucleo-F446RE)

| Axe | STEP | DIR | Fin de course | Encodeur A / B |
|---|---|---|---|---|
| 1 | PC0 | PB12 | PC9  | TIM1 : PA8 / PA9 |
| 2 | PC1 | PB13 | PC10 | TIM2 : PA15 / PB3 |
| 3 | PC2 | PB14 | PC11 | TIM3 : PA6 / PA7 |
| 4 | PC3 | PB15 | PC12 | TIM4 : PB6 / PB7 |
| 5 | PC4 | PB4  | PD2  | TIM5 : PA0 / PA1 |
| 6 | PC5 | PB5  | PA4  | TIM8 : PC6 / PC7 |

* `/EN` commun des 6 A4988 : **PC8** (actif bas) · servo de pince : **PB8** (TIM10_CH1) · micro-ROS : **USART2** (USB du ST-LINK).
* Fins de course : contact entre la broche et GND (pull-up interne). Un contact **NF** (normalement fermé) est plus sûr
  (fil coupé = arrêt) : dans ce cas inverser la logique dans `board_limit_raw`.
* A4988 : `MS1=MS2=MS3=HIGH` (1/16 de pas), alimentation moteur 12-24 V avec **condensateur 100 µF** au plus près de
  chaque driver, masse commune avec la Nucleo, régler la limite de courant (Vref) **avant** de brancher les moteurs.
  Ne jamais brancher/débrancher un moteur sous tension.
* Encodeurs : **vérifier le niveau logique** (beaucoup de NEMA 17 à encodeur sortent du 5 V). Les broches STM32F4 utilisées
  sont 5 V-tolérantes pour la plupart mais vérifiez la fiche technique pour **PA0/PA1** ; sinon utiliser un adaptateur de niveau.
  Sorties à collecteur ouvert : le pull-up interne (~40 kΩ) suffit à basse vitesse, sinon ajouter des pull-up 4,7 kΩ vers 3,3 V.

## Créer le projet (STM32CubeIDE ≥ 1.13)

1. *File → New → STM32 Project* → carte **NUCLEO-F446RE** ; ne pas initialiser les périphériques par défaut.
2. `.ioc` (CubeMX) :
   * **SYS** → *Timebase Source* = **TIM6** (le SysTick est réservé à FreeRTOS) ; Debug = Serial Wire.
   * **RCC** → HSE = *BYPASS Clock Source* ; **Clock Configuration** : HCLK **180 MHz** (PLLM 4, PLLN 180, PLLP 2, APB1 /4, APB2 /2).
   * **USART2** → *Asynchronous*, **921600 bauds**, 8N1 ; *DMA Settings* : `USART2_RX` (mode **Circular**, priorité High,
     data width Byte) et `USART2_TX` (mode Normal) ; *NVIC* : USART2 global interrupt **activé**.
   * **FREERTOS** → interface **CMSIS_V2** ; *Config parameters* : `TICK_RATE_HZ` = **1000**, `TOTAL_HEAP_SIZE` ≈ **60000**
     (la RAM de 128 Ko doit contenir tas + micro-ROS), `USE_NEWLIB_REENTRANT` = Enabled, `MAX_PRIORITIES` = 7.
     Aucune tâche à créer dans CubeMX (elles sont créées par `app.c`).
   * **Ne pas** activer TIM1/2/3/4/5/7/8/10 ni les GPIO STEP/DIR/EN/fins de course : `board.c` les configure lui-même
     (accès aux registres). Project Manager → *Generate peripheral initialization as a pair of .c/.h files* coché.
3. Générer le code. Copier `firmware/Core/Inc/*` dans `Core/Inc/` et `firmware/Core/Src/*` dans `Core/Src/`
   (**remplacer** `main.c`).
4. **Générer les constantes** du robot : `python tools/gen_firmware_config.py` (met à jour `robot_config_generated.h`).

## Intégrer micro-ROS (Humble)

Suivre [micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils) (branche `humble`) :

1. Cloner le dépôt à la racine du projet CubeIDE (dossier `micro_ros_stm32cubemx_utils`).
2. Remplacer `microros_static_library/library_generation/colcon.meta` par [`firmware/microros/colcon.meta`](microros/colcon.meta)
   (mémoire minimale : 1 nœud, 2 publishers, 4 subscriptions, transport custom) — c'est un point de départ, ajuster si la
   compilation de la bibliothèque signale un manque de ressources.
3. Générer la bibliothèque (Docker) :
   `docker pull microros/micro_ros_static_library_builder:humble` puis, à la racine du projet,
   `docker run --rm -v ${PWD}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library microros/micro_ros_static_library_builder:humble`
   (adapter `MICROROS_LIBRARY_FOLDER` au dossier `microros_static_library` de l'IDE ; voir le README du dépôt).
4. *Project → Properties → C/C++ Build → Settings* :
   * *MCU GCC Compiler → Include paths* : `.../microros_static_library/libmicroros/microros_include`
   * *MCU GCC Linker → Libraries* : `microros` ; *Library search path* : `.../microros_static_library/libmicroros`
   * *Miscellaneous* (Linker) : `-u _printf_float` est facultatif.
5. Ajouter au projet `micro_ros_stm32cubemx_utils/extra_sources/*.c` (`microros_time.c`, `microros_allocators.c`) et
   `extra_sources/microros_transports/dma_transport.c`. Ces fichiers fournissent `cubemx_transport_*` et
   `microros_allocate/…` utilisés par `microros_app.c`.
6. Compiler (Release recommandé), flasher via le ST-LINK.

Vérifier :

```bash
# sur le Pi (l'agent doit tourner : pi_real.launch.py ou docker run ... serial --dev /dev/ttyACM0 -b 921600)
ros2 topic list                       # /joint_states, /stm32/status, ...
ros2 topic echo /stm32/status         # state = 1 (DÉSACTIVÉ) au démarrage
ros2 topic hz /joint_states           # ~100 Hz
```

## Interface ROS du firmware

| Topic | Type | Sens | QoS | Rôle |
|---|---|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | publie 100 Hz | best effort | positions **encodeurs**, vitesses, effort (commande normalisée) |
| `/stm32/status` | `std_msgs/Int32MultiArray` | publie 10 Hz | best effort | `[état, axes référencés, fins de course, défauts, écart max (mrad), boucle (Hz), âge consigne (ms), uptime (s)]` |
| `/joint_commands` | `sensor_msgs/JointState` | s'abonne | reliable | position + vitesse de préaction (ordre `joint_1..joint_6`) |
| `/emergency_stop` | `std_msgs/Bool` | s'abonne | reliable | `true` = arrêt verrouillé ; tout message = heartbeat |
| `/stm32/command` | `std_msgs/Int32` | s'abonne | reliable | `1` ENABLE · `2` DISABLE · `3` HOME · `4` ZERO_HERE |
| `/gripper_command` | `std_msgs/Float32` | s'abonne | reliable | `0` fermée … `1` ouverte (servo) |

Les codes d'état/défaut sont définis à l'identique dans `app_types.h` et
`robot_kinematics/status_codes.py` : **les modifier ensemble**.

## Limites connues / à valider sur le matériel

* Le firmware n'a **pas pu être compilé ni exécuté** dans l'environnement de développement (pas de chaîne ARM ni de
  carte) : il a été relu avec soin mais **la première compilation dans CubeIDE peut demander de petits ajustements**
  (noms générés par CubeMX, chemins d'inclusion micro-ROS, tailles de pile/tas).
* Le PID est réglé avec des valeurs prudentes (`kp = 6…8`, `ki = 0.5`, `kd = 0`) : à affiner axe par axe avec les courbes
  de l'IHM (voir `docs/TESTS.md`).
* Encodeur : on suppose 1000 impulsions/tour par défaut (`encoder_ppr` dans `robot.yaml`) — à ajuster.
