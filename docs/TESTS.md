# Plan de tests

Progression : **simulation d'abord**, puis banc STM32 **sans moteurs**, puis **axe par axe** via l'IHM, puis la boucle
complète **vision → saisie**. On ne passe à l'étape suivante que si la précédente est validée.

Règles de sécurité pour toutes les étapes matérielles :
* **bouton d'arrêt physique** coupant l'alimentation moteurs à portée de main (l'arrêt logiciel garde le couple de maintien) ;
* vitesse réduite au début (`speed_scale: 0.1` dans `params.yaml`, ou curseur « Vitesse » de l'IHM) ;
* zone dégagée, personne dans le rayon d'action du bras (portée ≈ 0,6 m + outil).

---

## Étape 0 — Tests unitaires (aucun matériel, aucun ROS)

```bash
cd ros2_ws/src/robot_kinematics && python -m pytest tests -q          # 21 tests : FK/IK, singularités, trajectoires
cd ../robot_vision && PYTHONPATH=../robot_kinematics:. python -m pytest tests -q   # 10 tests : ArUco, HSV, projection, main-œil
cd ../robot_gui && QT_QPA_PLATFORM=offscreen PYTHONPATH=../robot_kinematics:. python -m pytest tests -q   # 30 tests : IHM, WebBridge (faux rosbridge), mode démo
# Total : 61 tests (21 + 10 + 30)
# (nécessite PySide6, pyqtgraph, PyOpenGL, roslibpy, websockets, opencv ; sous Windows : windows\.venv après install.bat)
```

Mesure de performance de l'IHM (compare deux versions avec `--json`) :

```bash
python tools/bench_gui.py --seconds 10        # boucle d'affichage : moyenne / p95 / max, coût par composant, CPU
```

Repère (v1.1, hors écran) : boucle ≈ 6 ms en moyenne, p95 ≈ 16 ms (budget 16 ms = 60 images/s). Sur ta machine,
l'indicateur « IHM : x img/s · y ms » de la barre d'état donne la mesure en direct (rendu GPU inclus).

Critères : tout est vert. Le test clé est `FK(IK(pose)) == pose` (tolérance 1 µm / 1 µrad) sur 200 poses aléatoires,
pour **toutes** les configurations analytiques (jusqu'à 8), plus le choix de la solution la plus proche de l'état actuel.

## Étape 0 bis — PC Windows : l'IHM sans robot

`windows\run_gui.bat --demo` puis vérifier : sliders, cartésien (atteignable / inatteignable / singulier), arrêt d'urgence (bloque tout jusqu'à « Activer »), saisie (« Aller saisir l'objet détecté » dans l'onglet Caméra), courbes, vue 3D (robot plein + fantôme). `--demo-unhomed` pour *Activer → Homing*.

## Étape 1 — Modèle du robot dans RViz2

```bash
ros2 launch robot_description view_robot.launch.py
```

| Vérification | Attendu |
|---|---|
| `q = 0` (tous sliders à 0) | bras **vertical**, centre du poignet à `d1 + a2 + a3` = 0,90 m |
| Bouger chaque slider | l'axe tourne dans le bon sens et autour du bon axe |
| `ros2 launch robot_bringup sim.launch.py rviz:=true gui:=false` puis afficher TF | les repères `dh_1..dh_6`, `tool_fk` (nos calculs) **coïncident** avec `link_*`, `tool0` (URDF) |
| Modifier `d1/a2/a3/d6` dans `robot.yaml`, relancer | le modèle et l'IK suivent (une seule source de vérité) |

## Étape 2 — Simulation complète avec l'IHM (sans matériel)

```bash
ros2 launch robot_bringup sim.launch.py            # ajouter start_ready:=false pour tester Activer + Homing
```

| # | Test | Résultat attendu |
|---|---|---|
| 2.1 | Le badge affiche **SIMULATION** (vert) ; l'IHM attend « Simulation » | commandes actives |
| 2.2 | Choisir « Réel » dans l'IHM alors que le système est en simulation | bandeau rouge, **commandes bloquées** ; l'arrêt d'urgence reste actif |
| 2.3 | Sliders J1..J6 (relâcher) | robot **plein** suit, robot **fantôme** (consigne) devance ; écart affiché ; latence de rendu < 20 ms |
| 2.4 | Cartésien : X 250, Y 0, Z 400, R 0, P 180, Y 0 → « Calculer IK » puis « Aller » | ✔ atteignable, le robot va à la pose |
| 2.5 | Cartésien : X 900 (hors portée) | ✖ « Pose inatteignable … portée max 0.600 m » |
| 2.6 | Pose avec poignet aligné (q5 ≈ 0) | ⚠ « proche d'une singularité (poignet…) » |
| 2.7 | Cible sous `z_min` (Z 10 mm) | ✖ refusée (plan de sécurité) |
| 2.8 | `start_ready:=false` : Activer → Homing | états DÉSACTIVÉ → NON RÉFÉRENCÉ → HOMING → PRÊT ; mouvements bloqués avant PRÊT |
| 2.9 | **ARRÊT D'URGENCE** pendant un mouvement | arrêt immédiat, état ARRÊT D'URGENCE, alarme ; mouvements impossibles ; « Activer » réarme |
| 2.10 | `ros2 node kill`/Ctrl-C de `trajectory_node` en PRÊT | après ≈ 0,3 s : **watchdog** → état DÉFAUT « perte de communication » |
| 2.11 | Onglet Dashboard : courbes consigne/mesure, tableau, alarmes, fréquence liaison ≈ 100 Hz | cohérents |
| 2.12 | Enregistrer une session, la rejouer | l'IHM passe en **REJEU**, commandes désactivées, le robot réel ne reçoit rien |
| 2.13 | Mode apprentissage : 3 poses, séquence en boucle, Enregistrer/Charger | la séquence se rejoue, arrêt propre |
| 2.15 | **PC Windows** : `windows\run_gui.bat --host <IP>` vers cette simulation | mêmes comportements qu'en 2.1-2.13 (le rosbag est piloté côté robot par `bag_node`) ; `ros2 topic hz /joint_states` sur le Pi ≈ 100 Hz |
| 2.14 | Vision avec webcam : marqueur ArUco → `/target_pose`, flux annoté dans l'onglet Caméra, bouton « Aller saisir » | phases affichées, la simulation exécute approche/descente/remontée |

## Étape 3 — Banc STM32, moteurs **non alimentés**

Flasher le firmware ; câbler seulement USB + (éventuellement) encodeurs et fins de course.

| # | Test | Attendu |
|---|---|---|
| 3.1 | `ros2 launch robot_bringup pi_real.launch.py vision:=false` | l'agent voit le STM32 ; `ros2 topic list` montre `/joint_states`, `/stm32/status` |
| 3.2 | `ros2 topic hz /joint_states` | ≈ 100 Hz ; l'IHM affiche « connecté » |
| 3.3 | Tourner un encodeur à la main | l'axe correspondant bouge **dans le bon sens** dans l'IHM (sinon `encoder_sign`) |
| 3.4 | Appuyer sur chaque fin de course | le bit correspondant apparaît dans « fins de course actifs » (contact vers GND) |
| 3.5 | Débrancher l'USB / tuer l'agent | l'IHM affiche « déconnecté » ; à la reconnexion le STM32 se ré-enregistre tout seul |
| 3.6 | `ros2 topic pub -r 10 /emergency_stop std_msgs/msg/Bool "{data: true}"` | état ARRÊT D'URGENCE dans `/stm32/status` |

## Étape 4 — Axe par axe (moteurs alimentés, bras **libre de toute collision**)

Régler d'abord le courant des A4988 (Vref selon la résistance de mesure de votre module) et vérifier `MS1=MS2=MS3=HIGH`
(1/16). Un axe à la fois, `speed_scale: 0.1`, arrêt physique à portée.

1. **Sens** : activer, `CMD_ZERO_HERE`, petit déplacement (+5°) d'un axe. Si l'axe part dans le mauvais sens et que
   l'écart de poursuite grandit → défaut « écart de poursuite » (normal) : inverser `dir_sign` (ou `encoder_sign` si
   l'encodeur compte à l'envers), relancer `python tools/gen_firmware_config.py`, reflasher.
2. **Homing** de l'axe : approche, recul lent, référence au front de relâchement. Un homing qui échoue signale
   « échec du homing (axe n) » : moteur bloqué, encodeur débranché ou polarité inversée (contrôle de progression).
3. **Précision** : aller à ±10°, ±45° ; comparer l'angle affiché et l'angle réel (rapporteur).
4. **Réglage PID** (onglet Dashboard, courbes consigne/mesure/erreur de l'axe choisi) : commencer par `kp` seul
   (`ki = kd = 0`) ; augmenter jusqu'à un suivi rapide sans oscillation, puis `ki` faible pour annuler l'erreur
   statique ; `kd` rarement utile avec des pas-à-pas. Un pas-à-pas est un intégrateur : un P suffit souvent.
5. **Perte de pas** : si l'écart de poursuite monte au-delà de `following_error_max_deg` pendant 200 ms, le firmware
   arrête tout (défaut) — réduire vitesse/accélération dans `robot.yaml`.
6. **Watchdog matériel** : arrêter `trajectory_node` en mouvement : les moteurs s'arrêtent en < 0,4 s.

## Étape 5 — Vision seule (bras immobile)

1. Calibrations caméra puis main-œil (voir `CALIBRATION.md`).
2. Marqueur/objet à une position connue : `/target_pose` correct à quelques millimètres près.
3. Cible qui bouge : lissage stable, pas de saut ; cible masquée : `target_visible = false`.

## Étape 6 — Boucle complète vision → saisie

Objet léger sur le plan de travail, vitesse réduite, pince en place :
1. la caméra voit l'objet, l'IHM affiche ses coordonnées 3D ;
2. « Aller saisir l'objet détecté » : ouverture, approche (10 cm au-dessus), descente, fermeture, remontée, repos ;
3. annuler en cours de route (bouton) et arrêt d'urgence : arrêt propre ;
4. répéter avec des objets à différentes positions ; noter l'écart de positionnement.

## Checklist avant chaque session réelle

- [ ] arrêt d'urgence physique testé
- [ ] `ROS_DOMAIN_ID` identique, IHM en mode **Réel** confirmé
- [ ] homing OK, état PRÊT
- [ ] vitesse réduite pour le premier mouvement
- [ ] zone de travail dégagée
