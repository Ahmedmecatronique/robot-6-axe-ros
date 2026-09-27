# Calibration de la caméra et calibration main-œil (hand-eye)

Deux calibrations, à faire **dans cet ordre**, avec la **même résolution et la même mise au point** qu'en
fonctionnement (la Pi Camera v3 a un autofocus : le code le désactive et fixe `lens_position` ; ne le changez plus
après la calibration).

| Fichier produit | Contenu | Utilisé par |
|---|---|---|
| `robot_vision/config/camera.yaml` | matrice intrinsèque K + distorsion | `vision_node` |
| `robot_vision/config/hand_eye.yaml` | pose de la caméra dans `base_link` | `vision_node` (position 3D) + TF statique |

Tant que `calibrated: false`, `vision_node` affiche un avertissement : les positions 3D ne sont **pas fiables**.

---

## 1. Calibration intrinsèque (échiquier)

**Matériel** : un échiquier imprimé et collé sur une surface **plane rigide** (carton-plume, plaque). Compter les
coins *intérieurs* (échiquier de 10×7 cases → 9×6 coins) et mesurer précisément le côté d'une case (par ex. 25 mm).

1. Fixer la caméra dans sa position de travail, éclairage uniforme, pas de reflet sur la mire.
2. Capture automatique (sur le Pi, sans écran ; une image est enregistrée seulement si l'échiquier est détecté) :

   ```bash
   ros2 run robot_vision camera_calibration capture --source picamera2 --out ~/calib --count 25 --cols 9 --rows 6
   # webcam USB :  --source opencv --device 0
   ```

   Pendant la capture, **déplacer et incliner la mire** : coins et bords de l'image, plusieurs distances, plusieurs
   inclinaisons (±30°). Ne pas la tenir parfaitement parallèle à la caméra.
3. Calcul :

   ```bash
   ros2 run robot_vision camera_calibration calibrate --images ~/calib --out camera.yaml --cols 9 --rows 6 --square 0.025
   ```

   Lire l'**erreur de reprojection** : **< 0.5 px** bon, 0.5-1 px acceptable, **> 1 px : refaire** (images floues, mire
   non plane, mauvais côté de case…).
4. Copier `camera.yaml` dans `ros2_ws/src/robot_vision/config/` puis `colcon build --symlink-install`.

---

## 2. Calibration main-œil (caméra fixe → repère du robot)

Configuration **eye-to-hand** : la caméra est **fixe** et regarde la zone de travail. On colle un **marqueur ArUco** sur
l'outil (bride) : c'est lui qui relie la vision au robot.

**Principe.** Le marqueur est rigide par rapport à l'outil. Pour chaque pose du robot on connaît

* `T_base_outil` : la cinématique directe (`/end_effector_pose`, issue des **encodeurs**) ;
* `T_cam_marqueur` : la vision (`/vision/marker_in_camera`, `solvePnP`).

Avec `T_base_cam · T_cam_marqueur = T_base_outil · T_outil_marqueur` (`T_outil_marqueur` inconnue mais constante), on
obtient un problème `AX = XB` résolu par `cv2.calibrateHandEye` (méthode de Park). Le résultat est `T_base_cam`.
Le calcul est **validé par un test unitaire** sur des données synthétiques (`test_hand_eye_eye_to_hand_recovers_camera_pose`).

**Préparation**

1. Imprimer un marqueur ArUco (dictionnaire `DICT_4X4_50`, ex. id 0) et **mesurer le côté du carré noir** (par ex.
   40 mm → `marker_length: 0.04` dans `params.yaml`). Une erreur de 1 mm sur 40 mm fausse la distance de ~2,5 %.
2. Le coller **à plat et rigidement** sur l'outil (pas de jeu, pas de papier qui gondole).
3. Robot **référencé** (état PRÊT), `vision_node` lancé avec le détecteur `aruco` et `target_id` = id du marqueur
   (`camera_source:=…` selon la caméra).

**Acquisition**

```bash
ros2 run robot_vision hand_eye_calibration --out hand_eye.yaml
```

Pour **chaque pose** : amener le robot avec l'IHM (sliders) à une position où le marqueur est **bien visible et
immobile**, puis appuyer sur **Entrée**. Le script moyenne 15 mesures et refuse la pose si le marqueur bouge
(> 2 mm de bruit).

* **12 à 20 poses**, **très variées** : différentes positions dans le champ **et surtout différentes orientations du
  poignet** (rotations autour de plusieurs axes). Des poses toutes parallèles donnent un résultat dégénéré.
* Le marqueur doit rester visible et à peu près face à la caméra (± 60°).

Taper `c` : calcul + enregistrement. Le script affiche la **dispersion de `T_outil_marqueur`** (constante en théorie) :

| Dispersion | Verdict |
|---|---|
| < 5 mm et < 3° | bon |
| 5-10 mm | moyen |
| > 10 mm | mauvais : refaire avec des poses plus variées ; vérifier `marker_length`, la calibration intrinsèque et le zéro des encodeurs |

Copier `hand_eye.yaml` dans `ros2_ws/src/robot_vision/config/`, rebuild. Dans RViz2 le repère `camera_optical_frame`
doit apparaître au bon endroit (TF statique diffusé par `vision_node`).

**Vérification** : poser un marqueur (ou un objet) à un endroit connu, lire `/target_pose`, comparer avec la position
mesurée à la règle ; l'erreur doit être de l'ordre de 5 mm. Sinon, revoir les étapes ci-dessus.

---

## 3. Détection par couleur (HSV) — réglages

`detector:=hsv` localise un objet sans profondeur, en supposant qu'il **repose sur un plan connu** : régler
`plane_z` (hauteur du **centre** de l'objet dans `base_link`, en mètres) et `hsv_lower/hsv_upper` (OpenCV : H 0-179,
S/V 0-255 ; rouge = `[170,120,70]` → `[10,255,255]`, l'intervalle de teinte « enjambe » 0). Pour trouver les bornes,
observer le flux `/camera/image_annotated` dans l'IHM et ajuster à chaud :
`ros2 param set /vision_node hsv_lower "[100,120,70]"`.

## 4. Astuces qui changent tout

* Lumière diffuse et constante ; éviter le soleil direct et les reflets sur le marqueur (papier **mat**).
* Un marqueur plus grand = pose plus précise (viser 4-6 cm à ~50 cm).
* Toute modification de la caméra (déplacement, focale, résolution) impose de **refaire** les deux calibrations.
