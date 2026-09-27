# Installation pas à pas

Trois machines : **Raspberry Pi 4** (cerveau, vision), **STM32 Nucleo-F446RE** (moteurs), **PC portable** (IHM).
Tout le monde sur le même réseau Wi-Fi et le même `ROS_DOMAIN_ID`.

> **Point important — le système du Raspberry Pi.** ROS 2 **Humble** n'est fourni en paquets officiels que pour
> **Ubuntu 22.04**. Raspberry Pi OS (Debian 12 « Bookworm ») n'est **pas** supporté par Humble. Deux options :
>
> * **A (recommandée)** : flasher **Ubuntu Server 22.04 LTS 64 bits** sur la carte SD (Raspberry Pi Imager →
>   *Other general-purpose OS → Ubuntu → Ubuntu Server 22.04 LTS (64-bit)*, régler Wi-Fi, SSH et nom d'utilisateur
>   dans les « réglages avancés »).
> * B : garder Raspberry Pi OS et exécuter ROS 2 dans Docker (`osrf/ros:humble-ros-base`, `--network host --privileged`).
>   Possible, mais l'accès à la caméra CSI depuis le conteneur est compliqué : à éviter au début.
>
> Les commandes ci-dessous supposent l'option A.

---

## 1. Raspberry Pi 4 — ROS 2 Humble

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl git
sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools \
     ros-humble-xacro ros-humble-robot-state-publisher ros-humble-tf2-ros ros-humble-diagnostic-msgs \
     ros-humble-rosbag2 ros-humble-rmw-cyclonedds-cpp ros-humble-rosbridge-server \
     python3-opencv python3-numpy python3-yaml python3-colcon-common-extensions python3-rosdep
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
sudo usermod -aG dialout $USER          # accès au port série du STM32 (se reconnecter ensuite)
```

### Espace de travail du robot

```bash
mkdir -p ~/robot_ws && cp -r <dossier du projet>/ros2_ws/src ~/robot_ws/
cd ~/robot_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys "PySide6 pyqtgraph PyOpenGL"
colcon build --symlink-install
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

Vérifier sans matériel (sur le Pi ou n'importe quelle machine) :

```bash
ros2 launch robot_bringup sim.launch.py gui:=false     # doit démarrer sans erreur
ros2 topic hz /joint_states                            # ~100 Hz
```

### Agent micro-ROS (pont UART ↔ ROS 2)

Option 1 — Docker (le plus simple) :

```bash
sudo apt install -y docker.io && sudo usermod -aG docker $USER
docker run -it --rm --net=host --privileged -v /dev:/dev \
   microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 921600
```

Option 2 — compilation native (`pi_real.launch.py` lance alors l'agent tout seul) :

```bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep install --from-paths src --ignore-src -y
colcon build && source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
echo "source ~/microros_ws/install/local_setup.bash" >> ~/.bashrc
```

Liaison Pi ↔ STM32 : par défaut **un câble USB** entre le Pi et la Nucleo (port du ST-LINK → `/dev/ttyACM0`,
alimente aussi la carte). Pour une liaison UART câblée (GPIO 14/15 du Pi ↔ PB10/PB11 avec USART3), il faut
changer d'USART dans CubeMX et dans `microros_app.c`, désactiver la console série
(`sudo systemctl disable --now serial-getty@ttyAMA0`) et ajouter `dtoverlay=disable-bt` dans
`/boot/firmware/config.txt`.

### Caméra

* **Webcam USB (le plus simple pour démarrer)** : `camera_source:=opencv`. Rien à installer.
* **Pi Camera Module 3** : le support de l'IMX708 sous **Ubuntu 22.04** demande une pile libcamera récente
  (le paquet Ubuntu est trop ancien). Il faut compiler libcamera/`picamera2` de Raspberry Pi depuis les sources
  (`https://github.com/raspberrypi/libcamera`, `pip install picamera2`), activer `dtoverlay=imx708` dans
  `/boot/firmware/config.txt`, puis utiliser `camera_source:=picamera2`. **Je n'ai pas pu tester cette étape** :
  valide d'abord que `libcamera-hello`/`picamera2` voient la caméra, sinon commence avec une webcam USB.
  La mise au point est verrouillée par le code (`lens_position`) — nécessaire pour que la calibration reste valable.

---

## 2. PC portable — IHM

### 2.a PC **Windows** (ton cas) : aucune installation de ROS 2

L'IHM se connecte au Pi par **rosbridge** (WebSocket, port 9090, lancé automatiquement par les launches) avec
`roslibpy`. Tout est décrit dans [`windows/README_WINDOWS.md`](../windows/README_WINDOWS.md) ; en résumé :

```bat
windows\install.bat                       :: une seule fois (environnement virtuel + dépendances)
windows\run_gui.bat --demo                :: essayer l'IHM SANS robot (robot simulé localement)
windows\run_gui.bat --host 192.168.1.50   :: se connecter au Pi (ou à une machine qui lance la simulation)
```

Sur le Pi, ouvrir le port 9090 **uniquement pour le PC** (`sudo ufw allow from <IP du PC> to any port 9090 proto tcp`) :
rosbridge n'a pas d'authentification. Sous Windows, **DDS/`ROS_DOMAIN_ID` ne sont pas nécessaires** : seule la
connexion TCP vers le Pi compte (ce qui évite aussi les problèmes de découverte multicast en Wi-Fi).

### 2.b PC **Ubuntu 22.04** : IHM native (ROS 2 + DDS)

```bash
# (mêmes dépôts apt que ci-dessus)
sudo apt install -y ros-humble-desktop ros-humble-xacro ros-humble-rosbag2 ros-humble-rmw-cyclonedds-cpp \
     python3-pip python3-colcon-common-extensions
pip3 install --user PySide6 pyqtgraph PyOpenGL
mkdir -p ~/robot_ws && cp -r <dossier du projet>/ros2_ws/src ~/robot_ws/
cd ~/robot_ws && colcon build --symlink-install && echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
ros2 launch robot_bringup pc_gui.launch.py            # ajouter rviz:=true pour RViz2 + robot fantôme
```

Cette variante utilise DDS directement (voir §3). Les deux variantes partagent **le même code d'IHM**.

---

## 3. Réseau, ROS_DOMAIN_ID et latence

*(Concerne l'IHM native Ubuntu et la communication entre machines ROS 2. Un PC Windows n'en a pas besoin.)*

Sur le **Pi et le PC**, dans `~/.bashrc` :

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

`~/cyclonedds.xml` (découverte en **unicast** : plus fiable que le multicast en Wi-Fi ; mettre les vraies IP) :

```xml
<?xml version="1.0"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General><AllowMulticast>false</AllowMulticast></General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="192.168.1.50"/>   <!-- Raspberry Pi -->
        <Peer address="192.168.1.60"/>   <!-- PC portable -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Conseils pour tenir **< 50 ms** :

* IP fixes (réservation DHCP), Wi-Fi 5 GHz proche du routeur ; **Ethernet Pi ↔ PC si possible** (gigue quasi nulle).
* Désactiver l'économie d'énergie Wi-Fi du Pi : `sudo iwconfig wlan0 power off`.
* Synchroniser les horloges (`sudo apt install chrony` sur les deux machines) : sans cela, la latence affichée
  dans l'IHM (« ≈ x ms ») n'est pas fiable et s'affiche « n/d ».
* La vidéo est envoyée en JPEG (`CompressedImage`), jamais en image brute.
* Vérification : `ros2 topic hz /joint_states` sur le PC (≈ 100 Hz) et `ros2 topic delay /joint_states`.

Budget de latence : encodeur → STM32 (1 ms) → micro-ROS (≤ 10 ms) → UART 921600 bauds (~2 ms) → DDS/Wi-Fi
(2-20 ms) → IHM (≤ 16 ms de rafraîchissement) ≈ 25-50 ms.

---

## 4. STM32 — compilation et flash

Voir [`firmware/README.md`](../firmware/README.md) : configuration CubeMX, intégration micro-ROS, câblage, flash.
Après toute modification de `robot.yaml`, régénérer les constantes du firmware :

```bash
python tools/gen_firmware_config.py
```

---

## 5. Premier démarrage

| Où | Commande |
|---|---|
| Simulation (une seule machine) | `ros2 launch robot_bringup sim.launch.py` |
| Pi — robot réel | `ros2 launch robot_bringup pi_real.launch.py serial_dev:=/dev/ttyACM0 camera_source:=opencv` |
| PC Windows — IHM | `windows\run_gui.bat --host <IP du Pi>` |
| PC Ubuntu — IHM native | `ros2 launch robot_bringup pc_gui.launch.py` |

Dans l'IHM : choisir le mode **Réel** (confirmation), **Activer**, puis **Homing**. Si tu n'as pas encore de
fins de course : placer le bras en pose zéro (vertical) à la main puis
`ros2 topic pub --once /stm32/command std_msgs/msg/Int32 "{data: 4}"` (`ZERO_HERE`).

Démarrage automatique sur le Pi (optionnel) : créer un service systemd qui exécute
`ros2 launch robot_bringup pi_real.launch.py` avec l'environnement de `~/.bashrc`.
