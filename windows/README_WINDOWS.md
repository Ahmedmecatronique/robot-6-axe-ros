# IHM sous Windows (sans ROS 2)

Sur Windows, **inutile d'installer ROS 2** : l'IHM se connecte au robot par **rosbridge** (un WebSocket sur le port
9090 du Raspberry Pi) avec la bibliothèque Python `roslibpy`. Tout le calcul (IK, trajectoires, sécurité, vision)
reste sur le Pi ; le PC ne fait que l'affichage et l'envoi des commandes.

```
PC Windows                                     Raspberry Pi (Ubuntu 22.04 + ROS 2 Humble)
┌──────────────────────┐   WebSocket :9090     ┌──────────────────────────────────────┐
│ IHM PySide6 + roslibpy│ ◀───────────────────▶ │ rosbridge_websocket ◀▶ nœuds ROS 2   │
└──────────────────────┘   (Wi-Fi / Ethernet)  │ fk / ik / trajectory / safety / …   │
                                               └──────────────────────────────────────┘
```

## 0. Le plus simple : double-cliquer sur `windows\demo.bat`

Au **premier lancement**, l'environnement s'installe tout seul (environ 2 minutes, connexion Internet requise), puis la
fenêtre s'ouvre en mode démo. Les lancements suivants sont immédiats. Si la fenêtre noire se referme trop vite, ouvrez
un terminal dans le dossier `windows` et tapez `.\demo.bat` pour lire le message d'erreur.

> **Dans un terminal, préfixez par `.\`** (`.\run_gui.bat --demo`) : sur certaines machines Windows la commande
> `run_gui.bat` seule est « non reconnue » car le dossier courant n'est pas dans le chemin de recherche.

## 1. Installation (facultative : `run_gui.bat` la fait seul au premier lancement)

1. Installer **Python 3.10 – 3.12** depuis python.org (cocher *Add python.exe to PATH*).
2. Double-cliquer sur **`windows\install.bat`** (ou, dans un terminal : `windows\install.bat`).
   Il crée un environnement virtuel `windows\.venv` et installe PySide6, pyqtgraph, PyOpenGL, OpenCV, roslibpy.

## 2. Essayer tout de suite, sans robot : le mode démo

```bat
windows\run_gui.bat --demo
```

Un robot simulé tourne **dans le programme** (vraie cinématique inverse, vraies trajectoires, vraies limites du
`robot.yaml`) : sliders, cartésien, arrêt d'urgence, saisie, courbes, vue 3D. Variante qui démarre désactivé pour
tester *Activer → Homing* : `windows\run_gui.bat --demo-unhomed`. Le mode démo **n'est pas** le robot réel
(pas de rosbag, caméra factice).

## 3. Se connecter au robot ou à la simulation

Côté robot (Pi, ou une machine Ubuntu qui lance la simulation), `rosbridge` démarre avec les launches
(`core.launch.py`, port **9090**). Installer une fois : `sudo apt install ros-humble-rosbridge-server`.

```bash
# sur le Pi
ros2 launch robot_bringup pi_real.launch.py            # robot réel
# ou, pour tester la chaîne complète sans matériel (sur le Pi ou une machine Ubuntu) :
ros2 launch robot_bringup sim.launch.py gui:=false
```

Sur le PC :

```bat
windows\run_gui.bat --host 192.168.1.50                :: IP du Raspberry Pi
windows\run_gui.bat                                    :: demande l'adresse (mémorisée)
```

La barre d'état affiche `● STM32 : connecté 100 Hz`. Si la connexion tombe, l'IHM se reconnecte toute seule et
affiche une alarme.

## 4. Sécurité et réseau

* **rosbridge n'a aucune authentification** : quiconque atteint le port 9090 peut piloter le robot. Ne l'exposez
  jamais à Internet ; limitez-le au PC, par exemple sur le Pi :
  `sudo ufw allow from <IP du PC> to any port 9090 proto tcp && sudo ufw enable`.
* L'arrêt d'urgence de l'IHM passe par le réseau : **il ne remplace pas un arrêt physique** (bouton coupant
  l'alimentation moteurs). Le firmware s'arrête de lui-même si la liaison ou le heartbeat disparaît (watchdog 300 ms).
* Pare-feu Windows : la connexion est **sortante** (PC → Pi:9090), rien à ouvrir côté Windows.
* Wi-Fi : IP fixes, 5 GHz, économie d'énergie désactivée sur le Pi (`sudo iwconfig wlan0 power off`) ; Ethernet
  est plus régulier. Pour une latence affichée fiable, synchroniser les horloges (Windows : *Paramètres → Heure et
  langue → Synchroniser maintenant* ; Pi : `chrony`).

## 5. Dépannage

| Symptôme | Cause probable |
|---|---|
| « Liaison rosbridge perdue » dès le lancement | Pi injoignable (`ping`), `rosbridge_websocket` non lancé, mauvais port, pare-feu du Pi |
| Connecté mais aucune donnée (`/joint_states` vide) | Le STM32 / `sim_robot_node` ne publie pas ; `ros2 topic hz /joint_states` sur le Pi |
| Connecté, le badge reste « MODE ? » | `safety_node` non lancé (publie `/robot_mode`) |
| Données reçues mais pas de flux `/joint_states` du STM32 | rosbridge (best effort ≠ reliable) : `ros2 topic info /joint_states -v` ; mettre à jour `ros-humble-rosbridge-server` (les versions récentes adaptent la QoS automatiquement) |
| Vue 3D noire | pilote graphique sans OpenGL 3 ; mettre à jour le pilote (NVIDIA/Intel) |
| `install.bat` : « Python est introuvable » | Python absent du PATH : réinstaller en cochant *Add to PATH* ; si `python` ouvre le Microsoft Store, désactiver l'alias (*Paramètres → Applications → Alias d'exécution*) |
| « `run_gui.bat` n'est pas reconnu » | taper `.\run_gui.bat` (avec `.\`) ou double-cliquer |
| L'installation échoue | connexion Internet ; supprimer le dossier `windows\.venv` puis relancer |

Le fichier de configuration du robot (`ros2_ws\src\robot_description\config\robot.yaml`) est lu **directement** par
l'IHM Windows : après l'avoir modifié, relancer l'IHM suffit (mais le Pi doit avoir la même version : recopier le
dossier `ros2_ws`, `colcon build`).
