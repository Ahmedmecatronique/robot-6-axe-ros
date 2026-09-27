# Journal des changements

## v1.1 — IHM plus performante (côté PC)

Objectif : une IHM plus fluide et plus réactive, avec une **latence réelle** mesurée. Chaque optimisation a été
choisie d'après une **mesure** (`tools/bench_gui.py`), pas au jugé.

### Mesures (robot simulé à 100 Hz, boucle d'affichage à 60 Hz, hors écran, 10 s)

| | v1.0 | v1.1 |
|---|---|---|
| Boucle d'affichage — moyenne | 8,1 ms | **6,0 – 6,3 ms** (−24 %) |
| Boucle d'affichage — p95 | 20,9 ms *(dépasse le budget de 16 ms)* | **15,8 – 15,9 ms** *(dans le budget)* |
| CPU du processus | ≈ 60 % d'un cœur | **44 – 47 %** |
| Historique (`history()`) | 0,49 ms | **0,03 ms** (−94 %) |
| Caméra, onglet masqué | 2,38 ms | **0,18 ms** (−92 %) |
| Courbes | 1,0 ms | **0,55 ms** (−45 %) |
| Vue 3D | 0,80 ms | 0,60 ms |

Limites de la mesure : elle est faite hors écran, donc **le dessin OpenGL par le GPU n'est pas mesuré** (seul le
travail Python/Qt qui bloque le fil d'interface l'est). Les pics ponctuels (≈ 40-65 ms) ne sont pas éliminés : ils
viennent en grande partie de la peinture Qt et du fil du robot simulé qui partage le processus. À revalider sur ta
machine avec `windows\run_gui.bat --demo` (l'indicateur « IHM : x img/s · y ms » de la barre d'état donne la mesure
en direct).

### Optimisations
* **Historique en tampon anneau NumPy** (écriture en O(1), lecture par tranche sans réordonnancement) au lieu d'une
  liste de tuples reconvertie en tableaux à chaque rafraîchissement des courbes.
* **Onglets masqués = aucun travail** : plus de décodage/redimensionnement JPEG de la caméra ni de tracé des courbes
  quand l'onglet n'est pas affiché (les **alarmes restent évaluées**).
* **Vue 3D** : cinématique directe calculée **une seule fois** par image (avant : deux fois) ; le robot n'est
  redessiné que si sa pose a changé.
* **Caméra** : redimensionnement par OpenCV (`INTER_AREA`) à la taille d'affichage au lieu de la mise à l'échelle
  « smooth » de Qt.
* **Courbes** : sous-échantillonnage automatique (`peak`), découpage à la zone visible, pas de contrôle de valeurs
  finies, plage X mise à jour seulement si elle change ; **tableau** : une cellule n'est repeinte que si son texte change.
* **Cadences décalées** (courbes 15 Hz, caméra 15 Hz, tableau 10 Hz, sur des images différentes) pour ne jamais
  cumuler tous les travaux sur la même image.
* **rosbridge (Windows)** : flux rapides limités à **50 Hz** par défaut (`--rate`, 0 = tous) : deux fois moins de JSON à
  décoder, sans effet visible à l'écran.

### Nouveautés
* **Latence réelle par ping/pong** : l'IHM publie `/gui/ping` deux fois par seconde, `safety_node` le renvoie sur
  `/gui/pong` ; la barre d'état affiche l'**aller-retour** (vert < 100 ms, orange < 250 ms, rouge au-delà). Contrairement
  à l'ancienne latence, elle **ne dépend pas de la synchronisation des horloges** du PC et du Pi.
* **Indicateurs de performance** dans la barre d'état : images/s et coût moyen d'une itération de la boucle d'affichage.
* **Banc de mesure** `tools/bench_gui.py` (`--json` pour comparer deux versions).

### Tests
IHM : 23 → **30** (tampon anneau : ordre, bouclage, fenêtre, copies ; ping/pong direct et à travers le faux
rosbridge). Cinématique : 21 (inchangés, `link_frames` accepte désormais des repères déjà calculés).

### Ce qui n'a PAS été fait / à surveiller
* Pas de fusion des maillages 3D (moins d'appels de dessin) : gain à mesurer sur un vrai GPU, et impossible à
  vérifier visuellement ici.
* `safety_node` (côté Pi) doit être mis à jour pour répondre au ping ; sans lui l'IHM affiche l'ancienne latence.
* Rien de tout cela n'a été mesuré avec le vrai rosbridge ni sur le matériel.

## v1.0 — Première version complète
Workspace ROS 2 (7 paquets), firmware STM32, IHM (ROS 2 natif, rosbridge/Windows, démo), documentation, 54 tests.
