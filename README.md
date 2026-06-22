# amr2ax_nav2

![ROS2](https://img.shields.io/badge/ROS%202-Jazzy-blue)
![Nav2](https://img.shields.io/badge/Nav2-Jazzy-purple)
![Platform](https://img.shields.io/badge/Platform-NOUZEN%204WD-red)
![Build](https://img.shields.io/badge/Build-ament__cmake-yellow)
![License](https://img.shields.io/badge/License-Apache%202.0-green)

Pachet ROS 2 cu **stiva completă de navigație autonomă și orchestrare a misiunilor intralogistice** pentru robotul mobil **NOUZEN**. Include configurația Nav2 (planner, controller, costmaps), Behavior Trees personalizate, parametrii pentru AMCL, integrarea cu `opennav_docking` și detecția AprilTag, harta laboratorului, precum și un set complet de scripturi Python pentru coordonarea misiunilor de transport între stații.

## Rol în arhitectura NOUZEN

Acest pachet se montează deasupra [`nouzen_bringup`](https://github.com/AndyD01/nouzen_bringup) și asigură:

1. **Localizarea** robotului pe o hartă pre-construită (AMCL) sau construirea unei hărți noi (SLAM Toolbox)
2. **Navigația autonomă** prin stiva Nav2 (planner hibrid, MPPI controller, recovery behaviors)
3. **Andocarea de precizie** la stații de încărcare/preluare prin `opennav_docking` cu detecție vizuală AprilTag
4. **Orchestrarea misiunilor** intralogistice: rutare inteligentă, executare secvențială cu retry, logare structurată, dashboard de monitorizare

## Structura pachetului

```
amr2ax_nav2/
├── behavior_trees/
│   ├── amr2ax_indoor_bt.xml          # BT principal pentru navigație indoor NOUZEN
│   ├── battery_aware_navigation.xml  # BT cu integrare nivel baterie
│   ├── honeybee_gps_bt.xml           # variantă alternativă (outdoor / GPS)
│   └── honeybee_urban_bt.xml         # variantă alternativă (urban)
├── config/
│   ├── xplorer_v2.yaml               # fișierul principal Nav2: planner, controller, costmaps, AMCL, docking
│   ├── slam_toolbox.yaml             # parametri SLAM Toolbox (mapping)
│   ├── 3d_localization.yaml          # parametri localizare 3D (variantă)
│   ├── apriltag_params.yaml          # parametri apriltag_ros (familia, dimensiuni, ID-uri)
│   ├── dock_database.yaml            # definițiile dock-urilor pentru opennav_docking
│   ├── station_config.yaml           # configurația stațiilor intralogistice
│   ├── mission.yaml                  # parametri executor misiuni
│   ├── waypoints.yaml                # puncte de trecere globale
│   └── navigation.rviz               # layout RViz pentru vizualizare
├── launch/
│   ├── include/
│   │   ├── amcl.launch.py
│   │   ├── slam_toolbox.launch.py
│   │   ├── navigation.launch.py
│   │   └── 3d_localization.launch.py
│   ├── nav2.launch.py                          # Nav2 standalone
│   ├── localization.launch.py                  # AMCL + map_server
│   ├── slam_launch.py                          # SLAM Toolbox pentru construire hartă
│   ├── navxplorer_with_docking_v2.launch.py    # Nav2 + opennav_docking + AprilTag
│   ├── Dispatcher_stack.launch.py              # stiva completă de orchestrare misiuni
│   └── rviz.launch.py
├── maps/
│   ├── holcb2024cb202_edited.pgm     # harta laboratorului
│   └── holcb2024cb202_edited.yaml
├── scripts/
│   ├── station.py                    # clasa Station (poziție, dock, atribute)
│   ├── station_manager.py            # registru și acces concurent la stații
│   ├── mission_executor_v2.py        # executor misiuni cu retry + verificare AMCL
│   ├── dispatcher.py                 # planificator misiuni la nivel înalt
│   ├── inject.py                     # injectare misiuni externe în coadă
│   ├── dashboard.py                  # interfață curses de monitorizare
│   └── generate_map.py               # utilitar generare/editare hărți
├── logs/                             # output runtime (.log + .csv), gitignored
├── CMakeLists.txt
├── package.xml
├── LICENSE
└── README.md
```

## Configurația Nav2 (`xplorer_v2.yaml`)

Fișierul principal de parametri, care acoperă întreaga stivă Nav2 + opennav_docking.

### Localizare (AMCL)

| Parametru | Valoare |
|-----------|---------|
| `robot_model_type` | `DifferentialMotionModel` |
| `max_particles` | 3500 |
| Hartă de referință | `holcb2024cb202_edited` |

### Planificator global (SmacPlannerHybrid)

| Parametru | Valoare |
|-----------|---------|
| `motion_model` | `REEDS_SHEPP` |
| `minimum_turning_radius` | 0.37 m |
| `reverse_penalty` | 3.0 |
| `analytic_expansion_ratio` | 0.5 |

### Controller local (MPPI)

| Parametru | Valoare |
|-----------|---------|
| `batch_size` | 500 |
| Frecvență evaluare | dinamică, în funcție de constraint-urile cinematice |

### Andocare (opennav_docking)

Plugin-uri folosite:
- `SimpleChargingDock` pentru stații cu încărcare
- `SimpleNonChargingDock` pentru stații de transfer

Parametri cheie pentru detecția vizuală:
- `external_detection_translation_x` = **-0.35** m (compensare distanță cameră - centru tag)
- `filter_coef` = 0.1
- `docking_threshold` = 0.10
- `staging_x_offset` = -0.80 m
- `k_phi` = 2.2, `k_delta` = 1.0 (gain-uri controller andocare)

## Behavior Trees

Behavior Tree-urile sunt fișiere XML care definesc fluxul decizional al Nav2 pentru fiecare acțiune de navigație. NOUZEN folosește în mod implicit `amr2ax_indoor_bt.xml`, optimizat pentru medii intralogistice indoor (recovery behaviors agresive, replanning frecvent).

`battery_aware_navigation.xml` extinde BT-ul standard cu noduri condiționale care evaluează nivelul bateriei și pot redirecționa robotul către o stație de încărcare. BT-urile `honeybee_*` sunt variante alternative păstrate ca referință pentru scenarii outdoor.

## Detecția AprilTag

Stațiile de andocare sunt marcate cu cinci marker-e AprilTag:

| Parametru | Valoare |
|-----------|---------|
| Familie | `tag36h11` |
| Dimensiune fizică | 150 mm |
| ID-uri | 1, 2, 3, 4, 5 |
| Pachet detecție | `apriltag_ros` |

`dock_pose_publisher` (parte din [`nouzen_bringup`](https://github.com/AndyD01/nouzen_bringup)) republică detecția curentă pe `/detected_dock_pose`, iar `dock_database.yaml` mapează fiecare ID de tag pe o poziție de andocare.

## Stiva de orchestrare a misiunilor (`scripts/`)

Componenta originală a acestui pachet, care transformă Nav2 dintr-un navigator individual într-un **sistem complet de execuție a sarcinilor intralogistice**.

| Modul | Rol |
|-------|-----|
| `station.py` | Clasa `Station`: nume, pose de aproach, ID dock, stare, atribute custom |
| `station_manager.py` | Registru centralizat al stațiilor, citire `station_config.yaml`, acces thread-safe |
| `mission_executor_v2.py` | Rulează o misiune pas cu pas: `navigate_to_pose` → `dock_robot` → operațiune locală → `undock_robot`. Retry cu backoff, verificare poziție AMCL înainte de fiecare segment |
| `dispatcher.py` | Planificator de nivel înalt: primește cereri și le distribuie executor-ilor disponibili, gestionează coada |
| `inject.py` | Utilitar CLI pentru injectare manuală de misiuni în dispatcher (testare și debugging) |
| `dashboard.py` | Interfață curses care afișează starea robotului, stația curentă, misiunea activă, log-urile recente |
| `generate_map.py` | Utilitar pentru generare / re-editare hărți după rularea SLAM Toolbox |

### Notă tehnică

Executor-ul de misiuni rulează acțiunile Nav2 pe un **thread separat**, cu helper de busy-wait pentru spin, pentru a evita `RuntimeError: executor already spinning`. La rulare ca nod ROS, `sys.argv` este filtrat înainte de `argparse` pentru a evita conflictele cu argumentele de launch.

### Logare

Fiecare execuție produce două fișiere în `logs/`:
- `.log` text uman-citibil cu timeline-ul evenimentelor
- `.csv` structurat pentru analiză statistică (timpi de segment, rezultate, retry-uri)

## Cum se compilează

```bash
cd ~/ros2_ws/src
git clone https://github.com/AndyD01/amr2ax_nav2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select amr2ax_nav2 --symlink-install
source install/setup.bash
```

## Cum se pornește

> Necesită ca [`nouzen_bringup`](https://github.com/AndyD01/nouzen_bringup) să ruleze deja (drivetrain + senzori).

### Construire hartă nouă (SLAM)

```bash
ros2 launch amr2ax_nav2 slam_launch.py
# după ce harta arată bine:
ros2 run nav2_map_server map_saver_cli -f ~/maps/harta_noua
```

### Navigație cu hartă existentă (AMCL)

```bash
ros2 launch amr2ax_nav2 localization.launch.py
ros2 launch amr2ax_nav2 nav2.launch.py
```

### Navigație + andocare AprilTag

```bash
ros2 launch amr2ax_nav2 navxplorer_with_docking_v2.launch.py
```

### Stiva completă cu dispatcher de misiuni

```bash
ros2 launch amr2ax_nav2 Dispatcher_stack.launch.py
```

### Monitorizare în timp real

```bash
ros2 run amr2ax_nav2 dashboard.py
```

### Injectare misiune manuală

```bash
ros2 run amr2ax_nav2 inject.py --from STATION_A --to STATION_B
```

## Dependențe principale

| Pachet | Sursă |
|--------|-------|
| `nav2_bringup`, `nav2_*` | `apt: ros-jazzy-navigation2` |
| `slam_toolbox` | `apt: ros-jazzy-slam-toolbox` |
| `opennav_docking` | `apt: ros-jazzy-opennav-docking` (sau din sursă) |
| `apriltag_ros` | `apt: ros-jazzy-apriltag-ros` |
| `robot_localization` | `apt: ros-jazzy-robot-localization` |
| Python: `numpy`, `pyyaml` | `pip` sau `apt` |

## Validare experimentală

Stiva a fost validată în condiții reale, cu o rată de succes la andocare de **85.2%** pe parcursul a **54 de încercări**, distribuite pe cele 5 stații. Intervalul de încredere Wilson la 95% este [73.4%, 92.3%]. Diferențele între stații sunt semnificative statistic (Kruskal-Wallis H = 23.41, p = 0.0001).

## Licență

Distribuit sub licența **Apache License 2.0**. Vezi fișierul [LICENSE](LICENSE) pentru detalii complete.

## Context academic

Proiect dezvoltat în cadrul lucrării de licență privind un sistem autonom de navigație și andocare pentru robot mobil diferențial în scenarii intralogistice, la **Facultatea de Inginerie Industrială și Robotică (FIIR), Universitatea Politehnica din București**, specializarea **Informatică Aplicată în Inginerie Industrială (IAII)**, sub coordonarea **Conf. Dr. Ing. Bogdan-Felician Abaza**.
