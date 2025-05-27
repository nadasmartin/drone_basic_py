# Drón szimulációs környezet készítése, lokalizáció és autonóm drónirányítás fejlesztése
### Robotrendszerek laboratórium (BMEGEMINMRL) házi feladat dokumentáció
### Balázs Miklós, Nádas Gergely Martin
<video controls src="docs/RoboLab_hazi_video.mp4" title="Title"></video>

## Tartalomjegyzék

1. [Install and setup](#install-and-setup)
   - [Prerequisites](#prerequisites)
   - [Git Repo klónozása](#git-repo-klónozása)
2. [Usage](#usage)
   - [Szimuláció indítása](#szimuláció-indítása)
   - [Teleop node indítása](#teleop-node-indítása)
3. [Drone modell](#drone-modell)
   - [Drone Mesh](#drone-mesh)
   - [Drone URDF](#drone-urdf)
     - [Szerkezeti felépítés](#szerkezeti-felépítés)
       - [Törzs (base_link)](#törzs-base_link)
       - [Rotorok](#rotorok)
     - [Szenzorok](#szenzorok)
       - [Kamera](#kamera)
       - [IMU](#imu)
       - [GPS](#gps)
     - [Kiterjesztések és sablonok](#kiterjesztések-és-sablonok)
4. [Gazebo bővítmények és szenzorbeállítások](#gazebo-bővítmények-és-szenzorbeállítások)
   - [MulticopterMotorModel plugin](#multicoptermotormodel-plugin)
   - [Vezérlés – VelocityControl plugin](#vezérlés--velocitycontrol-plugin)
   - [Szenzorok](#szenzorok-1)
     - [Kamera](#kamera-1)
     - [IMU](#imu-1)
     - [GPS (NavSat)](#gps-navsat)
   - [Pozíció és világbeállítások](#pozíció-és-világbeállítások)
   - [Kiegészítő plugin-ek](#kiegészítő-plugin-ek)
5. [Lokalizációs rendszer](#lokalizációs-rendszer)
   - [Kalman-szűrő (EKF)](#kalman-szűrő-ekf)
   - [GPS – Navsat átalakítás](#gps--navsat-átalakítás)
   - [Vizualizáció és hangolás – Trajectory Server](#vizualizáció-és-hangolás--trajectory-server)
6. [Szimuláció](#szimuláció)
   - [Távirányítás](#távirányítás)
     - [teleop_drone node](#teleop_drone-node)
     - [drone_way_home node](#drone_way_home-node)
       - [Finding home / waypoint](#finding-home--waypoint)
7. [Licence](#licence)

## Install and setup

### Prerequisites

A használathoz ROS2 Jazzy desktop valamint Gazebo Harmonic telepítése szükséges.

Továbbá a következő packagek:
```bash
git clone https://github.com/MOGI-ROS/mogi_trajectory_server
```

```bash
pip install tf-transformations
```


### Git Repo klónozása

A git repo a következő paranccsal klónozható a ROS2 workspace-be:

```bash
git clone https://github.com/nadasmartin/drone_basic_py.git
```

Ezután a futtatáshoz szükséges:

```bash
colcon build
```

valamint:

```bash
source install/setup.bash
```


## Usage
Szimuláció indítása:

```bash
ros2 launch drone_basic_py spawn_robot.launch.py
```
Általunk kibővített Teleop node indítása, a drón irányításához:
```bash
ros2 run drone_basic_py teleop_drone
```


# Drone modell

## Drone Mesh
A projekt során egy egyszerű, négymotoros kvadkopter modellt készítettünk, melynek testének modelljét Inventorban hoztuk létre, majd ezt Blenderben textúráztuk fel és innen exportáltuk ki a végső `.dae` kiterjesztésű mesh fájlt, amelyet a ROS2 környezetben egészítettünk ki további alkatrészekkel.

  ![Drone body](docs/drone_body.png)

## Drone URDF

 Az URDF modell tartalmazza a drón törzsét, rotorjait, valamint az autonóm működéshez szükséges szenzorokat (kamera, IMU, GPS).

   ![Drone model](docs/drone_urdf.png)

### Szerkezeti felépítés

- **Törzs (`base_link`)**
  - Egy könnyű doboz testet definiáltunk, amelyhez a rotorok és szenzorok kapcsolódnak.
  - A törzshöz valósághű tömeg- és tehetetlenségi paramétereket rendeltünk.
  - Megjelenéséhez a `.dae` formátumú 3D mesh modellt használtunk.

- **Rotorok**
  - Négy rotor található a modellen: `rotor_0`, `rotor_1`, `rotor_2`, `rotor_3`.
  - Mindegyik rotor külön linkként szerepel, és egy-egy folyamatos (`continuous`) típusú csuklóval (`joint`) csatlakozik a törzshöz.
  - A rotorokat eltérő színekkel jelöltük (piros és kék), valamint eltérő irányú forgással (CW és CCW) modelleztük.

### Szenzorok

- **Kamera**
  - Előre néző kamerát helyeztünk el a drón elején (`camera_link` és `camera_link_optical`).
  - A kamera fix csuklóval csatlakozik a törzshöz, az optikai tengely megfelelő tájolásával.
  - A Gazebo szimulációban piros színnel jelenik meg.

- **IMU**
  - Az IMU-t a `imu_link` elem képviseli, amelyet fixen rögzítettünk a törzshöz.
  - A későbbi szenzor plugin integrációkhoz megfelelő alapot biztosít.

- **GPS**
  - A `navsat_link` szintén egy fix csuklón keresztül kapcsolódik a törzshöz.
  - A lokalizációhoz szükséges.

### Kiterjesztések és sablonok

- A modell két további `xacro` sablont használ:
  - `materials.xacro` – az egyedi anyagok és színek definiálásához.
  - `drone_custom.gazebo` – Gazebo pluginok és szimulációspecifikus beállítások hozzáadásához.

  ![Drone node communication](docs/image_tftree.png)
  ![Drone model with axes](docs/image_drone_frames.png)
 

# Gazebo bővítmények és szenzorbeállítások

A Gazebo szimulációhoz a drónmodellünkhöz több, funkcionalitást biztosító plugin-t és szenzor konfigurációt integráltunk. Ezek biztosítják a fizikai szimuláció realisztikus viselkedését, valamint lehetővé teszik az autonóm vezérlést és a lokalizációt.

## MulticopterMotorModel plugin

Mind a négy rotorhoz külön `MulticopterMotorModel` plugin-t rendeltünk hozzá, amely a rotorok fizikai viselkedését modellezi:

- Beállítottuk a forgásirányt (CW/CCW), a gyorsulási/lelassulási időállandókat és a maximális fordulatszámot.
- A rotorok erő- és nyomaték-konstansait, valamint a légellenállási és gördülési momentummal kapcsolatos paramétereket is meghatároztuk.
- Minden rotorhoz külön `motorSpeedPubTopic` került hozzárendelésre.

## Vezérlés – VelocityControl plugin

A `MulticopterVelocityControl` plugin biztosítja az autonóm vezérlés lehetőségét:

- `drone/cmd_vel` topikon keresztül lineáris és szögsebesség parancsokat fogad.
- Három szinten történik a vezérlés: sebesség, dőlésszög (attitűd) és szögsebesség.

## Szenzorok

### Kamera

- Előre néző kamera került a drónra, amely 640×480 felbontású RGB képeket szolgáltat.
- A képek enyhe Gauss-zajjal rendelkeznek a realisztikusabb szimuláció érdekében.
- A kamera `camera/image` és `camera/camera_info` topikokra publikál.

### IMU

- 100 Hz frissítési frekvenciájú IMU került elhelyezésre a törzs középpontjában.
- A szimulációban valósághű gyorsulás- és szögsebesség-adatokat szolgáltat.
- Publikálási topik: `imu`.

### GPS (NavSat)

- 1 Hz-es frissítési frekvenciájú GPS szenzort helyeztünk el a modellen.
- A rendszer földrajzi helyzetét biztosítja.
- Publikálási topik: `navsat`.

## Pozíció és világbeállítások

- A szimulációs világ WGS84 koordináta-rendszerben van definiálva, a Ferihegyi repülőtéren:
  - **Szélesség:** `47.438998°`
  - **Hosszúság:** `19.268083°`
  - **Tengerszint feletti magasság:** `0 m`
- Az orientáció ENU (East-North-Up) rendszerű.

## Kiegészítő plugin-ek

- **OdometryPublisher**: Publikálja a drón valós pozícióját a `drone/odom_ground_truth` topikra, lokalizáció fejlesztés során hasznos volt, hogy tudtuk mihez hasonlítani a lokalizáció aktuális képességét.
- **JointStatePublisher**: Leképezi a rotorcsuklók állapotát a `joint_states` topikra.

## Lokalizációs rendszer

A drón pozíciójának és orientációjának megbízható meghatározása érdekében a ROS 2-höz készült **robot_localization** csomagot használtuk. A rendszer az IMU és GPS adatok egyesítésével becsült pontos és szűrt állapotot biztosít a drón számára.

### Kalman-szűrő (EKF)

Az állapotszűrés alapját az `ekf_node` adja, amely egy **Extended Kalman Filter** (EKF) algoritmust futtat:

- A szűrő a `ekf.yaml` konfigurációs fájl alapján dolgozik.
- A bemeneti szenzoradatok közé tartozik az IMU (`imu`) és a GPS alapú odometria (`odometry/gps`).
- A szűrt állapot a `odometry/filtered` topikon jelenik meg.

### GPS – Navsat átalakítás

A GPS adatokat a `navsat_transform_node` dolgozza fel, amely a földrajzi koordinátákat a lokális térbe transzformálja:

- A transzformáció beállításait a `navsat_transformation.yaml` fájl tartalmazza.
- Az IMU adatokat a `imu` topikról, a nyers GPS adatokat a `navsat` topikról olvassa be.
- Kimenetként a `odometry/gps` és `gps/filtered` topikokra publikál.

### Vizualizáció és hangolás – Trajectory Server

A lokalizáció finomhangolását és vizsgálatát **RViz-ben** végeztük, a **mogi_trajectory_server** csomag segítségével. Ez lehetővé tette a becsült és a ground truth pozíciók összehasonlítását:

- A `mogi_trajectory_server` folyamatosan naplózza a szűrt pozíciót az `odom_estimate` frame-ben.
- A `mogi_trajectory_server_topic_based` komponens a Gazebo-ból érkező `drone/odom_ground_truth` topikot jeleníti meg a `trajectory_ground_truth` néven.

Ennek köszönhetően vizuálisan is nyomon tudtuk követni a lokalizáció teljesítményét, és pontosan tudtuk hangolni a szűrő paramétereit.

Az alábbi videóban a piros trajektória a ground truth és a zöld pedig a becsült.

<video controls src="docs/20250517-1517-47.6764496.mp4" title="Title"></video>



## Szimuláció

A korábban elmített paranccsal elindíthatjuk a szimulációt:

```bash
ros2 launch drone_basic_py spawn_robot.launch.py
```

Ez a launch file elindítja a Gazebo szimulációt, alapértelmezetten a `home.sdf` worldbe betölti a drone modelljét valamint elindítja az Rviz-t és a helymeghatározáshoz valamint az automatikus haza repüléshez szükséges node-okat.

A `home.sdf` modell a https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics könyvtárból származik.

### Távirányítás

#### teleop_drone node
```bash
ros2 run drone_basic_py teleop_drone
```

A fenti paranccsal elindítható távirányító node-ot a [teleop_twist_keyboard] (https://index.ros.org/p/teleop_twist_keyboard/) módosításával hoztuk létre. Ezáltal billentyűzet segítségével küldhetünk a `drone/cmd_vel` topikra parancsokat:

```
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)
h : go home
[1-5] : save / go to waypoints

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

```

Az egyszerű vezérlés mellett kibővítettük a lehetséges parancsokat `go home` illetve `save waypoint`, valamint `go to waypoint` opciókkal, melyek a `h` illetve az `1` - `5` billentyűk segítségével aktiválhatóak és a 'drone/waypoint' topikra küldenek üzeneteket.

Ezeknek a parancsoknak a végrehajtásáért a `drone_way_home.py` node felel, amelyet a `spawn_robot.launch.py` szintén automatikusan indít.

#### drone_way_home node

Ez a node a `odometry/filtered` topikot figyeli, és automatikusan elmenti az első pozíciót mint home pozíció, illetve ha a 'drone/waypoint' topikon `1` - `5` parancsot kap, annak megfelelően 5 db waypoint koordinátát tud menteni. Ammenyiben már létezik mentett pozíció az adott billentyűparancs alatt, a drone elkezdi a manőverezést az adott helyre. 

##### Finding home / waypoint

A manőverezéshez a drone először `takeoff_height` paraméteren meghatározott magasságba emelkedik (default: 3 m), ha túl alacsonyan van, majd vízszintesen megközelíti a mentett pozíció X-Y koordinátáit PD szabályzás segítségével és ezután landol `home` parancs esetén, vagy ereszkedik a waypoint Z koordinátájának megfelelő magasságba.

## Licence

Ez a projekt az Apache License 2.0 alatt áll. Részletek a `LICENSE` fájlban.
