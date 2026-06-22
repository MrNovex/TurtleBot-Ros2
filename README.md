# TurtleBot-Ros2 – `drive_control`

Fahrsteuerung für den **TurtleBot3 Burger** unter **ROS 2 (Jazzy)**.

Der Roboter fährt geradeaus, erkennt mit dem Laserscanner Hindernisse
direkt vor sich und weicht ihnen durch eine ~180°-Drehung aus. Dieses
Paket ist die **vollständige Python-Portierung** des ursprünglichen
C++-Nodes (`drive_node.cpp`).

---

## Inhaltsverzeichnis
- [Was wurde geändert?](#was-wurde-geändert)
- [Der Bugfix: „Roboter fährt leicht schräg"](#der-bugfix-roboter-fährt-leicht-schräg)
- [Paketstruktur](#paketstruktur)
- [Bauen (Build)](#bauen-build)
- [Starten](#starten)
- [Einstellungen](#einstellungen)
- [Funktionsweise](#funktionsweise)

---

## Was wurde geändert?

| Vorher (C++)                          | Nachher (Python)                              |
|---------------------------------------|-----------------------------------------------|
| `ament_cmake`, `rclcpp`               | `ament_python`, `rclpy`                       |
| `src/drive_node.cpp`                  | `drive_control/drive_node.py`                 |
| Drehung **zeitbasiert** (3,14 s)      | Drehung **odometriebasiert** (präzise ~180°)  |
| Hinderniserkennung über feste Indizes | Erkennung über **Scan-Winkel** (robuster)     |
| Dauerhaftes leichtes Verziehen        | **Kurshaltung** über Odometrie → fährt gerade |

Die alten C++-Dateien (`drive_node.cpp`, `CMakeLists.txt`, die
C++-bezogenen `.vscode`-Dateien) werden nicht mehr benötigt und sind
durch das Python-Paket ersetzt.

---

## Der Bugfix: „Roboter fährt leicht schräg"

Im C++-Original wurde beim Geradeausfahren **dauerhaft eine kleine
Drehgeschwindigkeit** mitgesendet:

```cpp
msg.linear.x  = 0.1;
msg.angular.z = -0.02;   // <-- Ursache für die ständige Kurve
```

Bei `0.1 m/s` und `-0.02 rad/s` ergibt sich ein Kurvenradius von

```
r = v / ω = 0.1 / 0.02 = 5 m
```

Der Roboter beschreibt also eine permanente Kurve mit 5 m Radius – das
ist genau das wahrgenommene „leichte Schrägfahren".

**Lösung:** Statt eines festen Korrekturwerts hält der Node den Kurs
über die Odometrie. Beim Start und nach jeder Drehung wird die aktuelle
Ausrichtung (Yaw) als *Sollkurs* gemerkt und über einen einfachen
P-Regler gehalten. So wird auch ein *mechanischer* Drift (z. B. minimal
unterschiedliche Räder oder Reibung) automatisch ausgeglichen – der
Roboter fährt eine gerade Linie statt einer Kurve.

---

## Paketstruktur

```
TurtleBot-Ros2/
├── README.md
└── drive_control/                     # ROS-2-Paket (ament_python)
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── drive_control              # ament-Marker (leer)
    ├── launch/
    │   └── drive_control.launch.py    # Start per "ros2 launch"
    └── drive_control/
        ├── __init__.py
        └── drive_node.py              # der eigentliche Node
```

---

## Bauen (Build)

Voraussetzung: ROS 2 (Jazzy) ist installiert und das Paket liegt im
`src`-Ordner eines Colcon-Workspace.

```bash
# 1) Workspace anlegen und Paket hineinklonen/-kopieren
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <REPO-URL>           # oder den Ordner drive_control hierher kopieren

# 2) Abhängigkeiten installieren (optional, empfohlen)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3) Bauen
colcon build --packages-select drive_control

# 4) Workspace „sourcen"
source install/setup.bash
```

---

## Starten

Zuerst muss der Roboter (oder die Simulation) laufen, sodass die Topics
`/scan`, `/odom` und `/cmd_vel` verfügbar sind.

**Echter Roboter** (auf dem TurtleBot, vorher `bringup` starten):
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

**Oder Gazebo-Simulation** (auf dem PC):
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Dann den Fahr-Node starten:**

```bash
# Variante A – einfach per ros2 run
ros2 run drive_control drive_node

# Variante B – per Launch-Datei (empfohlen)
ros2 launch drive_control drive_control.launch.py
```

---

## Einstellungen

Die wichtigsten Werte stehen als Konstanten oben in `__init__`
(`drive_control/drive_node.py`) und können dort direkt angepasst werden:

| Wert            | Standard | Bedeutung                               |
|-----------------|----------|-----------------------------------------|
| `speed`         | `0.1`    | Vorwärtsgeschwindigkeit in m/s          |
| `turn_speed`    | `1.1`    | Drehgeschwindigkeit beim Ausweichen     |
| `safe_distance` | `0.3`    | Abstand vorne in m, ab dem gedreht wird |
| `kp`            | `1.5`    | Stärke der Kurshaltung (P-Regler)       |

---

## Funktionsweise

Die Hauptschleife (`control_loop`, 20 Hz) entscheidet bei jedem Takt
zwischen **Fahren** und **Drehen**:

- **Fahren:** `linear.x = speed`. Der Kurs wird gehalten, indem die
  Abweichung zur gemerkten Soll-Ausrichtung mit `kp` ausgeregelt wird
  (`angular.z = kp · Fehler`) – das verhindert das Schrägfahren.
- **Hindernis erkannt** (Abstand ≤ `safe_distance` im Bereich ±10° vorne):
  Der Roboter dreht sich, bis er ~180° weiter ausgerichtet ist, und
  merkt sich danach die neue Richtung als Sollkurs.

Die Ausrichtung (Yaw) kommt aus der Odometrie (`/odom`). Solange noch
keine Odometrie empfangen wurde, fährt der Roboter einfach geradeaus.

**Topics**

| Richtung  | Topic      | Typ                         |
|-----------|------------|-----------------------------|
| Subscribe | `/scan`    | `sensor_msgs/msg/LaserScan` |
| Subscribe | `/odom`    | `nav_msgs/msg/Odometry`     |
| Publish   | `/cmd_vel` | `geometry_msgs/msg/Twist`   |

Beim Beenden (`Ctrl+C`) sendet der Node noch einen Null-`Twist`, damit
der Roboter stehen bleibt.
