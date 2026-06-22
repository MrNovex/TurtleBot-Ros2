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
- [Parameter](#parameter)
- [Funktionsweise im Detail](#funktionsweise-im-detail)

---

## Was wurde geändert?

| Vorher (C++)                         | Nachher (Python)                                   |
|--------------------------------------|----------------------------------------------------|
| `ament_cmake`, `rclcpp`              | `ament_python`, `rclpy`                            |
| `src/drive_node.cpp`                 | `drive_control/drive_node.py`                      |
| Feste Werte im Code                  | Konfigurierbare **ROS-2-Parameter**                |
| Drehung nur **zeitbasiert** (3,14 s) | Drehung **odometriebasiert** (präzise ~180°)       |
| Hinderniserkennung über feste Indizes| Erkennung über **Scan-Winkel** (robuster)          |
| Dauerhaftes leichtes Verziehen       | **Kurshaltung** über Odometrie → fährt gerade      |

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

**Lösung (zweistufig):**

1. **Keine künstliche Drehrate mehr.** Beim Geradeausfahren ist die
   Soll-Drehrate `0.0 rad/s`.
2. **Aktive Kurshaltung über Odometrie** (Standard: an). Beim Start und
   nach jeder Drehung wird die aktuelle Ausrichtung (Yaw) als *Sollkurs*
   gespeichert und mit einem **P-Regler** gehalten. Damit wird auch ein
   *mechanischer* Drift (z. B. minimal unterschiedliche Räder, Reibung,
   Bodenunebenheiten) automatisch ausgeglichen – der Roboter fährt eine
   gerade Linie statt einer Kurve.

> Hinweis: Die Kurshaltung lässt sich über den Parameter
> `use_heading_control:=false` abschalten. Dann gilt nur Stufe 1
> (`angular.z = 0.0`), was das Verziehen aus der Software entfernt, einen
> reinen Hardware-Drift aber nicht aktiv korrigiert.

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

# Variante C – mit überschriebenen Parametern
ros2 run drive_control drive_node --ros-args \
  -p linear_speed:=0.15 -p use_heading_control:=true
```

---

## Parameter

Alle Parameter haben sinnvolle Standardwerte und können beim Start
überschrieben werden.

| Parameter              | Typ   | Standard | Bedeutung                                                        |
|------------------------|-------|----------|------------------------------------------------------------------|
| `linear_speed`         | float | `0.1`    | Vorwärtsgeschwindigkeit in m/s                                   |
| `turn_speed`           | float | `1.1`    | Max. Drehgeschwindigkeit beim Ausweichen in rad/s               |
| `safe_distance`        | float | `0.3`    | Sicherheitsabstand vorne in m (darunter wird gedreht)           |
| `front_angle_deg`      | float | `10.0`   | Halber Öffnungswinkel des vorderen Prüfsektors in Grad (±)      |
| `control_frequency`    | float | `20.0`   | Frequenz der Regelschleife in Hz                                |
| `use_heading_control`  | bool  | `true`   | Odometrie-Kurshaltung an/aus (gegen schräges Fahren)            |
| `heading_kp`           | float | `1.5`    | P-Verstärkung der Kursregelung                                  |
| `max_correction`       | float | `0.5`    | Max. Korrektur-Drehrate beim Geradeausfahren in rad/s           |
| `turn_tolerance_deg`   | float | `5.0`    | Restwinkel-Toleranz, ab der eine Drehung als fertig gilt        |
| `turn_duration`        | float | `3.14`   | Drehdauer in s – **nur** im Fallback ohne Odometrie             |

---

## Funktionsweise im Detail

Der Node arbeitet als einfache **Zustandsmaschine** mit zwei Zuständen:

**`DRIVE` (Geradeausfahren)**
- Sendet `linear.x = linear_speed`.
- Ist die Kurshaltung aktiv und Odometrie vorhanden, wird die Abweichung
  vom Sollkurs per P-Regler ausgeglichen
  (`angular.z = clamp(heading_kp · Fehler, ±max_correction)`).
- Sonst: `angular.z = 0.0`.
- Wird im vorderen Sektor (±`front_angle_deg`) ein Abstand ≤
  `safe_distance` gemessen, wird nach `TURN` gewechselt.

**`TURN` (Ausweichen, ~180°)**
- *Mit Odometrie:* dreht, bis das Ziel (aktueller Yaw + 180°) bis auf
  `turn_tolerance_deg` erreicht ist – präzise und unabhängig von der
  Akkuspannung.
- *Ohne Odometrie (Fallback):* dreht für `turn_duration` Sekunden mit
  `turn_speed` (Verhalten wie im Original).
- Danach zurück zu `DRIVE`; die neue Ausrichtung wird als Sollkurs
  übernommen.

**Topics**

| Richtung   | Topic      | Typ                          |
|------------|------------|------------------------------|
| Subscribe  | `/scan`    | `sensor_msgs/msg/LaserScan`  |
| Subscribe  | `/odom`    | `nav_msgs/msg/Odometry`      |
| Publish    | `/cmd_vel` | `geometry_msgs/msg/Twist`    |

Beim Beenden (z. B. `Ctrl+C`) sendet der Node noch einen Null-`Twist`,
damit der Roboter sicher stehen bleibt.
