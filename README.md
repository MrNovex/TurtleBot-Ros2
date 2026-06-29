<p align="center">
  <img src="https://www.ros.org/imgs/logo-white.png" alt="ROS 2 Logo" width="120">
</p>

<h1 align="center">🐢 TurtleBot3 Burger – Drive Control & Wall Follow</h1>

<p align="center">
  <strong>Autonome Fahrsteuerung mit Hindernisvermeidung & Wall-Following für den TurtleBot3 Burger</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Jazzy-blue?style=for-the-badge&logo=ros&logoColor=white" alt="ROS 2 Jazzy">
  <img src="https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white" alt="Python 3.10+">
  <img src="https://img.shields.io/badge/Lizenz-Apache_2.0-green?style=for-the-badge" alt="Lizenz">
  <img src="https://img.shields.io/badge/Plattform-TurtleBot3_Burger-orange?style=for-the-badge" alt="Plattform">
</p>

---

## 📋 Überblick

Dieses Paket enthält **zwei Fahrmodi** für den TurtleBot3 Burger:

| Modus | Node | Beschreibung |
|---|---|---|
| 🔄 **Drive (180°-Drehen)** | `drive_node` | Fährt geradeaus, dreht bei Hindernis um ~180° |
| 🧱 **Wall-Follow** | `wall_follow_node` | Fährt an der rechten Wand entlang mit 30 cm Abstand |

Beide Nodes sind als **reine Python-Portierung** umgesetzt und nutzen den LiDAR-Scanner zur Navigation.

### ✨ Highlights

- 🎯 **Odometriebasierte Kurshaltung** – fährt exakt geradeaus statt in einer leichten Kurve
- 🔄 **Präzise 180°-Drehung** – odometriegesteuert statt zeitbasiert
- 🧱 **Wall-Following** – folgt der rechten Wand mit konstantem Abstand (fährt im Kreis durch den Raum)
- 🛡️ **Robuste Hinderniserkennung** – über Scan-Winkel statt fester Array-Indizes
- 🐍 **Reines Python** – einfacher zu lesen, zu erweitern und zu debuggen

---

## 📑 Inhaltsverzeichnis

- [Überblick](#-überblick)
- [Architektur](#-architektur)
- [Was wurde geändert?](#-was-wurde-geändert)
- [Der Bugfix: Schrägfahren](#-der-bugfix-schrägfahren)
- [Paketstruktur](#-paketstruktur)
- [Voraussetzungen](#-voraussetzungen)
- [Installation & Build](#-installation--build)
- [Starten](#-starten)
- [Konfiguration](#-konfiguration)
- [Funktionsweise – Drive Node](#-funktionsweise--drive-node)
- [Funktionsweise – Wall Follow](#-funktionsweise--wall-follow)
- [Lizenz](#-lizenz)

---

## 🏗️ Architektur

```
                  ┌──────────────┐
                  │   LiDAR      │
                  │  /scan       │
                  └──────┬───────┘
                         │
                         ▼
┌──────────────┐   ┌───────────────────┐   ┌──────────────┐
│  Odometrie   │──▶│   drive_node.py   │──▶│  /cmd_vel    │
│  /odom       │   │                   │   │  (Motoren)   │
└──────────────┘   │  ┌─────────────┐  │   └──────────────┘
                   │  │ Zustand:    │  │
                   │  │ FAHREN oder │  │
                   │  │ DREHEN      │  │
                   │  └─────────────┘  │
                   └───────────────────┘
```

Der Node arbeitet als **Zustandsmaschine** mit zwei Modi:
1. **Fahren** – Geradeausfahrt mit aktiver Kurshaltung (P-Regler)
2. **Drehen** – 180°-Ausweichmanöver bei erkanntem Hindernis

---

## 🔄 Was wurde geändert?

| | Vorher (C++) | Nachher (Python) |
|---|---|---|
| **Build-System** | `ament_cmake` / `rclcpp` | `ament_python` / `rclpy` |
| **Quellcode** | `src/drive_node.cpp` | `drive_control/drive_node.py` |
| **Drehung** | Zeitbasiert (3,14 s) | Odometriebasiert (präzise ~180°) |
| **Hinderniserkennung** | Feste Array-Indizes | Über Scan-Winkel (robuster) |
| **Kurshaltung** | Fester Offset → Drift | P-Regler über Odometrie → gerade Linie |

> [!NOTE]
> Die alten C++-Dateien (`drive_node.cpp`, `CMakeLists.txt`, C++-bezogene `.vscode`-Dateien) werden nicht mehr benötigt und sind durch das Python-Paket ersetzt.

---

## 🐛 Der Bugfix: Schrägfahren

### Problem

Im C++-Original wurde beim Geradeausfahren **dauerhaft eine feste Drehgeschwindigkeit** mitgesendet:

```cpp
msg.linear.x  = 0.1;
msg.angular.z = -0.02;   // ← Ursache für die permanente Kurve
```

Bei `v = 0.1 m/s` und `ω = 0.02 rad/s` ergibt sich ein Kurvenradius von:

$$r = \frac{v}{\omega} = \frac{0.1}{0.02} = 5\,\text{m}$$

Der Roboter beschreibt also eine **permanente Kurve mit 5 m Radius** – genau das wahrgenommene „leichte Schrägfahren".

### Lösung

Statt eines festen Korrekturwerts hält der Node den Kurs über die **Odometrie**:

1. Beim Start und nach jeder Drehung wird die aktuelle Ausrichtung (Yaw) als **Sollkurs** gespeichert
2. Ein **P-Regler** regelt die Abweichung zum Sollkurs aktiv aus
3. Auch **mechanischer Drift** (z. B. unterschiedliche Räder, Reibung) wird automatisch kompensiert

```python
# Kurshaltung: Abweichung vom Sollkurs ausregeln
msg.angular.z = self.kp * self.angle_diff(self.target_yaw, self.yaw)
```

> [!TIP]
> Falls der Roboter immer noch leicht driftet, kann der Wert `kp` erhöht werden (Standard: `1.5`). Ein zu hoher Wert führt allerdings zu Oszillation.

---

## 📁 Paketstruktur

```
TurtleBot-Ros2/
├── 📄 README.md
└── 📦 drive_control/                      # ROS 2 Paket (ament_python)
    ├── package.xml                         # Paket-Manifest & Abhängigkeiten
    ├── setup.py                            # Python-Build-Konfiguration
    ├── setup.cfg                           # Colcon Entry-Point-Konfiguration
    ├── resource/
    │   └── drive_control                   # ament-Index-Marker (leer)
    ├── launch/
    │   ├── drive_control.launch.py         # Launch: Drive-Node (180°-Drehen)
    │   └── wall_follow.launch.py           # Launch: Wall-Follow-Node
    └── drive_control/
        ├── __init__.py
        ├── drive_node.py                   # Node: Geradeaus + 180°-Drehung
        └── wall_follow_node.py             # Node: Wandverfolgung rechts
```

---

## ✅ Voraussetzungen

| Anforderung | Details |
|---|---|
| **Betriebssystem** | Ubuntu 24.04 (empfohlen) |
| **ROS 2** | Jazzy Jalisco |
| **Python** | ≥ 3.10 |
| **Roboter / Simulation** | TurtleBot3 Burger oder Gazebo |

Benötigte ROS 2 Pakete:

```
rclpy  geometry_msgs  sensor_msgs  nav_msgs
```

---

## 🔧 Installation & Build

### 1. Workspace anlegen & Repository klonen

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/MrNovex/TurtleBot-Ros2.git
```

### 2. Abhängigkeiten installieren

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Bauen

```bash
colcon build --packages-select drive_control
```

### 4. Workspace sourcen

```bash
source install/setup.bash
```

> [!IMPORTANT]
> Schritt 4 muss in **jedem neuen Terminal** wiederholt werden, oder in die `~/.bashrc` eingetragen werden:
> ```bash
> echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## 🚀 Starten

### Schritt 1: Roboter oder Simulation starten

Die Topics `/scan`, `/odom` und `/cmd_vel` müssen verfügbar sein.

<details>
<summary><strong>🤖 Echter Roboter</strong> (auf dem TurtleBot)</summary>

```bash
ros2 launch turtlebot3_bringup robot.launch.py
```

</details>

<details>
<summary><strong>💻 Gazebo-Simulation</strong> (auf dem PC)</summary>

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

</details>

### Schritt 2: Einen der beiden Modi starten

**Modus A – Drive (180°-Drehen bei Hindernis):**
```bash
ros2 run drive_control drive_node
# oder per Launch-Datei:
ros2 launch drive_control drive_control.launch.py
```

**Modus B – Wall-Follow (an der rechten Wand entlang):**
```bash
ros2 run drive_control wall_follow_node
# oder per Launch-Datei:
ros2 launch drive_control wall_follow.launch.py
```

> [!WARNING]
> Immer nur **einen** der beiden Nodes gleichzeitig starten – beide publishen auf `/cmd_vel`.

---

## ⚙️ Konfiguration

### Drive Node (`drive_node.py`)

| Parameter | Standardwert | Beschreibung |
|---|:---:|---|
| `speed` | `0.1` m/s | Vorwärtsgeschwindigkeit |
| `turn_speed` | `1.1` rad/s | Drehgeschwindigkeit beim Ausweichmanöver |
| `safe_distance` | `0.3` m | Mindestabstand zum Hindernis (vorne) |
| `kp` | `1.5` | Verstärkungsfaktor des P-Reglers für die Kurshaltung |

### Wall-Follow Node (`wall_follow_node.py`)

| Parameter | Standardwert | Beschreibung |
|---|:---:|---|
| `wanted_distance` | `0.30` m | Gewünschter Abstand zur rechten Wand |
| `speed` | `0.08` m/s | Vorwärtsgeschwindigkeit |
| `max_turn` | `1.0` rad/s | Maximale Drehgeschwindigkeit |

> [!WARNING]
> Ein `safe_distance` / `wanted_distance`-Wert unter `0.2 m` kann bei höheren Geschwindigkeiten dazu führen, dass der Roboter nicht rechtzeitig reagiert.

---

## 🧠 Funktionsweise – Drive Node

### Kontrollschleife (20 Hz)

```
┌─────────┐     Hindernis?     ┌─────────┐
│ FAHREN  │ ──── ja ──────────▶│ DREHEN  │
│         │                    │  ~180°  │
│ v = 0.1 │◀── fertig ────────│ ω = 1.1 │
│ P-Regler│    (< 5° Fehler)  │         │
└─────────┘                    └─────────┘
```

- **Fahren:** `linear.x = speed`. Die Abweichung vom Sollkurs wird über den P-Regler ausgeglichen: `angular.z = kp · Δyaw`
- **Hindernis erkannt** (Abstand ≤ `safe_distance` im Bereich **±10°** vor dem Roboter): Der Roboter dreht sich, bis er ~180° weiter ausgerichtet ist, und speichert die neue Richtung als Sollkurs
- **Fallback:** Solange keine Odometrie empfangen wurde, fährt der Roboter einfach geradeaus

### ROS 2 Topics (Drive Node)

| Richtung | Topic | Nachrichtentyp | Beschreibung |
|:---:|---|---|---|
| ⬅️ Sub | `/scan` | `sensor_msgs/LaserScan` | LiDAR-Daten für Hinderniserkennung |
| ⬅️ Sub | `/odom` | `nav_msgs/Odometry` | Odometrie für Yaw-Berechnung |
| ➡️ Pub | `/cmd_vel` | `geometry_msgs/Twist` | Geschwindigkeitsbefehle an den Roboter |

---

## 🧱 Funktionsweise – Wall Follow

### Kontrollschleife (10 Hz)

Der Node misst drei Richtungen per LiDAR (vorne, rechts, vorne-rechts) und entscheidet:

```
┌────────────────┐   Wand vorne?    ┌──────────────┐
│ AN WAND FAHREN │ ──── ja ────────▶│ LINKS DREHEN │
│   v = 0.08     │                  │  (auf Stelle) │
│   Abstand 30cm │◀── frei ────────│              │
└───────┬────────┘                  └──────────────┘
        │
        │ keine Wand rechts?
        ▼
┌────────────────┐
│  RECHTS DREHEN │
│  (Wand suchen) │
└────────────────┘
```

| Situation | Aktion |
|---|---|
| 🧱 Wand **vorne** (< 35 cm) | Stoppen, nach links drehen |
| 🔍 **Keine Wand** rechts (> 45 cm) | Vorwärts + rechts drehen |
| ⚠️ **Zu nah** an Wand rechts (< 25 cm) | Vorwärts + leicht links lenken |
| ✅ Abstand **passt** (~30 cm) | Geradeaus fahren |

### ROS 2 Topics (Wall Follow)

| Richtung | Topic | Nachrichtentyp | Beschreibung |
|:---:|---|---|---|
| ⬅️ Sub | `/scan` | `sensor_msgs/LaserScan` | LiDAR-Daten (vorne, rechts, vorne-rechts) |
| ➡️ Pub | `/cmd_vel` | `geometry_msgs/Twist` | Geschwindigkeitsbefehle an den Roboter |

> [!NOTE]
> Beim Beenden (`Ctrl+C`) senden beide Nodes einen **Null-Twist**, damit der Roboter stehen bleibt.

---

## 📄 Lizenz

Dieses Projekt ist unter der **Apache License 2.0** lizenziert – siehe [`package.xml`](drive_control/package.xml) für Details.

---

<p align="center">
  <sub>Erstellt von <a href="https://github.com/MrNovex">Timo Fahrmer</a> · TurtleBot3 Burger · ROS 2 Jazzy</sub>
</p>
