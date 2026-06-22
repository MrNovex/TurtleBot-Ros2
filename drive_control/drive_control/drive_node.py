#!/usr/bin/env python3
"""TurtleBot3 Burger - Fahrsteuerung (drive_control).

Dieses Modul ist die Python-Portierung des urspruenglichen C++ Nodes
``drive_node.cpp``. Der Roboter faehrt geradeaus, erkennt mit dem
Laserscanner (LDS-01/LDS-02) Hindernisse direkt vor sich und dreht sich
bei zu geringem Abstand um ca. 180 Grad, um anschliessend weiterzufahren.

Behobener Fehler - "Roboter faehrt leicht schraeg"
--------------------------------------------------
Im C++ Original wurde beim Geradeausfahren dauerhaft eine kleine
Drehgeschwindigkeit gesendet::

    msg.linear.x  = 0.1;
    msg.angular.z = -0.02;   // <-- verursacht eine staendige Kurve

Bei 0.1 m/s und -0.02 rad/s ergibt sich ein Kurvenradius von etwa
``v / w = 0.1 / 0.02 = 5 m``. Der Roboter faehrt dadurch eine permanente
(Rechts-)Kurve - also "leicht schraeg".

Loesung in dieser Version:

1. Beim Geradeausfahren ist die Soll-Drehrate 0.0 rad/s (kein
   kuenstliches Verziehen mehr).
2. Optional (Standard: an) wird ueber die Odometrie (``/odom``) der Kurs
   geregelt: Beim Start bzw. nach jeder Drehung wird die aktuelle
   Gier-Ausrichtung (Yaw) als Sollkurs gespeichert und mit einem
   P-Regler gehalten. Damit wird zusaetzlich ein mechanischer Drift
   (z. B. durch minimal unterschiedliche Raeder oder Reibung) aktiv
   ausgeglichen - der Roboter faehrt eine gerade Linie.

Alle wichtigen Werte sind ROS-2-Parameter und koennen zur Laufzeit bzw.
ueber die Launch-Datei angepasst werden.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


# Zustaende der einfachen Zustandsmaschine
STATE_DRIVE = 'DRIVE'   # Geradeausfahren (mit Kurshaltung)
STATE_TURN = 'TURN'     # Drehen um ca. 180 Grad


def normalize_angle(angle):
    """Normiert einen Winkel (rad) auf das Intervall [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x, y, z, w):
    """Berechnet aus einer Quaternion den Gierwinkel (Yaw, Drehung um z)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, low, high):
    """Begrenzt ``value`` auf das Intervall [low, high]."""
    return max(low, min(high, value))


class DriveNode(Node):
    """ROS-2-Node, der den TurtleBot3 Burger steuert.

    Verhalten:
      * Geradeausfahren mit konstanter Geschwindigkeit und optionaler
        Odometrie-basierter Kurshaltung (gegen schraeges Fahren).
      * Hinderniserkennung im vorderen Sektor (+/- ``front_angle_deg``).
      * Ausweichen durch eine ~180-Grad-Drehung.
    """

    def __init__(self):
        super().__init__('drive_node')

        # --- Parameter deklarieren (mit Standardwerten) ---
        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('turn_speed', 1.1)
        self.declare_parameter('safe_distance', 0.3)
        self.declare_parameter('front_angle_deg', 10.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('use_heading_control', True)
        self.declare_parameter('heading_kp', 1.5)
        self.declare_parameter('max_correction', 0.5)
        self.declare_parameter('turn_tolerance_deg', 5.0)
        self.declare_parameter('turn_duration', 3.14)

        # --- Parameterwerte einlesen ---
        self.linear_speed = self.get_parameter('linear_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.front_angle = math.radians(
            self.get_parameter('front_angle_deg').value)
        self.control_frequency = self.get_parameter('control_frequency').value
        self.use_heading_control = \
            self.get_parameter('use_heading_control').value
        self.heading_kp = self.get_parameter('heading_kp').value
        self.max_correction = self.get_parameter('max_correction').value
        self.turn_tolerance = math.radians(
            self.get_parameter('turn_tolerance_deg').value)
        self.turn_duration = self.get_parameter('turn_duration').value

        # --- Interner Zustand ---
        self.state = STATE_DRIVE
        self.too_close = False              # Hindernis im vorderen Sektor?
        self.front_distance = float('inf')  # kleinster Abstand vorne (nur Info)
        self.current_yaw = None             # aktuelle Ausrichtung aus /odom
        self.target_yaw = None              # Sollkurs (Geradeaus / Drehziel)
        self.have_odom = False              # wurde schon Odometrie empfangen?
        self.turn_start_time = None         # fuer den zeitbasierten Fallback

        # --- ROS-Schnittstellen ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Regelschleife mit fester Frequenz
        self.timer = self.create_timer(
            1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info(
            'drive_node gestartet | Kursregelung (Odometrie): %s'
            % ('AN' if self.use_heading_control else 'AUS'))

    # ------------------------------------------------------------------
    # Callbacks (Sensordaten)
    # ------------------------------------------------------------------
    def scan_callback(self, msg):
        """Wertet den Laserscan aus und prueft den vorderen Sektor.

        Die relevanten Strahlen werden ueber ``angle_min`` und
        ``angle_increment`` bestimmt (robust gegenueber verschiedenen
        Scannern), statt - wie im Original - feste Indizes (0..10 und
        350..360) zu verwenden.
        """
        too_close = False
        min_dist = float('inf')

        for i, r in enumerate(msg.ranges):
            angle = normalize_angle(msg.angle_min + i * msg.angle_increment)
            if abs(angle) > self.front_angle:
                continue  # Strahl liegt ausserhalb des vorderen Sektors
            if not math.isfinite(r) or r <= 0.0:
                continue  # ungueltige Messung (inf/NaN/0) ueberspringen
            min_dist = min(min_dist, r)
            if r <= self.safe_distance:
                too_close = True

        self.too_close = too_close
        self.front_distance = min_dist

    def odom_callback(self, msg):
        """Speichert die aktuelle Ausrichtung (Yaw) aus der Odometrie."""
        q = msg.pose.pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.have_odom = True

    # ------------------------------------------------------------------
    # Hauptregelschleife
    # ------------------------------------------------------------------
    def control_loop(self):
        """Wird periodisch aufgerufen und sendet einen Twist-Befehl."""
        msg = Twist()
        if self.state == STATE_DRIVE:
            self._drive(msg)
        else:
            self._turn(msg)
        self.cmd_pub.publish(msg)

    def _drive(self, msg):
        """Geradeausfahren mit optionaler Kurshaltung."""
        # Hindernis erkannt -> in den Drehzustand wechseln.
        if self.too_close:
            self._start_turn()
            return  # msg bleibt 0 -> kurzer Stopp in diesem Takt

        msg.linear.x = self.linear_speed

        if self.use_heading_control and self.have_odom:
            # Beim ersten Mal die aktuelle Ausrichtung als Sollkurs merken.
            if self.target_yaw is None:
                self.target_yaw = self.current_yaw
            error = normalize_angle(self.target_yaw - self.current_yaw)
            # P-Regler haelt den Kurs (begrenzte Korrektur).
            msg.angular.z = clamp(
                self.heading_kp * error,
                -self.max_correction, self.max_correction)
        else:
            # Minimal-Fix: keine kuenstliche Drehrate mehr (frueher -0.02).
            msg.angular.z = 0.0

        self.get_logger().info(
            'FAHREN | Abstand vorne: %.2f m' % self.front_distance,
            throttle_duration_sec=1.0)

    def _start_turn(self):
        """Wechselt in den Drehzustand und legt das Drehziel fest."""
        self.state = STATE_TURN
        self.turn_start_time = self.get_clock().now()
        if self.have_odom:
            # Ziel: aktuelle Ausrichtung + 180 Grad.
            self.target_yaw = normalize_angle(self.current_yaw + math.pi)
        else:
            self.target_yaw = None  # -> zeitbasierter Fallback
        self.get_logger().info('Hindernis erkannt -> drehe ~180 Grad')

    def _turn(self, msg):
        """Dreht den Roboter um ca. 180 Grad (odometrie- oder zeitbasiert)."""
        if self.use_heading_control and self.have_odom \
                and self.target_yaw is not None:
            # Praezise Drehung anhand der Odometrie.
            error = normalize_angle(self.target_yaw - self.current_yaw)
            if abs(error) <= self.turn_tolerance:
                self._finish_turn()
                return
            # P-Regler dreht Richtung Ziel und bremst kurz davor ab.
            msg.angular.z = clamp(
                self.heading_kp * error, -self.turn_speed, self.turn_speed)
        else:
            # Fallback wie im Original: feste Drehdauer (keine Odometrie).
            elapsed = (self.get_clock().now()
                       - self.turn_start_time).nanoseconds / 1e9
            if elapsed >= self.turn_duration:
                self._finish_turn()
                return
            msg.angular.z = self.turn_speed

        msg.linear.x = 0.0  # waehrend der Drehung nicht vorwaerts fahren

    def _finish_turn(self):
        """Beendet die Drehung und uebernimmt den neuen Sollkurs."""
        self.state = STATE_DRIVE
        self.too_close = False
        # Neuer Sollkurs = aktuelle Ausrichtung nach der Drehung.
        self.target_yaw = self.current_yaw if self.have_odom else None
        self.get_logger().info('Drehung abgeschlossen -> fahre weiter')


def main(args=None):
    """Einstiegspunkt: initialisiert ROS, startet den Node und raeumt auf."""
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Roboter beim Beenden sicher anhalten.
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
