#!/usr/bin/env python3
# Wall-Follow Node: Der Roboter faehrt an der rechten Wand entlang mit 30cm Abstand.
# So faehrt er im Raum im Kreis.

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollowNode(Node):

    def __init__(self):
        super().__init__('wall_follow_node')

        # Einstellungen
        self.wanted_distance = 0.30   # 30cm Abstand zur Wand
        self.speed = 0.08             # wie schnell vorwaerts (m/s)
        self.max_turn = 1.0           # maximale Drehgeschwindigkeit

        # Laser-Daten (werden im Callback gefuellt)
        self.front_dist = float('inf')
        self.right_dist = float('inf')
        self.front_right_dist = float('inf')

        # Publisher und Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # Timer: 10x pro Sekunde die Steuerung ausfuehren
        self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Wall-Follow Node gestartet!')

    def scan_callback(self, msg):
        """Liest die Laser-Daten und merkt sich die wichtigen Richtungen."""

        # Wir brauchen 3 Richtungen:
        # - vorne (0 Grad)
        # - rechts (270 Grad bzw. -90 Grad)
        # - vorne-rechts (315 Grad bzw. -45 Grad)
        self.front_dist = self.get_range_at_angle(msg, 0)
        self.right_dist = self.get_range_at_angle(msg, -90)
        self.front_right_dist = self.get_range_at_angle(msg, -45)

    def get_range_at_angle(self, msg, angle_deg):
        """Gibt die Entfernung bei einem bestimmten Winkel zurueck."""

        # Winkel in Radiant umrechnen
        angle_rad = math.radians(angle_deg)

        # Den passenden Index im Scan-Array berechnen
        if angle_rad < msg.angle_min:
            angle_rad += 2 * math.pi
        if angle_rad > msg.angle_max:
            angle_rad -= 2 * math.pi

        index = int((angle_rad - msg.angle_min) / msg.angle_increment)
        index = index % len(msg.ranges)

        # Ein paar Werte drumherum nehmen und den kleinsten Abstand nutzen
        # (damit ein einzelner Messfehler uns nicht verwirrt)
        best = float('inf')
        for i in range(index - 5, index + 6):
            idx = i % len(msg.ranges)
            r = msg.ranges[idx]
            if 0.05 < r < best:
                best = r

        return best

    def timer_callback(self):
        """Hauptlogik: Entscheidet was der Roboter machen soll."""

        msg = Twist()

        # Fall 1: Wand vorne zu nah -> auf der Stelle nach links drehen
        if self.front_dist < self.wanted_distance + 0.05:
            msg.linear.x = 0.0
            msg.angular.z = self.max_turn
            self.get_logger().info('Wand vorne! Drehe links...')

        # Fall 2: Wand rechts verloren (Aussenecke) -> rechts um die Kurve
        elif self.right_dist > self.wanted_distance + 0.25:
            msg.linear.x = self.speed / 1.5  # etwas langsamer um die Ecke
            msg.angular.z = -0.8             # stark rechts drehen
            self.get_logger().info('Ecke! Drehe rechts...')

        # Fall 3: Normale Wandverfolgung (Kombinierte Abstands- und Winkelregelung)
        else:
            msg.linear.x = self.speed
            
            # 1. Abstands-Fehler: positiv = zu nah, negativ = zu weit weg
            dist_error = self.wanted_distance - self.right_dist
            
            # 2. Winkel-Fehler: 
            # Bei paralleler Fahrt zur Wand ist der Strahl schraeg rechts (-45 Grad)
            # durch die Dreiecksgeometrie ca. 1.41-mal (Wurzel 2) so lang wie der direkte Strahl (-90 Grad).
            # Abweichungen davon deuten auf eine unerwuenschte Rotation zur Wand oder von ihr weg hin.
            angle_error = (self.right_dist * 1.41) - self.front_right_dist
            
            # Kombinierte Lenkkorrektur aus Distanz- und Winkelfehler (wirkt wie ein PD-Regler)
            msg.angular.z = (dist_error * 0.8) + (angle_error * 1.5)
            
            self.get_logger().info('Folge Wand (Dist: {:.2f}m)'.format(self.right_dist))

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Roboter anhalten wenn man Ctrl+C drueckt
        node.cmd_pub.publish(Twist())
        node.get_logger().info('Angehalten!')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
