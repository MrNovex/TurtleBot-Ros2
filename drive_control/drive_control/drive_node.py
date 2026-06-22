#!/usr/bin/env python3
# Fahrsteuerung fuer den TurtleBot3 Burger.
# Faehrt geradeaus und haelt den Kurs ueber die Odometrie (damit er nicht
# schraeg faehrt). Vor einem Hindernis dreht er um ca. 180 Grad.

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class DriveNode(Node):

    def __init__(self):
        super().__init__('drive_node')

        # Einstellungen
        self.speed = 0.1          # Vorwaertsgeschwindigkeit (m/s)
        self.turn_speed = 1.1     # Drehgeschwindigkeit (rad/s)
        self.safe_distance = 0.3  # Abstand, ab dem gedreht wird (m)
        self.kp = 1.5             # Verstaerkung fuer die Kurshaltung

        self.turning = False
        self.too_close = False
        self.yaw = None           # aktuelle Ausrichtung aus der Odometrie
        self.target_yaw = None    # Sollkurs

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def scan_callback(self, msg):
        # Hindernis im Bereich +/- 10 Grad vor dem Roboter?
        self.too_close = False
        for i, r in enumerate(msg.ranges):
            angle = math.degrees(msg.angle_min + i * msg.angle_increment)
            if angle > 180:
                angle -= 360
            if abs(angle) <= 10 and 0.0 < r <= self.safe_distance:
                self.too_close = True

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def control_loop(self):
        msg = Twist()
        if self.yaw is None:
            msg.linear.x = self.speed  # noch keine Odometrie -> geradeaus
        elif self.too_close or self.turning:
            self.turn(msg)
        else:
            self.drive(msg)
        self.cmd_pub.publish(msg)

    def drive(self, msg):
        if self.target_yaw is None:
            self.target_yaw = self.yaw
        msg.linear.x = self.speed
        # gegen die Abweichung vom Sollkurs lenken
        msg.angular.z = self.kp * self.angle_diff(self.target_yaw, self.yaw)

    def turn(self, msg):
        if not self.turning:
            self.turning = True
            self.target_yaw = self.yaw + math.pi  # Ziel: 180 Grad weiter
        error = self.angle_diff(self.target_yaw, self.yaw)
        if abs(error) < math.radians(5):
            self.turning = False
            self.too_close = False
            self.target_yaw = self.yaw
            return
        msg.angular.z = self.turn_speed if error > 0 else -self.turn_speed

    def angle_diff(self, a, b):
        # Differenz zweier Winkel auf [-pi, pi]
        return math.atan2(math.sin(a - b), math.cos(a - b))


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())  # Roboter anhalten
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
