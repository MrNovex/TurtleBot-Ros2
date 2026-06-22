#!/usr/bin/env python3
# Fahrsteuerung fuer den TurtleBot3 Burger.
# Faehrt geradeaus und dreht bei einem Hindernis vorne um ca. 180 Grad.

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    # Winkel auf [-pi, pi] begrenzen
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x, y, z, w):
    # Yaw (Drehung um z) aus der Quaternion
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def clamp(value, low, high):
    return max(low, min(high, value))


class DriveNode(Node):

    def __init__(self):
        super().__init__('drive_node')

        # Parameter
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

        self.linear_speed = self.get_parameter('linear_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.safe_distance = self.get_parameter('safe_distance').value
        self.front_angle = math.radians(
            self.get_parameter('front_angle_deg').value)
        freq = self.get_parameter('control_frequency').value
        self.use_heading_control = self.get_parameter('use_heading_control').value
        self.heading_kp = self.get_parameter('heading_kp').value
        self.max_correction = self.get_parameter('max_correction').value
        self.turn_tolerance = math.radians(
            self.get_parameter('turn_tolerance_deg').value)
        self.turn_duration = self.get_parameter('turn_duration').value

        # Zustand
        self.turning = False
        self.too_close = False
        self.front_distance = float('inf')
        self.current_yaw = None
        self.target_yaw = None
        self.have_odom = False
        self.turn_start_time = None

        # Publisher / Subscriber
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_timer(1.0 / freq, self.control_loop)

    def scan_callback(self, msg):
        # vorderen Sektor (+/- front_angle) auf Hindernisse pruefen
        too_close = False
        min_dist = float('inf')
        for i, r in enumerate(msg.ranges):
            angle = normalize_angle(msg.angle_min + i * msg.angle_increment)
            if abs(angle) > self.front_angle:
                continue
            if not math.isfinite(r) or r <= 0.0:
                continue
            min_dist = min(min_dist, r)
            if r <= self.safe_distance:
                too_close = True
        self.too_close = too_close
        self.front_distance = min_dist

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        self.current_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.have_odom = True

    def control_loop(self):
        msg = Twist()
        if self.turning:
            self.turn(msg)
        else:
            self.drive(msg)
        self.cmd_pub.publish(msg)

    def drive(self, msg):
        # Hindernis -> Drehung starten
        if self.too_close:
            self.turning = True
            self.turn_start_time = self.get_clock().now()
            if self.have_odom:
                self.target_yaw = normalize_angle(self.current_yaw + math.pi)
            else:
                self.target_yaw = None
            return

        msg.linear.x = self.linear_speed

        if self.use_heading_control and self.have_odom:
            # Kurs halten: Abweichung vom Sollwinkel ausregeln
            if self.target_yaw is None:
                self.target_yaw = self.current_yaw
            error = normalize_angle(self.target_yaw - self.current_yaw)
            msg.angular.z = clamp(
                self.heading_kp * error, -self.max_correction, self.max_correction)
        else:
            msg.angular.z = 0.0  # frueher -0.02, das verursachte die Kurve

    def turn(self, msg):
        if self.use_heading_control and self.have_odom and self.target_yaw is not None:
            # drehen, bis die 180 Grad erreicht sind
            error = normalize_angle(self.target_yaw - self.current_yaw)
            if abs(error) <= self.turn_tolerance:
                self.finish_turn()
                return
            msg.angular.z = clamp(
                self.heading_kp * error, -self.turn_speed, self.turn_speed)
        else:
            # ohne Odometrie: feste Drehzeit (wie im Original)
            elapsed = (self.get_clock().now()
                       - self.turn_start_time).nanoseconds / 1e9
            if elapsed >= self.turn_duration:
                self.finish_turn()
                return
            msg.angular.z = self.turn_speed

    def finish_turn(self):
        self.turning = False
        self.too_close = False
        self.target_yaw = self.current_yaw if self.have_odom else None


def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # Roboter anhalten
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
