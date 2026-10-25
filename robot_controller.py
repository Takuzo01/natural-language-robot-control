
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.publisher = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )

        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.position = None
        self.yaw = None

        self.front_distance = None

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

    def scan_callback(self, msg):
        distances = []

        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # 正面±15度を確認
            angle = math.atan2(math.sin(angle), math.cos(angle))

            if abs(angle) <= math.radians(15):
                if math.isfinite(distance) and distance > 0:
                    distances.append(distance)

        self.front_distance = min(distances) if distances else None

    # def odom_callback(self, msg):
    #     self.position = (
    #         msg.pose.pose.position.x,
    #         msg.pose.pose.position.y
    #     )

    def odom_callback(self, msg):
        self.position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation

        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def send_velocity(self, speed=0.0, angular=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # 前進・後退速度
        msg.twist.linear.x = float(speed)

        # 左右の旋回速度
        msg.twist.angular.z = float(angular)

        self.publisher.publish(msg)


    def move_forward(self, distance):
        if not 0.05 <= distance <= 1.0:
            raise ValueError("移動距離が範囲外です")

        # オドメトリ取得を最大5秒待つ
        wait_start = time.monotonic()
        while rclpy.ok() and self.position is None:
            if time.monotonic() - wait_start > 5:
                raise RuntimeError("位置情報を取得できません")
            rclpy.spin_once(self, timeout_sec=0.1)

        # LiDARの最初のデータを最大5秒待つ
        print("LiDARデータ受信待機中...")

        wait_start = time.monotonic()

        while rclpy.ok() and self.front_distance is None:
            if time.monotonic() - wait_start > 5:
                raise RuntimeError(
                    "LiDARデータを5秒以内に取得できませんでした"
                )

            rclpy.spin_once(self, timeout_sec=0.1)

        print(f"LiDAR受信成功: {self.front_distance:.2f}m")

        start_x, start_y = self.position
        start_time = time.monotonic()

        print(f"{distance}m 前進開始")

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                if self.front_distance is None:
                    raise RuntimeError("LiDARデータが取得できません")

                if self.front_distance < 0.30:
                    raise RuntimeError(
                        f"障害物を検知したため停止: {self.front_distance:.2f}m"
                    )

                x, y = self.position
                traveled = math.hypot(x - start_x, y - start_y)
                remaining = distance - traveled

                if remaining <= 0.01:
                    print(f"前進完了：{traveled:.3f}m")
                    return

                if time.monotonic() - start_time > 20:
                    raise RuntimeError("前進タイムアウト")

                speed = min(0.12, max(0.03, remaining * 0.6))
                self.send_velocity(speed=speed)

            raise RuntimeError("ROS 2が停止しました")

        finally:
            for _ in range(10):
                self.send_velocity()
                time.sleep(0.05)


    def turn(self, angle_deg):
        if not 1 <= abs(angle_deg) <= 180:
            raise ValueError("旋回角度が範囲外です")

        wait_start = time.monotonic()
        while rclpy.ok() and self.yaw is None:
            if time.monotonic() - wait_start > 5:
                raise RuntimeError("角度情報を取得できません")
            rclpy.spin_once(self, timeout_sec=0.1)

        target = math.radians(angle_deg)
        previous_yaw = self.yaw
        rotated = 0.0
        start_time = time.monotonic()

        print(f"{angle_deg}度 旋回開始")

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)

                current_yaw = self.yaw
                delta = math.atan2(
                    math.sin(current_yaw - previous_yaw),
                    math.cos(current_yaw - previous_yaw)
                )

                rotated += delta
                previous_yaw = current_yaw

                remaining = target - rotated

                if abs(remaining) < math.radians(2):
                    print(
                        f"旋回完了：{math.degrees(rotated):.1f}度"
                    )
                    return

                if time.monotonic() - start_time > 20:
                    raise RuntimeError("旋回タイムアウト")

                angular = min(
                    0.8,
                    max(0.15, abs(remaining) * 1.5)
                )

                self.send_velocity(
                    angular=math.copysign(angular, remaining)
                )

            raise RuntimeError("ROS 2が停止しました")

        finally:
            for _ in range(10):
                self.send_velocity()
                time.sleep(0.05)


    def send_turn(self, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.angular.z = angular
        self.publisher.publish(msg)


# def main():
#     rclpy.init()
#     node = RobotController()

#     try:
#         # node.move_forward(0.5)
#         node.turn(-90)
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
def main():
    rclpy.init()
    node = RobotController()

    try:
        # 50cm前進
        node.move_forward(0.5)

        # 右に90度旋回
        node.turn(-90)

        # さらに30cm前進
        node.move_forward(0.3)

    finally:
        node.send_velocity()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()