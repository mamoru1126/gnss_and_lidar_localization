#!/usr/bin/env python3
"""AWSIM の車の代わり（Humble のコンテナでの確認用）。awsim_drive.py の指令で自転車モデルを動かし、真値の姿勢を出す。

  python3 fake_awsim_vehicle.py --x 0 --y 0 --yaw 0 --timeout 120 --goal X,Y

- 受け取る: /control/command/control_cmd（autoware_control_msgs/Control）、/control/command/gear_cmd（DRIVE でないと動かない）。
  QoS は AWSIM v1.3.1 と同じ RELIABLE・TRANSIENT_LOCAL・深さ 1
- 出す: /awsim/ground_truth/vehicle/pose（PoseStamped、100 Hz、BEST_EFFORT。AWSIM v1.3.1 と同じ）、
  /vehicle/status/velocity_status（VelocityReport、30 Hz）
- 速度に比例する抵抗（--drag）を入れる。比例だけの速度制御では目標に届かない車の模擬
- 車が --goal から 1 m 以内で止まったら終了コード 0、--timeout 秒たっても着かなければ 1。
"""
import argparse
import math
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0, help="[deg]")
    ap.add_argument("--wheelbase", type=float, default=2.79)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--goal", required=True)
    ap.add_argument("--drag", type=float, default=0.2, help="速度に比例する減速 [1/s]")
    args = ap.parse_args()
    gx, gy = (float(v) for v in args.goal.split(","))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from autoware_control_msgs.msg import Control
    from autoware_vehicle_msgs.msg import GearCommand, VelocityReport

    class Vehicle(Node):
        def __init__(self):
            super().__init__("fake_awsim_vehicle")
            self.x, self.y, self.yaw, self.v = args.x, args.y, math.radians(args.yaw), 0.0
            self.steer = self.acc = 0.0
            self.drive = False
            self.moved = 0.0
            self.result = None
            self.t0 = self.get_clock().now().nanoseconds * 1e-9
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(Control, "/control/command/control_cmd", self.on_cmd, qos)
            self.create_subscription(GearCommand, "/control/command/gear_cmd", self.on_gear, qos)
            self.pub = self.create_publisher(PoseStamped, "/awsim/ground_truth/vehicle/pose",
                                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.pub_vel = self.create_publisher(VelocityReport, "/vehicle/status/velocity_status", 10)
            self.vmax = 0.0
            self.n = 0
            self.create_timer(0.01, self.tick)

        def on_cmd(self, m):
            self.steer = m.lateral.steering_tire_angle
            self.acc = m.longitudinal.acceleration

        def on_gear(self, m):
            self.drive = m.command == GearCommand.DRIVE

        def tick(self):
            dt = 0.01
            if self.drive:
                self.v = max(0.0, self.v + (self.acc - args.drag * self.v) * dt)
            else:
                self.v = 0.0
            self.x += self.v * math.cos(self.yaw) * dt
            self.y += self.v * math.sin(self.yaw) * dt
            self.yaw += self.v / args.wheelbase * math.tan(self.steer) * dt
            self.moved += self.v * dt
            m = PoseStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.pose.position.x, m.pose.position.y = self.x, self.y
            m.pose.orientation.z, m.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            self.pub.publish(m)
            self.vmax = max(self.vmax, self.v)
            self.n += 1
            if self.n % 3 == 0:
                r = VelocityReport()
                r.header.stamp = m.header.stamp
                r.header.frame_id = "base_link"
                r.longitudinal_velocity = float(self.v)
                self.pub_vel.publish(r)
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.moved > 1.0 and self.v < 0.01 and math.hypot(self.x - gx, self.y - gy) < 1.0:
                # 止まった後も 3 s は姿勢を出し続ける（awsim_drive.py が止まったことを見て終われるように）
                self.stopped_at = getattr(self, "stopped_at", None) or now
                if now - self.stopped_at > 3.0:
                    self.result = 0
            else:
                self.stopped_at = None
            if self.result is None and now - self.t0 > args.timeout:
                self.result = 1
            if self.result is not None:
                print(f"vehicle: ({self.x:.2f}, {self.y:.2f}), moved {self.moved:.1f} m, max {self.vmax * 3.6:.1f} km/h, "
                      f"{'reached' if self.result == 0 else 'did NOT reach'} the goal ({gx}, {gy})", flush=True)
                raise SystemExit(self.result)

    rclpy.init()
    node = Vehicle()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        sys.exit(e.code)


if __name__ == "__main__":
    main()
