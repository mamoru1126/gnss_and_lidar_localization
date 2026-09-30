#!/usr/bin/env python3
"""AWSIM の車を、決めた経路に沿って一定の速さで走らせる ROS 2 ノード（Humble。AWSIM を動かす PC で使う）。

  source /opt/ros/humble/setup.bash
  python3 awsim_drive.py route.txt [--kmh 25 | --speed 1.67] [--wait 5] [--max-distance 0]
  python3 awsim_drive.py route.txt --check      # ROS なしで、経路を走れるかだけ確かめる
  python3 awsim_drive.py --print-pose           # 車の今の姿勢（plan_route.py の --start に渡す）

- 真値（/awsim/ground_truth/vehicle/pose）を見て、pure pursuit で経路をなぞる。障害物・信号・ほかの車は見ない。
- 速度は既定 6 km/h（本システムの想定の最高速度）。--kmh で上げられる。曲がる所の手前で、横加速度が 1.5 m/s² を
  超えないように減速し、経路の終わりでちょうど止まる。
- 最初に --wait 秒止まったまま待つ（推定ノードの静止初期化のため）。経路の終わりで止まって、ノードも終わる。
- 経路は「x y」の行（地図座標）。車の今の位置から 5 m 以内に経路の点が無いと始めない。
  経路の作り方は README.md（tools/tile_demo/lanelet_route.py で、車の位置から始まる経路を作る）。
- 出すもの: /control/command/control_cmd（autoware_control_msgs/Control）、/control/command/gear_cmd（DRIVE）。
  QoS は AWSIM v1.3.1 の購読に合わせて RELIABLE・TRANSIENT_LOCAL・深さ 1。AWSIM は加速度と操舵角を使う。
必要なパッケージ: rclpy、geometry_msgs、autoware_control_msgs、autoware_vehicle_msgs（docker/awsim/Dockerfile のコンテナに入っている）。
"""
import argparse
import collections
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from drive_core import PurePursuit, load_route  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("route", type=Path, nargs="?")
    ap.add_argument("--speed", type=float, default=1.67, help="目標速度 [m/s]（既定 6 km/h）")
    ap.add_argument("--kmh", type=float, default=None,
                    help="目標速度 [km/h]（--speed の代わり。例 25）。曲がる所では横加速度 1.5 m/s² 以下に落とす")
    ap.add_argument("--wait", type=float, default=5.0, help="走り出す前に止まっている時間 [s]")
    ap.add_argument("--wheelbase", type=float, default=2.79, help="ホイールベース [m]（AWSIM の Lexus RX 450h）")
    ap.add_argument("--max-distance", type=float, default=0.0, help="この距離 [m] を走ったら止まる（0 なら経路の終わりまで）")
    ap.add_argument("--gt-topic", default="/awsim/ground_truth/vehicle/pose")
    ap.add_argument("--velocity-topic", default="/vehicle/status/velocity_status",
                    help="車の速さ（VelocityReport）。来なければ真値の位置の差から求める")
    ap.add_argument("--print-pose", action="store_true",
                    help="車の今の姿勢を 'X,Y,YAW_DEG' で 1 行出して終わる（plan_route.py の --start に渡す）")
    ap.add_argument("--check", action="store_true",
                    help="ROS を使わず、経路を自転車モデルで走らせて、長さ・時間・急な曲がり・横ずれを出すだけ")
    args = ap.parse_args()
    if args.kmh is not None:
        args.speed = args.kmh / 3.6

    if not args.print_pose and args.route is None:
        ap.error("経路のファイルを指定する（--print-pose のときだけ要らない）")
    if args.check:
        import numpy as np
        from drive_core import simulate
        route = load_route(args.route)
        d = route[1] - route[0]
        traj, dev, pp = simulate(route, route[0, 0], route[0, 1], math.atan2(d[1], d[0]), t_max=36000,
                                 wheelbase=args.wheelbase, speed=args.speed)
        length = float(np.sum(np.linalg.norm(np.diff(route, axis=0), axis=1)))
        print(f"route: {len(route)} points, {length:.0f} m, about {length / args.speed / 60:.1f} min at "
              f"{args.speed * 3.6:.0f} km/h（曲がる所の減速を除く）")
        print(f"simulated: max deviation from the route {dev:.2f} m, stopped {pp.remaining():.1f} m before the end")
        tight = pp.tight_turns()
        if tight:
            print(f"tight turns (the car cannot follow) at {', '.join(f'{x:.0f}' for x in tight)} m along the route")
        sys.exit(1 if tight or dev > 1.0 else 0)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from rclpy.qos import DurabilityPolicy
    try:
        from autoware_control_msgs.msg import Control
        from autoware_vehicle_msgs.msg import GearCommand, VelocityReport
    except ImportError:
        sys.exit("autoware_msgs（autoware_control_msgs・autoware_vehicle_msgs）が無い: "
                 "docker/awsim/Dockerfile のコンテナ（tools/awsim/container.sh）で動かす")

    if args.print_pose:
        got = []

        def on_pose(m):
            p, q = m.pose.position, m.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            got.append((f"{p.x:.2f},{p.y:.2f},{math.degrees(yaw):.1f}",
                        f"{p.x:.3f},{p.y:.3f},{p.z:.3f},{math.degrees(yaw):.1f}"))

        rclpy.init()
        node = rclpy.create_node("awsim_print_pose")
        node.create_subscription(PoseStamped, args.gt_topic, on_pose,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        for _ in range(100):
            rclpy.spin_once(node, timeout_sec=0.1)
            if got:
                break
        node.destroy_node()
        rclpy.shutdown()
        if not got:
            sys.exit(f"{args.gt_topic} が 10 s 来ない（AWSIM が動いているか、ros2 topic list で確かめる）")
        print(got[0][0])
        print(f"（AWSIM をこの位置から始めるなら: tools/awsim/run_awsim.sh --start {got[0][1]}）", file=sys.stderr)
        return

    route = load_route(args.route)

    class Driver(Node):
        def __init__(self):
            super().__init__("awsim_drive")
            self.pp = PurePursuit(route, wheelbase=args.wheelbase, speed=args.speed)
            self.pose = None
            self.last = None
            self.t_start = None
            self.dist = 0.0
            self.started = False
            self.finished = False
            self.create_subscription(PoseStamped, args.gt_topic, self.on_pose,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            # 速さは真値の位置の変化から求める（時刻はメッセージの header.stamp、0.2 s の幅で）。
            # AWSIM の VelocityReport は、ぶつかった後などに ±数百 km/h の値を出すことがあったので、表示用にだけ使う
            self.hist = collections.deque(maxlen=200)  # (t, x, y)
            self.v_report = None
            self.create_subscription(VelocityReport, args.velocity_topic, self.on_velocity,
                                     QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.brake_until = None
            self.failed = False
            # AWSIM v1.3.1 の購読は RELIABLE・TRANSIENT_LOCAL・深さ 1。出す側も TRANSIENT_LOCAL でないとつながらない
            cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_ctrl = self.create_publisher(Control, "/control/command/control_cmd", cmd_qos)
            self.pub_gear = self.create_publisher(GearCommand, "/control/command/gear_cmd", cmd_qos)
            self.timer = self.create_timer(1.0 / 30.0, self.tick)

        def on_velocity(self, m):
            self.v_report = (self.get_clock().now().nanoseconds * 1e-9, float(m.longitudinal_velocity))

        def on_pose(self, m):
            p, q = m.pose.position, m.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if self.last is not None and t > self.last[0]:
                if self.started:
                    self.dist += math.hypot(p.x - self.last[1], p.y - self.last[2])
            if self.last is None or t > self.last[0]:
                self.hist.append((t, p.x, p.y))
            self.last = (t, p.x, p.y)
            self.pose = (p.x, p.y, yaw)

        def speed(self):
            """真値の位置の、この 0.2 s の変化から求めた前向きの速さ [m/s]（後ろ向きなら負）。"""
            if len(self.hist) < 2:
                return 0.0
            t1, x1, y1 = self.hist[-1]
            t0, x0, y0 = self.hist[0]
            for h in reversed(self.hist):
                if t1 - h[0] >= 0.2:
                    t0, x0, y0 = h
                    break
            if t1 - t0 < 1e-3:
                return 0.0
            yaw = self.pose[2]
            return ((x1 - x0) * math.cos(yaw) + (y1 - y0) * math.sin(yaw)) / (t1 - t0)

        def send(self, steer, v_ref, acc):
            now = self.get_clock().now().to_msg()
            c = Control()
            c.stamp = now
            c.lateral.stamp = now
            c.lateral.steering_tire_angle = float(steer)
            c.longitudinal.stamp = now
            c.longitudinal.velocity = float(v_ref)
            c.longitudinal.acceleration = float(acc)
            c.longitudinal.is_defined_acceleration = True
            self.pub_ctrl.publish(c)
            g = GearCommand()
            g.stamp = now
            g.command = GearCommand.DRIVE
            self.pub_gear.publish(g)

        def tick(self):
            if self.pose is None:
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.t_start is None:
                off = self.pp.start(self.pose[0], self.pose[1])
                self.t_start = now
                self.get_logger().info(f"route: {len(route)} points, {self.pp.remaining():.0f} m ahead "
                                       f"(start {off:.1f} m from the car). waiting {args.wait:.0f} s")
                tight = self.pp.tight_turns()
                if tight:
                    self.get_logger().warn(f"route has {len(tight)} turn(s) tighter than the car can make, at "
                                           f"{', '.join(f'{d:.0f}' for d in tight[:5])} m along the route. "
                                           "the car will leave the lane there (make the route again)")
            if now - self.t_start < args.wait:
                self.send(0.0, 0.0, -1.0)
                return
            self.started = True
            v = self.speed()
            if self.brake_until is not None:  # 終わった後も 2 s はブレーキを出し続けてから終わる
                self.send(0.0, 0.0, -2.0)
                if now > self.brake_until and abs(v) < 0.05:
                    self.finished = True
                return
            # 速さがありえない値（ぶつかった、地面から落ちたなど）なら、止めて終わる
            if abs(v) > max(2.0 * self.pp.v_ref, 15.0):
                self.get_logger().error(f"abnormal speed {v * 3.6:.0f} km/h: the car probably hit something or fell. "
                                        "stopping (restart AWSIM; tools/awsim/run_awsim.sh starts it without other cars)")
                self.send(0.0, 0.0, -2.0)
                self.failed = True
                self.finished = True
                return
            steer, v_ref, acc, done = self.pp.step(*self.pose, v)
            if args.max_distance > 0 and self.dist >= args.max_distance:
                v_ref, acc, done = 0.0, -2.0, True
            self.send(steer, v_ref, acc)
            if done and abs(v) < 0.05:
                self.get_logger().info(f"finished: {self.dist:.0f} m. braking 2 s before exit")
                self.brake_until = now + 2.0
            if int(now) != int(now - 1.0 / 30.0):  # 1 s ごとに状態を出す
                rep = f", report {self.v_report[1] * 3.6:5.1f}" if self.v_report else ""
                self.get_logger().info(f"v {v * 3.6:5.1f} km/h (target {v_ref * 3.6:5.1f}{rep}), "
                                       f"{self.pp.remaining():5.0f} m to go")

    rclpy.init()
    node = Driver()
    try:
        # 経路の終わりで止まったら抜ける（コールバックの中で rclpy.shutdown() を呼ぶと、プロセスが終わらなかった）
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.send(0.0, 0.0, -1.0)
    finally:
        failed = node.failed
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(2 if failed else 0)


if __name__ == "__main__":
    main()
