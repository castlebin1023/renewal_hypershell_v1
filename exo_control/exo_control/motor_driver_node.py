#!/usr/bin/env python3
import math
import struct
from dataclasses import dataclass
from typing import Optional

import can
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool

@dataclass
class MotorState:
    pos_motor_deg: float
    pos_joint_rad: float
    vel_motor_erpm: float
    vel_joint_rad_s: float
    vel_joint_deg_s: float
    cur_a: float
    temp_c: int
    err: int

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver_node")

        # ---------- parameters ----------
        self.declare_parameter("state_topic", "/motor/state")
        self.declare_parameter("cmd_topic", "/motor/cmd_current")
        self.declare_parameter("init_topic", "/motor/init_done")

        self.declare_parameter("channel", "can1")
        self.declare_parameter("controller_id", 0x68)
        self.declare_parameter("upload_id", 0x2968)
        self.declare_parameter("gear_ratio", 9.0)
        self.declare_parameter("pole_pairs", 21.0)
        self.declare_parameter("joint_name", "hip_joint")

        self.declare_parameter("rx_period", 0.002)
        self.declare_parameter("publish_rate", 500.0)

        self.declare_parameter("cmd_timeout", 0.2)
        self.declare_parameter("current_limit", 30.0)
        self.declare_parameter("init_countdown", 3)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.init_topic = str(self.get_parameter("init_topic").value)

        self.channel = self.get_parameter("channel").value
        self.controller_id = int(self.get_parameter("controller_id").value)
        self.upload_id = int(self.get_parameter("upload_id").value)
        self.gear_ratio = float(self.get_parameter("gear_ratio").value)
        self.pole_pairs = float(self.get_parameter("pole_pairs").value)
        self.joint_name = str(self.get_parameter("joint_name").value)

        self.rx_period = float(self.get_parameter("rx_period").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.current_limit = float(self.get_parameter("current_limit").value)
        self.init_countdown = int(self.get_parameter("init_countdown").value)
                
        self.init_done = False

        # ---------- CAN ----------
        self.bus = can.interface.Bus(channel=self.channel, interface="socketcan")

        # ---------- ROS I/O ----------
        self.state_pub = self.create_publisher(
            JointState, self.state_topic, 10
        )
        self.init_pub = self.create_publisher(
            Bool, self.init_topic, 10
        )

        self.cmd_sub = self.create_subscription(
            Float32, self.cmd_topic, self.cmd_current_cb, 10
        )
        # ---------- timers ----------
        self.rx_timer = self.create_timer(self.rx_period, self.rx_loop)
        self.pub_timer = self.create_timer(1.0 / self.publish_rate, self.publish_state)
        self.watchdog_timer = self.create_timer(0.02, self.watchdog_loop)
        self.init_timer = self.create_timer(1.0, self.init_sequence_loop)

        # ---------- state ----------
        self.latest_state: Optional[MotorState] = None
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_current = 0.0
        self.filtered_effort = None

        self.get_logger().info(
            f"Opened {self.channel}, controller_id={self.controller_id}, "
            f"upload_id=0x{self.upload_id:08X}, gear_ratio={self.gear_ratio}"
        )

    # ------------------------------------------------------------------
    # CAN send helpers
    # ------------------------------------------------------------------
    def send_current(self, current_a: float) -> None:
        current_a = max(min(current_a, self.current_limit), -self.current_limit)
        current_int = int(round(current_a * 1000.0))
        data = struct.pack(">i", current_int)
        arb_id = (1 << 8) | self.controller_id  # Servo current loop mode
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
        self.bus.send(msg)

    def set_origin(self, temporary: bool = True) -> None:
        data = bytes([0x00 if temporary else 0x01])
        arb_id = (5 << 8) | self.controller_id
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
        self.bus.send(msg)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def cmd_current_cb(self, msg: Float32) -> None:
        self.last_cmd_current = float(msg.data)
        self.last_cmd_time = self.get_clock().now()
        self.send_current(self.last_cmd_current)

    # ------------------------------------------------------------------
    # CAN receive / parse
    # ------------------------------------------------------------------
    def init_sequence_loop(self) -> None:
        if self.init_done:
            return

        if self.init_countdown > 0:
            self.get_logger().info(f"Setting origin in {self.init_countdown}...")
            self.init_countdown -= 1
            return

        try:
            self.set_origin(temporary=True)
            self.get_logger().info("Origin set at current position.")
        except Exception as e:
            self.get_logger().error(f"Failed to set origin: {e}")

        self.init_done = True

        try:
            self.init_timer.cancel()
        except Exception:
            pass

    def parse_upload(self, msg: can.Message) -> Optional[MotorState]:
        d = msg.data
        if len(d) < 8:
            return None

        pos_int = struct.unpack(">h", d[0:2])[0]
        spd_int = struct.unpack(">h", d[2:4])[0]
        cur_int = struct.unpack(">h", d[4:6])[0]
        temp_c = int(d[6])
        err = int(d[7])

        # manual conversion
        pos_motor_deg = pos_int * (-0.1)
        vel_motor_erpm = spd_int * (-10.0)
        cur_a = cur_int * 0.01

        # motor -> joint
        pos_joint_deg = pos_motor_deg / self.gear_ratio
        pos_joint_rad = math.radians(pos_joint_deg)

        motor_mech_rpm = vel_motor_erpm / self.pole_pairs
        motor_mech_rad_s = motor_mech_rpm * 2.0 * math.pi / 60.0
        vel_joint_rad_s = motor_mech_rad_s / self.gear_ratio
        vel_joint_angle = math.degrees(vel_joint_rad_s)

        return MotorState(
            pos_joint_deg=pos_joint_deg,
            pos_joint_rad=pos_joint_rad,
            vel_motor_erpm=vel_motor_erpm,
            vel_joint_rad_s=vel_joint_rad_s,
            vel_joint_deg_s=vel_joint_angle,
            cur_a=cur_a,
            temp_c=temp_c,
            err=err,
        )

    def rx_loop(self) -> None:
        latest = None
        while True:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                break
            if msg.is_extended_id and msg.arbitration_id == self.upload_id:
                latest = msg

        if latest is not None:
            parsed = self.parse_upload(latest)
            if parsed is not None:
                self.latest_state = parsed

    def publish_state(self) -> None:
        if not self.init_done:
            return
            
        init_msg = Bool()
        init_msg.data = True
        self.init_pub.publish(init_msg)

        if self.latest_state is None:
            return

        s = self.latest_state

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [s.pos_joint_rad]
        js.velocity = [s.vel_joint_rad_s]
        js.effort = [abs(s.cur_a)]  
        self.state_pub.publish(js)

    def watchdog_loop(self) -> None:
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout and abs(self.last_cmd_current) > 1e-6:
            self.last_cmd_current = 0.0
            try:
                self.get_logger().warn(
                    f"WATCHDOG TRIGGERED dt={dt:.3f}, last_cmd_current={self.last_cmd_current:.3f}"
                    )
                self.send_current(0.0)
            except Exception as e:
                self.get_logger().warn(f"Watchdog stop failed: {e}")

    def destroy_node(self):
        try:
            self.send_current(0.0)
        except Exception:
            pass
        try:
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()