#!/usr/bin/env python3
import csv
import math
import os
from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32, Bool


class SmoothTrapezoidTorqueProfileNode(Node):
    def __init__(self):
        super().__init__("smooth_trapezoid_torque_profile_node")

        # =========================
        # Topics
        # =========================
        self.declare_parameter("left_phase_topic", "/left_gait/phase")
        self.declare_parameter("right_phase_topic", "/right_gait/phase")
        self.declare_parameter("left_walking_topic", "/left_gait/is_walking")
        self.declare_parameter("right_walking_topic", "/right_gait/is_walking")
        self.declare_parameter("left_init_topic", "/left_motor/init_done")
        self.declare_parameter("right_init_topic", "/right_motor/init_done")
        self.declare_parameter("left_cmd_topic", "/left_motor/cmd_current")
        self.declare_parameter("right_cmd_topic", "/right_motor/cmd_current")

        # =========================
        # Timing
        # =========================
        self.declare_parameter("publish_rate", 500.0)

        # =========================
        # 6D torque profile parameters
        # Units: joint torque [Nm], phase [% gait cycle]
        # =========================
        self.declare_parameter("extension_peak_torque", 5.0)
        self.declare_parameter("extension_peak_time", 15.0)
        self.declare_parameter("extension_rise_time", 10.0)

        self.declare_parameter("flexion_peak_torque", 5.0)
        self.declare_parameter("flexion_peak_time", 60.0)
        self.declare_parameter("flexion_rise_time", 10.0)

        # =========================
        # Motor / transmission parameters
        # current [A] = joint_torque [Nm] / (Kt [Nm/A] * gear_ratio * efficiency)
        # =========================
        self.declare_parameter("left_motor_kt", 0.095)
        self.declare_parameter("right_motor_kt", 0.105)
        self.declare_parameter("gear_ratio", 9.0)
        self.declare_parameter("transmission_efficiency", 1.0)

        # =========================
        # Direction / safety
        # =========================
        self.declare_parameter("left_direction", -1.0)
        self.declare_parameter("right_direction", -1.0)

        self.declare_parameter("current_limit", 9.0)

        # --- NEW: current slew rate limiter ---
        self.declare_parameter("current_slew_rate", 5.0)  # [A/s]
        self.declare_parameter("enable_slew_rate", True)

        self.declare_parameter("enable_assistance", True)
        self.declare_parameter("zero_when_not_walking", True)
        self.declare_parameter("enable_csv_logging", False)

        # Hip torque sign convention
        self.declare_parameter("extension_sign", -1.0)
        self.declare_parameter("flexion_sign", 1.0)

        # =========================
        # Fixed topic parameters
        # =========================
        self.left_phase_topic = str(self.get_parameter("left_phase_topic").value)
        self.right_phase_topic = str(self.get_parameter("right_phase_topic").value)
        self.left_walking_topic = str(self.get_parameter("left_walking_topic").value)
        self.right_walking_topic = str(self.get_parameter("right_walking_topic").value)
        self.left_init_topic = str(self.get_parameter("left_init_topic").value)
        self.right_init_topic = str(self.get_parameter("right_init_topic").value)
        self.left_cmd_topic = str(self.get_parameter("left_cmd_topic").value)
        self.right_cmd_topic = str(self.get_parameter("right_cmd_topic").value)

        self.publish_rate = float(self.get_parameter("publish_rate").value)

        # =========================
        # Cached dynamic parameters
        # =========================
        self.ext_peak_torque = float(self.get_parameter("extension_peak_torque").value)
        self.ext_peak_time = float(self.get_parameter("extension_peak_time").value)
        self.ext_rise_time = float(self.get_parameter("extension_rise_time").value)

        self.flex_peak_torque = float(self.get_parameter("flexion_peak_torque").value)
        self.flex_peak_time = float(self.get_parameter("flexion_peak_time").value)
        self.flex_rise_time = float(self.get_parameter("flexion_rise_time").value)

        self.left_motor_kt = float(self.get_parameter("left_motor_kt").value)
        self.right_motor_kt = float(self.get_parameter("right_motor_kt").value)
        self.gear_ratio = float(self.get_parameter("gear_ratio").value)
        self.transmission_efficiency = float(
            self.get_parameter("transmission_efficiency").value
        )

        self.left_direction = float(self.get_parameter("left_direction").value)
        self.right_direction = float(self.get_parameter("right_direction").value)

        self.current_limit = float(self.get_parameter("current_limit").value)
        self.current_slew_rate = float(
            self.get_parameter("current_slew_rate").value
        )
        self.enable_slew_rate = bool(self.get_parameter("enable_slew_rate").value)

        self.enable_assistance = bool(self.get_parameter("enable_assistance").value)
        self.zero_when_not_walking = bool(
            self.get_parameter("zero_when_not_walking").value
        )
        self.enable_csv_logging = bool(self.get_parameter("enable_csv_logging").value)

        self.extension_sign = float(self.get_parameter("extension_sign").value)
        self.flexion_sign = float(self.get_parameter("flexion_sign").value)

        self.add_on_set_parameters_callback(self.param_update_cb)

        # =========================
        # State
        # =========================
        self.left_phase: Optional[float] = None
        self.right_phase: Optional[float] = None

        self.left_is_walking = False
        self.right_is_walking = False

        self.left_init_done = False
        self.right_init_done = False

        self.prev_left_current = 0.0
        self.prev_right_current = 0.0

        # =========================
        # ROS I/O
        # =========================
        self.create_subscription(Float32, self.left_phase_topic, self.left_phase_cb, 10)
        self.create_subscription(Float32, self.right_phase_topic, self.right_phase_cb, 10)

        self.create_subscription(Bool, self.left_walking_topic, self.left_walking_cb, 10)
        self.create_subscription(Bool, self.right_walking_topic, self.right_walking_cb, 10)

        self.create_subscription(Bool, self.left_init_topic, self.left_init_cb, 10)
        self.create_subscription(Bool, self.right_init_topic, self.right_init_cb, 10)

        self.left_cmd_pub = self.create_publisher(Float32, self.left_cmd_topic, 10)
        self.right_cmd_pub = self.create_publisher(Float32, self.right_cmd_topic, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_current)

        self.csv_file = None
        self.csv_writer = None
        if self.enable_csv_logging:
            file_path = "/home/castlebin/robot_ws/data/torque_profile_log.csv"
            # Using write mode initially to start fresh on node launch
            self.csv_file = open(file_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "time_sec",
                "left_phase",
                "left_torque_target_nm",
                "left_torque_slewed_nm",
                "right_phase",
                "right_torque_target_nm",
                "right_torque_slewed_nm",
            ])
            self.get_logger().info(f"CSV logging ENABLED -> {file_path}")

        self.get_logger().info(
            "Smooth trapezoid torque profile node started | "
            f"left_cmd={self.left_cmd_topic}, right_cmd={self.right_cmd_topic}"
        )

    # =========================
    # Parameter update callback
    # =========================
    def param_update_cb(self, params):
        new_vals = {}

        for p in params:
            try:
                if p.name == "extension_peak_torque":
                    new_vals["ext_peak_torque"] = float(p.value)
                elif p.name == "extension_peak_time":
                    new_vals["ext_peak_time"] = float(p.value)
                elif p.name == "extension_rise_time":
                    new_vals["ext_rise_time"] = float(p.value)

                elif p.name == "flexion_peak_torque":
                    new_vals["flex_peak_torque"] = float(p.value)
                elif p.name == "flexion_peak_time":
                    new_vals["flex_peak_time"] = float(p.value)
                elif p.name == "flexion_rise_time":
                    new_vals["flex_rise_time"] = float(p.value)

                elif p.name == "left_motor_kt":
                    new_vals["left_motor_kt"] = float(p.value)
                elif p.name == "right_motor_kt":
                    new_vals["right_motor_kt"] = float(p.value)
                elif p.name == "gear_ratio":
                    new_vals["gear_ratio"] = float(p.value)
                elif p.name == "transmission_efficiency":
                    new_vals["transmission_efficiency"] = float(p.value)

                elif p.name == "left_direction":
                    new_vals["left_direction"] = float(p.value)
                elif p.name == "right_direction":
                    new_vals["right_direction"] = float(p.value)

                elif p.name == "current_limit":
                    new_vals["current_limit"] = float(p.value)
                elif p.name == "current_slew_rate":
                    new_vals["current_slew_rate"] = float(p.value)
                elif p.name == "enable_slew_rate":
                    new_vals["enable_slew_rate"] = bool(p.value)

                elif p.name == "enable_assistance":
                    new_vals["enable_assistance"] = bool(p.value)
                elif p.name == "zero_when_not_walking":
                    new_vals["zero_when_not_walking"] = bool(p.value)
                elif p.name == "enable_csv_logging":
                    new_vals["enable_csv_logging"] = bool(p.value)

                elif p.name == "extension_sign":
                    new_vals["extension_sign"] = float(p.value)
                elif p.name == "flexion_sign":
                    new_vals["flexion_sign"] = float(p.value)

            except Exception as e:
                return SetParametersResult(
                    successful=False,
                    reason=f"Invalid parameter {p.name}: {e}",
                )

        # Range Validation
        if "ext_peak_torque" in new_vals and not (0.0 <= new_vals["ext_peak_torque"] <= 20.0):
            return SetParametersResult(successful=False, reason="extension_peak_torque out of range")
        if "ext_peak_time" in new_vals and not (0.0 <= new_vals["ext_peak_time"] <= 100.0):
            return SetParametersResult(successful=False, reason="extension_peak_time out of range")
        if "ext_rise_time" in new_vals and not (1.0 <= new_vals["ext_rise_time"] <= 50.0):
            return SetParametersResult(successful=False, reason="extension_rise_time out of range")
            
        if "flex_peak_torque" in new_vals and not (0.0 <= new_vals["flex_peak_torque"] <= 20.0):
            return SetParametersResult(successful=False, reason="flexion_peak_torque out of range")
        if "flex_peak_time" in new_vals and not (0.0 <= new_vals["flex_peak_time"] <= 100.0):
            return SetParametersResult(successful=False, reason="flexion_peak_time out of range")
        if "flex_rise_time" in new_vals and not (1.0 <= new_vals["flex_rise_time"] <= 50.0):
            return SetParametersResult(successful=False, reason="flexion_rise_time out of range")

        for key, value in new_vals.items():
            setattr(self, key, value)

        if "enable_csv_logging" in new_vals:
            if self.enable_csv_logging and getattr(self, "csv_file", None) is None:
                file_path = "/home/castlebin/robot_ws/data/torque_profile_log.csv"
                file_exists = os.path.isfile(file_path)
                self.csv_file = open(file_path, "a", newline="")
                self.csv_writer = csv.writer(self.csv_file)
                if not file_exists:
                    self.csv_writer.writerow([
                        "time_sec",
                        "left_phase",
                        "left_torque_target_nm",
                        "left_torque_slewed_nm",
                        "right_phase",
                        "right_torque_target_nm",
                        "right_torque_slewed_nm",
                    ])
                self.get_logger().info(f"CSV logging ENABLED dynamically -> {file_path}")
            elif not self.enable_csv_logging and getattr(self, "csv_file", None) is not None:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
                self.get_logger().info("CSV logging DISABLED dynamically")

        return SetParametersResult(successful=True)

    # =========================
    # Callbacks
    # =========================
    def left_phase_cb(self, msg: Float32):
        self.left_phase = float(msg.data)

    def right_phase_cb(self, msg: Float32):
        self.right_phase = float(msg.data)

    def left_walking_cb(self, msg: Bool):
        self.left_is_walking = bool(msg.data)

    def right_walking_cb(self, msg: Bool):
        self.right_is_walking = bool(msg.data)

    def left_init_cb(self, msg: Bool):
        self.left_init_done = bool(msg.data)

    def right_init_cb(self, msg: Bool):
        self.right_init_done = bool(msg.data)

    # =========================
    # Helpers
    # =========================
    @staticmethod
    def clamp(x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def apply_current_slew_limit(self, target: float, prev: float) -> float:
        """
        Limit how fast current can change.

        target : desired current [A]
        prev   : previous output current [A]
        """
        if not self.enable_slew_rate:
            return target

        dt = 1.0 / max(self.publish_rate, 1e-6)
        max_delta = self.current_slew_rate * dt
        delta = target - prev
        delta = self.clamp(delta, -max_delta, max_delta)
        return prev + delta

    @staticmethod
    def smoothstep_half_cosine(s: float) -> float:
        s = max(0.0, min(1.0, s))
        return 0.5 - 0.5 * math.cos(math.pi * s)

    def smooth_trapezoid_pulse(
        self,
        phase: float,
        start: float,
        rise_end: float,
        fall_start: float,
        end: float,
        peak_tau: float,
    ) -> float:
        if phase < start or phase > end:
            return 0.0

        if start <= phase < rise_end:
            denom = max(rise_end - start, 1e-6)
            s = (phase - start) / denom
            return peak_tau * self.smoothstep_half_cosine(s)

        if rise_end <= phase <= fall_start:
            return peak_tau

        if fall_start < phase <= end:
            denom = max(end - fall_start, 1e-6)
            s = (phase - fall_start) / denom
            return peak_tau * (1.0 - self.smoothstep_half_cosine(s))

        return 0.0

    # =========================
    # Torque profile
    # =========================
    def compute_torque(self, phase_pct: float) -> float:
        """
        phase_pct: 0~100 [%GC]
        return: desired hip joint torque [Nm]
        """
        phase = self.clamp(phase_pct, 0.0, 100.0)

        f1 = self.clamp(self.ext_peak_torque, 0.0, 20.0)
        f2 = self.clamp(self.ext_peak_time, 0.0, 100.0)
        f3 = self.clamp(self.ext_rise_time, 1.0, 50.0)

        f4 = self.clamp(self.flex_peak_torque, 0.0, 20.0)
        f5 = self.clamp(self.flex_peak_time, 0.0, 100.0)
        f6 = self.clamp(self.flex_rise_time, 1.0, 50.0)

        ext_plateau_half = 0.20 * f3
        flex_plateau_half = 0.20 * f6

        ext_start = f2 - f3
        ext_rise_end = f2 - ext_plateau_half
        ext_fall_start = f2 + ext_plateau_half
        ext_end = f2 + f3

        flex_start = f5 - f6
        flex_rise_end = f5 - flex_plateau_half
        flex_fall_start = f5 + flex_plateau_half
        flex_end = f5 + f6

        tau_ext = self.smooth_trapezoid_pulse(
            phase=phase,
            start=ext_start,
            rise_end=ext_rise_end,
            fall_start=ext_fall_start,
            end=ext_end,
            peak_tau=self.extension_sign * f1,
        )

        tau_flex = self.smooth_trapezoid_pulse(
            phase=phase,
            start=flex_start,
            rise_end=flex_rise_end,
            fall_start=flex_fall_start,
            end=flex_end,
            peak_tau=self.flexion_sign * f4,
        )

        return tau_ext + tau_flex

    # =========================
    # Torque -> current
    # =========================
    def torque_to_current(self, tau_joint_nm: float, side: str) -> float:
        """
        Desired joint torque [Nm] -> motor current [A]

        tau_joint = Kt * I * gear_ratio * efficiency
        I = tau_joint / (Kt * gear_ratio * efficiency)
        """
        if side == "left":
            kt = self.left_motor_kt
            direction = self.left_direction
        else:
            kt = self.right_motor_kt
            direction = self.right_direction

        kt = max(kt, 1e-6)
        gear_ratio = max(self.gear_ratio, 1e-6)
        efficiency = max(self.transmission_efficiency, 1e-6)

        current = tau_joint_nm / (kt * gear_ratio * efficiency)
        current = direction * current

        return self.clamp(current, -self.current_limit, self.current_limit)

    def current_to_torque(self, current_a: float, side: str) -> float:
        if side == "left":
            kt = self.left_motor_kt
            direction = self.left_direction
        else:
            kt = self.right_motor_kt
            direction = self.right_direction

        kt = max(kt, 1e-6)
        gear_ratio = max(self.gear_ratio, 1e-6)
        efficiency = max(self.transmission_efficiency, 1e-6)

        return current_a * direction * kt * gear_ratio * efficiency

    # =========================
    # Main loop
    # =========================
    def publish_current(self):
        left_current = 0.0
        right_current = 0.0

        if (
            self.enable_assistance
            and self.left_init_done
            and self.left_phase is not None
            and (self.left_is_walking or not self.zero_when_not_walking)
        ):
            tau_left = self.compute_torque(self.left_phase)
            left_current = self.torque_to_current(tau_left, side="left")

        if (
            self.enable_assistance
            and self.right_init_done
            and self.right_phase is not None
            and (self.right_is_walking or not self.zero_when_not_walking)
        ):
            tau_right = self.compute_torque(self.right_phase)
            right_current = self.torque_to_current(tau_right, side="right")

        # --- NEW: apply slew-rate limiter ---
        left_current = self.apply_current_slew_limit(
            left_current,
            self.prev_left_current
        )

        right_current = self.apply_current_slew_limit(
            right_current,
            self.prev_right_current
        )

        self.prev_left_current = left_current
        self.prev_right_current = right_current


        left_msg = Float32()
        left_msg.data = float(left_current)
        self.left_cmd_pub.publish(left_msg)

        right_msg = Float32()
        right_msg.data = float(right_current)
        self.right_cmd_pub.publish(right_msg)

        if self.enable_csv_logging and getattr(self, "csv_writer", None) is not None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            left_torque_slewed  = self.current_to_torque(left_current,  "left")
            right_torque_slewed = self.current_to_torque(right_current, "right")

            self.csv_writer.writerow([
                now_sec,
                float(self.left_phase) if self.left_phase is not None else 0.0,
                float(tau_left) if 'tau_left' in locals() else 0.0,
                float(left_torque_slewed),
                float(self.right_phase) if self.right_phase is not None else 0.0,
                float(tau_right) if 'tau_right' in locals() else 0.0,
                float(right_torque_slewed),
            ])
            self.csv_file.flush()

    def destroy_node(self):
        try:
            msg = Float32()
            msg.data = 0.0
            self.left_cmd_pub.publish(msg)
            self.right_cmd_pub.publish(msg)
        except Exception:
            pass

        if getattr(self, "enable_csv_logging", False) and getattr(self, "csv_file", None) is not None:
            try:
                self.csv_file.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SmoothTrapezoidTorqueProfileNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()