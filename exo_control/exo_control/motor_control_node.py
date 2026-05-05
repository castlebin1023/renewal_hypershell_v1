#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool


class AdmittanceCurrentNode(Node):
    def __init__(self):
        super().__init__("admittance_current_node")
        # =========================
        # Parameters
        # =========================
        self.declare_parameter("dt", 0.01)                   # 100 Hz

        self.declare_parameter("current_bias", 1.0)         # preload 걸린 정지 상태 effort baseline
        self.declare_parameter("current_deadband", 0.15)
        self.declare_parameter("lpf_alpha", 0.1)            # Low-pass filter alpha (0.0 to 1.0)

        self.declare_parameter("preload_current_a", -1.0)   # 항상 깔리는 CW preload
        self.declare_parameter("current_limit", 8.0)        # total current saturation
        self.declare_parameter("enable_output", True)
        self.declare_parameter("state_topic", "/motor/state")
        self.declare_parameter("cmd_topic", "/motor/cmd_current")
        self.declare_parameter("init_topic", "/motor/init_done")

        # =========================
        # Load parameters
        # =========================
        self.dt = float(self.get_parameter("dt").value)
        
        self.current_bias = float(self.get_parameter("current_bias").value)
        self.current_deadband = float(self.get_parameter("current_deadband").value)
        self.lpf_alpha = float(self.get_parameter("lpf_alpha").value)

        self.preload_current_a = float(self.get_parameter("preload_current_a").value)
        self.current_limit = float(self.get_parameter("current_limit").value)
        self.enable_output = bool(self.get_parameter("enable_output").value)
        
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.init_topic = str(self.get_parameter("init_topic").value)

        # =========================
        # ROS I/O
        # =========================
        self.state_sub = self.create_subscription(
            JointState, self.state_topic, self.state_cb, 10
        )
        self.init_sub = self.create_subscription(
            Bool, self.init_topic, self.init_cb, 10
        )
        self.cmd_pub = self.create_publisher(Float32, self.cmd_topic, 10)
        self.timer = self.create_timer(self.dt, self.update)

        # =========================
        # State & Safety
        # =========================
        self.current_a = 0.0
        self.filter_stage1 = 0.0
        self.last_i_eff = 0.0
        self.last_i_assist = 0.0
        
        self.last_i_cmd = 0.0  
        
        self.active_cmd_sign = -1.0  
        
        self.startup_counter = 0
        self.startup_duration = int(1.0 / self.dt) 
        
        self.driver_init_done = False

        self.get_logger().info(f"AdmittanceCurrentNode started | Waiting for init_done = True")

    def init_cb(self, msg: Bool) -> None:
        if msg.data and not self.driver_init_done:
            self.get_logger().info("Driver init_done is True! Starting control loop.")
            self.driver_init_done = True

    def state_cb(self, msg: JointState) -> None:
        if len(msg.position) > 0:
            self.joint_pos_meas = float(msg.position[0])
        if len(msg.velocity) > 0:
            self.joint_vel_meas = float(msg.velocity[0])
        if len(msg.effort) > 0:
            self.current_a = float(msg.effort[0])

    @staticmethod
    def clamp(x: float, lo: float, hi: float) -> float:
        return max(min(x, hi), lo)

    def update(self) -> None:
        if not self.driver_init_done:
            return

        # =========================================================
        # 0) Soft Start 
        # =========================================================
        if self.startup_counter < self.startup_duration:
            self.startup_counter += 1
            i_cmd = self.preload_current_a
            self.last_i_cmd = i_cmd
            
            if self.enable_output:
                msg = Float32()
                msg.data = float(i_cmd)
                self.cmd_pub.publish(msg)
            return

        # ---------------------------------
        # 1) Estimation Human force
        # ---------------------------------
        cmd_mag = abs(self.last_i_cmd)
        meas_mag = abs(self.current_a)
        
        load_diff = meas_mag - cmd_mag 
        
        sign_threshold = 0.7 
        if self.last_i_cmd > sign_threshold:
            self.active_cmd_sign = 1.0
        elif self.last_i_cmd < -sign_threshold:
            self.active_cmd_sign = -1.0
        
        raw_i_eff = load_diff * (-self.active_cmd_sign)

        # ---------------------------------
        # 2) Continuous Deadband
        # ---------------------------------
        if raw_i_eff > self.current_deadband:
            raw_i_eff = raw_i_eff - self.current_deadband
        elif raw_i_eff < -self.current_deadband:
            raw_i_eff = raw_i_eff + self.current_deadband
        else:
            raw_i_eff = 0.0

        # ---------------------------------
        # 3) 2nd-order Low-pass filter
        # ---------------------------------
        self.filter_stage1 = self.lpf_alpha * raw_i_eff + (1.0 - self.lpf_alpha) * self.filter_stage1
        i_eff = self.lpf_alpha * self.filter_stage1 + (1.0 - self.lpf_alpha) * self.last_i_eff

        # ---------------------------------
        # 4) Assist torque Calculation
        # ---------------------------------
        assist_gain = 40.0
        i_assist = assist_gain * i_eff 
        # ---------------------------------
        # 5) Final Command Calculation
        # ---------------------------------
        target_i_cmd = self.preload_current_a + i_assist
        target_i_cmd = self.clamp(target_i_cmd, -self.current_limit, self.current_limit)

        max_delta = 0.15
        cmd_diff = target_i_cmd - self.last_i_cmd
        
        if cmd_diff > max_delta:
            i_cmd = self.last_i_cmd + max_delta
        elif cmd_diff < -max_delta:
            i_cmd = self.last_i_cmd - max_delta
        else:
            i_cmd = target_i_cmd

        # store
        self.last_i_eff = i_eff
        self.last_i_assist = i_assist
        self.last_i_cmd = i_cmd

        # publish
        if self.enable_output:
            msg = Float32()
            msg.data = float(i_cmd)
            self.cmd_pub.publish(msg)

    def destroy_node(self):
        try:
            stop_msg = Float32()
            stop_msg.data = 0.0
            self.cmd_pub.publish(stop_msg)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AdmittanceCurrentNode()
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