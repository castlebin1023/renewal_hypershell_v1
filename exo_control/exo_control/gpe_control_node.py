#!/usr/bin/env python3
import csv
import math
import os
from datetime import datetime
from collections import deque
from typing import Optional

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool


class GaitPhaseEstimationNode(Node):
    def __init__(self):
        super().__init__("gait_phase_estimation_node")

        self.declare_parameter("state_topic", "/motor/state")
        self.declare_parameter("init_topic", "/motor/init_done")
        self.declare_parameter("phase_topic", "/gait/phase")
        self.declare_parameter("heel_strike_topic", "/gait/heel_strike")
        self.declare_parameter("walking_state_topic", "/gait/is_walking")

        self.declare_parameter("publish_rate", 500.0)
        self.declare_parameter("window_size", 500)
        self.declare_parameter("enable_plot", True)
        self.declare_parameter("plot_rate", 30.0)
        self.declare_parameter("enable_csv_logging", False)

        # GO detection mode
        self.declare_parameter("use_steady_go_detection", True)
        self.declare_parameter("go_vel_rms_threshold", 0.18)

        # CSP
        self.declare_parameter("csp_k", 1.5)

        # HS detection
        self.declare_parameter("peak_velocity_threshold", 0.05)
        self.declare_parameter("hs_angle_ratio", 0.75)
        self.declare_parameter("hs_refractory_time", 0.45)

        # steady walking detection
        self.declare_parameter("steady_window_size", 100)
        self.declare_parameter("min_angle_range_for_walking", 0.07)
        self.declare_parameter("steady_min_peaks", 3)
        self.declare_parameter("steady_stride_cv_threshold", 0.5)
        self.declare_parameter("steady_peak_ratio", 0.75)

        # STOP / GO smoothing
        self.declare_parameter("walking_window_size", 50)
        self.declare_parameter("go_count_threshold", 2)
        self.declare_parameter("stop_count_threshold", 80)
        self.declare_parameter("stop_vel_rms_threshold", 0.12)
        self.declare_parameter("go_cooldown_time", 1.0)

        # -------------------------------------------------------
        # 캘리브레이션 시간 파라미터 (새로 추가)
        # -------------------------------------------------------
        self.declare_parameter("csp_calibration_time", 5.0)

        self.state_topic = str(self.get_parameter("state_topic").value)
        self.init_topic = str(self.get_parameter("init_topic").value)
        self.phase_topic = str(self.get_parameter("phase_topic").value)
        self.heel_strike_topic = str(self.get_parameter("heel_strike_topic").value)
        self.walking_state_topic = str(self.get_parameter("walking_state_topic").value)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.window_size = int(self.get_parameter("window_size").value)
        self.enable_plot = bool(self.get_parameter("enable_plot").value)
        self.plot_rate = float(self.get_parameter("plot_rate").value)
        self.enable_csv_logging = bool(self.get_parameter("enable_csv_logging").value)

        self.use_steady_go_detection = bool(
            self.get_parameter("use_steady_go_detection").value
        )
        self.go_vel_rms_threshold = float(
            self.get_parameter("go_vel_rms_threshold").value
        )

        self.csp_k = float(self.get_parameter("csp_k").value)

        self.peak_velocity_threshold = float(self.get_parameter("peak_velocity_threshold").value)
        self.hs_angle_ratio = float(self.get_parameter("hs_angle_ratio").value)
        self.hs_refractory_time = float(self.get_parameter("hs_refractory_time").value)

        self.steady_window_size = int(self.get_parameter("steady_window_size").value)
        self.min_angle_range_for_walking = float(self.get_parameter("min_angle_range_for_walking").value)
        self.steady_min_peaks = int(self.get_parameter("steady_min_peaks").value)
        self.steady_stride_cv_threshold = float(self.get_parameter("steady_stride_cv_threshold").value)
        self.steady_peak_ratio = float(self.get_parameter("steady_peak_ratio").value)

        self.walking_window_size = int(self.get_parameter("walking_window_size").value)
        self.go_count_threshold = int(self.get_parameter("go_count_threshold").value)
        self.stop_count_threshold = int(self.get_parameter("stop_count_threshold").value)
        self.stop_vel_rms_threshold = float(self.get_parameter("stop_vel_rms_threshold").value)
        self.go_cooldown_time = float(self.get_parameter("go_cooldown_time").value)

        self.csp_calibration_time = float(self.get_parameter("csp_calibration_time").value)

        self.state_sub = self.create_subscription(
            JointState, self.state_topic, self.state_cb, 10
        )
        self.init_sub = self.create_subscription(
            Bool, self.init_topic, self.init_cb, 10
        )

        self.phase_pub = self.create_publisher(Float32, self.phase_topic, 10)
        self.hs_pub = self.create_publisher(Bool, self.heel_strike_topic, 10)
        self.walking_pub = self.create_publisher(Bool, self.walking_state_topic, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_phase)
        self.plot_timer = None

        # ---------- CSV logging ----------
        if self.enable_csv_logging:
            log_dir = "/home/castlebin/robot_ws/data"
            os.makedirs(log_dir, exist_ok=True)
            node_name = self.get_name()
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_path = os.path.join(log_dir, f"gait_phase_{node_name}_{timestamp}.csv")
            self.csv_file = open(csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "time_sec",
                "phase_pct",
                "heel_strike",
                "is_walking",
                "angle_rad",
                "vel_rad_s",
            ])
            self.get_logger().info(f"CSV logging ENABLED → {csv_path}")
        else:
            self.csv_file = None
            self.csv_writer = None
            self.get_logger().info("CSV logging DISABLED")

        self.init_done = False

        self.latest_angle: Optional[float] = None
        self.latest_vel: Optional[float] = None
        self.latest_time: Optional[float] = None

        self.prev_angle: Optional[float] = None
        self.prev_vel: Optional[float] = None
        self.prev_time: Optional[float] = None

        self.current_phase = 0.0
        self.hs_detected_flag = False

        # -------------------------------------------------------
        # 보행 상태 분리
        #   is_walking      : 내부 보행 감지 상태 (angle/vel window 쌓기용)
        #   _torque_enabled : 외부에 publish하는 is_walking (토크 허용 여부)
        #   _calibrating    : GO 감지 후 캘리브레이션 중 플래그
        # -------------------------------------------------------
        self.is_walking = False
        self._torque_enabled = False
        self._calibrating = False
        self._calib_start_time: Optional[float] = None

        self.go_counter = 0
        self.stop_counter = 0
        self.last_stop_time: Optional[float] = None

        self.walking_vel_window = deque(maxlen=self.walking_window_size)
        self.steady_angle_window = deque(maxlen=self.steady_window_size)
        self.steady_time_window = deque(maxlen=self.steady_window_size)

        self.angle_window = deque(maxlen=self.window_size)
        self.vel_window = deque(maxlen=self.window_size)

        self.last_hs_angle: Optional[float] = None
        self.last_hs_time: Optional[float] = None
        self.hs_x = deque(maxlen=50)
        self.hs_y = deque(maxlen=50)

        # ---- CSP cache ----
        self._csp_current_angle: Optional[float] = None
        self._csp_x_now: Optional[float] = None
        self._csp_y_now: Optional[float] = None
        self._csp_x_data: list = []
        self._csp_y_data: list = []

        # -------------------------------------------------------
        # CSP 고정 파라미터 (새로 추가)
        # -------------------------------------------------------
        self._csp_params_frozen = False
        self._frozen_scale: float = 1.0
        self._frozen_alpha: float = 0.0
        self._frozen_beta: float = 0.0

        # ---- Monotonic phase filter ----
        self.prev_phase_mono: float = 0.0

        if self.enable_plot:
            plt.ion()
            self.fig, self.ax = plt.subplots()
            self.fig.canvas.manager.set_window_title(self.get_name())
            self.ax.set_aspect("equal")
            self.ax.grid(True)
            self.ax.set_xlabel("CSP x")
            self.ax.set_ylabel("CSP y")

            self.portrait_line, = self.ax.plot([], [], ".", markersize=3,
                                               color="steelblue", label="Portrait")
            self.current_point, = self.ax.plot([], [], "ko", markersize=8, label="Current")
            self.hs_scatter, = self.ax.plot([], [], "r*", markersize=14, label="Heel Strike")
            self.center_to_current_line, = self.ax.plot([], [], "k-", linewidth=2,
                                                        label="Current Angle")
            self.center_to_hs_line, = self.ax.plot([], [], "r-", linewidth=2,
                                                   label="HS Angle")
            self.ax.legend()
            self.plot_timer = self.create_timer(1.0 / self.plot_rate, self.plot_timer_cb)

        self.get_logger().info(
            f"GPE Node Started | use_steady_go_detection={self.use_steady_go_detection}, "
            f"csp_k={self.csp_k}, calib_time={self.csp_calibration_time}s"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def init_cb(self, msg: Bool):
        self.init_done = bool(msg.data)

    def state_cb(self, msg: JointState):
        if len(msg.position) == 0 or len(msg.velocity) == 0:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        angle = float(msg.position[0])
        vel = float(msg.velocity[0])

        self.prev_angle = self.latest_angle
        self.prev_vel = self.latest_vel
        self.prev_time = self.latest_time

        self.latest_angle = angle
        self.latest_vel = vel
        self.latest_time = now_sec

        self.walking_vel_window.append(vel)
        self.steady_angle_window.append(angle)
        self.steady_time_window.append(now_sec)

        self.update_stop_go_state()

        if self.is_walking:
            self.angle_window.append(angle)
            self.vel_window.append(vel)

            # 캘리브레이션 완료 여부 확인
            self._check_calibration_complete(now_sec)

            self.detect_heel_strike()
            self._update_csp_cache()
        else:
            self.hs_detected_flag = False
            self.current_phase = 0.0
            self.last_hs_angle = None
            self.last_hs_time = None
            self._csp_current_angle = None

    # ------------------------------------------------------------------
    # State helpers
    # ------------------------------------------------------------------

    def clear_gait_state(self):
        self.hs_detected_flag = False
        self.current_phase = 0.0
        self.prev_phase_mono = 0.0
        self.last_hs_angle = None
        self.last_hs_time = None
        self._csp_current_angle = None
        self._csp_x_now = None
        self._csp_y_now = None
        self._csp_x_data = []
        self._csp_y_data = []

        self.angle_window.clear()
        self.vel_window.clear()
        self.hs_x.clear()
        self.hs_y.clear()

    def clear_detection_windows(self):
        self.walking_vel_window.clear()
        self.steady_angle_window.clear()
        self.steady_time_window.clear()

    def _reset_csp_frozen(self):
        """CSP 고정 파라미터 초기화 (STOP 시 호출)."""
        self._csp_params_frozen = False
        self._frozen_scale = 1.0
        self._frozen_alpha = 0.0
        self._frozen_beta = 0.0

    # ------------------------------------------------------------------
    # 캘리브레이션 완료 체크 (새로 추가)
    # ------------------------------------------------------------------

    def _check_calibration_complete(self, now_sec: float):
        """
        GO 감지 후 csp_calibration_time(5초) 경과 여부를 확인.
        5초 경과 시:
          1. 현재 window 기반으로 alpha/beta/scale 고정
          2. _torque_enabled = True → 토크 프로파일 노드에 is_walking=True 전송
        """
        if not self._calibrating:
            return
        if self._calib_start_time is None:
            return

        elapsed = now_sec - self._calib_start_time
        remaining = self.csp_calibration_time - elapsed

        # 1초마다 남은 시간 로그
        if hasattr(self, '_last_calib_log_time'):
            if now_sec - self._last_calib_log_time >= 1.0:
                self.get_logger().info(
                    f"CSP calibrating... {remaining:.1f}s remaining"
                )
                self._last_calib_log_time = now_sec
        else:
            self._last_calib_log_time = now_sec

        if elapsed < self.csp_calibration_time:
            return

        # ---- 5초 경과: 파라미터 고정 ----
        if len(self.angle_window) < 20 or len(self.vel_window) < 20:
            # 데이터 부족 → 잠시 더 기다림
            self._calib_start_time = now_sec
            self.get_logger().warn("CSP calibration: window too small, extending...")
            return

        theta_max = max(self.angle_window)
        theta_min = min(self.angle_window)
        vel_max   = max(self.vel_window)
        vel_min   = min(self.vel_window)

        theta_range = theta_max - theta_min
        vel_range   = vel_max   - vel_min

        if theta_range < 1e-6 or vel_range < 1e-6:
            self._calib_start_time = now_sec
            self.get_logger().warn("CSP calibration: range too small, extending...")
            return

        self._frozen_scale = abs(vel_range / theta_range)
        self._frozen_alpha = 0.5 * (theta_max + theta_min)
        self._frozen_beta  = 0.5 * (vel_max  + vel_min)
        self._csp_params_frozen = True

        # 캘리브레이션 완료 → 토크 허용
        self._calibrating = False
        self._torque_enabled = True

        self.get_logger().info(
            f"CSP params FROZEN | "
            f"alpha={self._frozen_alpha:.4f} rad, "
            f"beta={self._frozen_beta:.4f} rad/s, "
            f"scale={self._frozen_scale:.4f} | "
            f"Torque ENABLED"
        )

    # ------------------------------------------------------------------
    # Walking detection
    # ------------------------------------------------------------------

    def detect_repeated_angle_waveform(self) -> bool:
        if len(self.steady_angle_window) < self.steady_window_size:
            return False

        angles = list(self.steady_angle_window)
        times = list(self.steady_time_window)

        angle_min = min(angles)
        angle_max = max(angles)
        angle_range = angle_max - angle_min

        if angle_range < self.min_angle_range_for_walking:
            return False

        peak_threshold = angle_min + self.steady_peak_ratio * angle_range
        peak_times = []

        for i in range(1, len(angles) - 1):
            is_local_max = (
                angles[i - 1] < angles[i]
                and angles[i] >= angles[i + 1]
                and angles[i] >= peak_threshold
            )
            if not is_local_max:
                continue
            if len(peak_times) == 0 or times[i] - peak_times[-1] > 0.35:
                peak_times.append(times[i])

        if len(peak_times) < self.steady_min_peaks:
            return False

        intervals = [
            peak_times[i] - peak_times[i - 1]
            for i in range(1, len(peak_times))
        ]
        intervals = [dt for dt in intervals if 0.45 <= dt <= 2.0]

        if len(intervals) < self.steady_min_peaks - 1:
            return False

        mean_dt = sum(intervals) / len(intervals)
        if mean_dt <= 1e-6:
            return False

        std_dt = math.sqrt(
            sum((dt - mean_dt) ** 2 for dt in intervals) / len(intervals)
        )
        cv = std_dt / mean_dt
        return cv <= self.steady_stride_cv_threshold

    def update_stop_go_state(self):
        if len(self.walking_vel_window) < self.walking_window_size:
            return

        vel_rms = math.sqrt(
            sum(v * v for v in self.walking_vel_window) / len(self.walking_vel_window)
        )
        repeated_waveform = self.detect_repeated_angle_waveform()

        if not self.is_walking:
            if self.last_stop_time is not None and self.latest_time is not None:
                if self.latest_time - self.last_stop_time < self.go_cooldown_time:
                    self.go_counter = 0
                    return

            go_condition = repeated_waveform if self.use_steady_go_detection \
                else vel_rms > self.go_vel_rms_threshold

            if go_condition:
                self.go_counter += 1
            else:
                self.go_counter = 0

            if self.go_counter >= self.go_count_threshold:
                self.is_walking = True
                self.stop_counter = 0
                self.go_counter = 0
                self.clear_gait_state()

                # -----------------------------------------------
                # 캘리브레이션 모드 진입 (토크는 아직 비활성화)
                # -----------------------------------------------
                self._calibrating = True
                self._torque_enabled = False
                self._csp_params_frozen = False
                self._calib_start_time = self.latest_time

                if self.use_steady_go_detection:
                    self.get_logger().info(
                        f"GO detected | steady waveform | "
                        f"CSP calibrating for {self.csp_calibration_time:.1f}s (torque OFF)"
                    )
                else:
                    self.get_logger().info(
                        f"GO detected | vel_rms={vel_rms:.3f} | "
                        f"CSP calibrating for {self.csp_calibration_time:.1f}s (torque OFF)"
                    )
        else:
            stop_condition = (
                (not repeated_waveform) or (vel_rms < self.stop_vel_rms_threshold)
                if self.use_steady_go_detection
                else vel_rms < self.stop_vel_rms_threshold
            )

            if stop_condition:
                self.stop_counter += 1
            else:
                self.stop_counter = 0

            if self.stop_counter >= self.stop_count_threshold:
                self.is_walking = False
                self.stop_counter = 0
                self.go_counter = 0
                self.last_stop_time = self.latest_time

                # -----------------------------------------------
                # STOP: 토크 비활성화 + 고정 파라미터 해제
                # -----------------------------------------------
                self._torque_enabled = False
                self._calibrating = False
                self._reset_csp_frozen()

                self.clear_gait_state()
                self.clear_detection_windows()

                self.get_logger().info(
                    f"STOP detected | vel_rms={vel_rms:.3f}, "
                    f"repeated_waveform={repeated_waveform} | Torque DISABLED"
                )

    # ------------------------------------------------------------------
    # Heel strike detection
    # ------------------------------------------------------------------

    def detect_heel_strike(self):
        if not self.is_walking:
            return
        if (
            self.prev_vel is None
            or self.latest_vel is None
            or self.latest_angle is None
            or self.latest_time is None
        ):
            return
        if len(self.angle_window) < 20:
            return

        theta_min = min(self.angle_window)
        theta_max = max(self.angle_window)
        theta_range = theta_max - theta_min

        if theta_range < self.min_angle_range_for_walking:
            return

        near_max_angle = self.latest_angle >= theta_min + self.hs_angle_ratio * theta_range
        if not near_max_angle:
            return

        velocity_cross_down = (
            self.prev_vel > self.peak_velocity_threshold
            and self.latest_vel <= self.peak_velocity_threshold
        )
        if not velocity_cross_down:
            return

        if self.last_hs_time is not None:
            if self.latest_time - self.last_hs_time < self.hs_refractory_time:
                return

        self.last_hs_time = self.latest_time
        self.hs_detected_flag = True

    # ------------------------------------------------------------------
    # CSP helpers
    # ------------------------------------------------------------------

    def apply_csp_transform(self, x: float, y: float):
        k = self.csp_k
        x_t = 0.5 * ((k + 1.0) * x + (1.0 - k) * y)
        y_t = 0.5 * ((1.0 - k) * x + (k + 1.0) * y)
        return x_t, y_t

    def _update_csp_cache(self):
        """
        CSP 현재 위상 포인트 계산.

        캘리브레이션 중(_calibrating=True):
            window로 alpha/beta/scale을 매번 계산 (기존 동작 유지).
            → window가 채워지면서 안정적인 값을 확보.

        캘리브레이션 완료(_csp_params_frozen=True):
            고정된 _frozen_alpha/beta/scale을 사용.
            → 토크가 켜져도 파라미터가 오염되지 않음.
        """
        if len(self.angle_window) < 20 or len(self.vel_window) < 20:
            self._csp_current_angle = None
            return

        if self._csp_params_frozen:
            # 고정 파라미터 사용
            scale = self._frozen_scale
            alpha = self._frozen_alpha
            beta  = self._frozen_beta
        else:
            # 캘리브레이션 중: window로 계산
            theta_max = max(self.angle_window)
            theta_min = min(self.angle_window)
            vel_max   = max(self.vel_window)
            vel_min   = min(self.vel_window)

            theta_range = theta_max - theta_min
            vel_range   = vel_max   - vel_min

            if theta_range < 1e-6 or vel_range < 1e-6:
                self._csp_current_angle = None
                return

            scale = abs(vel_range / theta_range)
            alpha = 0.5 * (theta_max + theta_min)
            beta  = 0.5 * (vel_max  + vel_min)

        xr = scale * (self.latest_angle - alpha)
        yr = self.latest_vel - beta
        x_now, y_now = self.apply_csp_transform(xr, yr)
        self._csp_x_now = x_now
        self._csp_y_now = y_now

        current_angle = math.atan2(y_now, x_now)
        if current_angle < 0:
            current_angle += 2.0 * math.pi
        self._csp_current_angle = current_angle

        if self.hs_detected_flag:
            self.last_hs_angle = current_angle
            self.hs_x.append(x_now)
            self.hs_y.append(y_now)

    # ------------------------------------------------------------------
    # Phase computation
    # ------------------------------------------------------------------

    def _compute_phase(self):
        """CSP 캐시에서 gait phase 계산."""
        if self._csp_current_angle is None or self.last_hs_angle is None:
            self.current_phase = 0.0
            return

        if self.hs_detected_flag:
            self.prev_phase_mono = 0.0
            self.current_phase = 0.0
            return

        phase_angle = self.last_hs_angle - self._csp_current_angle
        if phase_angle < 0:
            phase_angle += 2.0 * math.pi

        raw_phase = phase_angle / (2.0 * math.pi)

        wrap_high = 0.90
        wrap_low  = 0.10
        jump_threshold = 0.5

        if self.prev_phase_mono > wrap_high and raw_phase < wrap_low:
            phase_mono = raw_phase
        elif raw_phase - self.prev_phase_mono > jump_threshold:
            phase_mono = self.prev_phase_mono
        else:
            phase_mono = max(self.prev_phase_mono, raw_phase)

        phase_mono = min(max(phase_mono, 0.0), 1.0)
        self.prev_phase_mono = phase_mono
        self.current_phase = phase_mono

    # ------------------------------------------------------------------
    # Plot (선택적)
    # ------------------------------------------------------------------

    def _compute_plot_portrait(self):
        """Compute full CSP portrait only for visualization at low rate."""
        if len(self.angle_window) < 20 or len(self.vel_window) < 20:
            self._csp_x_data = []
            self._csp_y_data = []
            return

        if self._csp_params_frozen:
            scale = self._frozen_scale
            alpha = self._frozen_alpha
            beta  = self._frozen_beta
        else:
            theta_max = max(self.angle_window)
            theta_min = min(self.angle_window)
            vel_max = max(self.vel_window)
            vel_min = min(self.vel_window)

            theta_range = theta_max - theta_min
            vel_range = vel_max - vel_min

            if theta_range < 1e-6 or vel_range < 1e-6:
                self._csp_x_data = []
                self._csp_y_data = []
                return

            scale = abs(vel_range / theta_range)
            alpha = 0.5 * (theta_max + theta_min)
            beta = 0.5 * (vel_max + vel_min)

        x_data, y_data = [], []
        for theta, vel in zip(self.angle_window, self.vel_window):
            xr = scale * (theta - alpha)
            yr = vel - beta
            xc, yc = self.apply_csp_transform(xr, yr)
            x_data.append(xc)
            y_data.append(yc)

        self._csp_x_data = x_data
        self._csp_y_data = y_data

    def _draw_plot(self):
        """Draw CSP portrait."""
        self._compute_plot_portrait()

        if not self._csp_x_data or self._csp_x_now is None:
            return

        self.portrait_line.set_data(self._csp_x_data, self._csp_y_data)
        self.current_point.set_data([self._csp_x_now], [self._csp_y_now])
        self.hs_scatter.set_data(self.hs_x, self.hs_y)

        max_abs = max(
            max(abs(x) for x in self._csp_x_data),
            max(abs(y) for y in self._csp_y_data),
            1e-3,
        )

        ang = self._csp_current_angle if self._csp_current_angle is not None else 0.0
        cx = math.cos(ang) * max_abs
        cy = math.sin(ang) * max_abs
        self.center_to_current_line.set_data([0, cx], [0, cy])

        if self.last_hs_angle is not None:
            hx = math.cos(self.last_hs_angle) * max_abs
            hy = math.sin(self.last_hs_angle) * max_abs
            self.center_to_hs_line.set_data([0, hx], [0, hy])
        else:
            self.center_to_hs_line.set_data([], [])

        margin = 0.2 * max_abs
        self.ax.set_xlim(-max_abs - margin, max_abs + margin)
        self.ax.set_ylim(-max_abs - margin, max_abs + margin)

        if self._calibrating:
            elapsed = 0.0
            if self._calib_start_time and self.latest_time:
                elapsed = self.latest_time - self._calib_start_time
            remain = max(0.0, self.csp_calibration_time - elapsed)
            self.ax.set_title(
                f"CALIBRATING | {remain:.1f}s left | "
                f"Phase = {self.current_phase * 100.0:.1f}% (torque OFF)"
            )
        else:
            frozen_str = "FROZEN" if self._csp_params_frozen else "LIVE"
            self.ax.set_title(
                f"GO [{frozen_str}] | Phase = {self.current_phase * 100.0:.1f}% | Torque ON"
            )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def plot_timer_cb(self):
        """Low-rate visualization callback."""
        if not self.enable_plot:
            return
        if not self.init_done:
            return

        if not self.is_walking:
            mode = "steady waveform" if self.use_steady_go_detection else "velocity RMS"
            self.ax.set_title(f"STOP | Waiting for GO by {mode} | HS OFF")
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            return

        self._draw_plot()

    # ------------------------------------------------------------------
    # Main publish timer
    # ------------------------------------------------------------------

    def publish_phase(self):
        if not self.init_done:
            return
        if self.latest_angle is None or self.latest_vel is None:
            return

        if not self.is_walking:
            self.hs_detected_flag = False
            self.current_phase = 0.0
            self.prev_phase_mono = 0.0
        else:
            self._compute_phase()

        # --- publish ---
        phase_msg = Float32()
        phase_msg.data = float(self.current_phase * 100.0)
        self.phase_pub.publish(phase_msg)

        hs_msg = Bool()
        hs_msg.data = bool(self.hs_detected_flag)
        self.hs_pub.publish(hs_msg)

        # -------------------------------------------------------
        # is_walking 토픽에는 _torque_enabled를 publish
        # (캘리브레이션 중에는 False → 토크 프로파일 노드 비활성)
        # -------------------------------------------------------
        walking_msg = Bool()
        walking_msg.data = bool(self._torque_enabled)
        self.walking_pub.publish(walking_msg)

        # --- CSV ---
        if self.enable_csv_logging and self.csv_writer is not None:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            self.csv_writer.writerow([
                now_sec,
                float(self.current_phase * 100.0),
                int(self.hs_detected_flag),
                int(self._torque_enabled),
                float(self.latest_angle) if self.latest_angle is not None else 0.0,
                float(self.latest_vel)   if self.latest_vel   is not None else 0.0,
            ])
            self.csv_file.flush()

        if self.is_walking:
            self.hs_detected_flag = False

    # ------------------------------------------------------------------

    def destroy_node(self):
        if self.enable_csv_logging and self.csv_file is not None:
            try:
                self.csv_file.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GaitPhaseEstimationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            plt.close("all")
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()