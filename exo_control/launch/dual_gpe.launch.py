from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # =========================
        # LEFT DRIVER
        # =========================
        Node(
            package="exo_control",
            executable="gpe_driver_node",
            name="left_driver",
            parameters=[{
                "channel":          "can1",
                "controller_id":    0x68,
                "upload_id":        0x00002968,

                "gear_ratio":       9.0,
                "pole_pairs":       21.0,
                "joint_name":       "left_hip_joint",

                "rx_period":        0.002,
                "publish_rate":     500.0,

                "cmd_timeout":      0.2,
                "current_limit":    16.0,
                "init_countdown":   3,

                "enable_csv_logging": False,

                "state_topic":      "/left_motor/state",
                "cmd_topic":        "/left_motor/cmd_current",
                "init_topic":       "/left_motor/init_done",
                "acc_topic":        "/left_motor/acc",

            }],
        ),

        # =========================
        # RIGHT DRIVER
        # =========================
        Node(
            package="exo_control",
            executable="gpe_driver_node",
            name="right_driver",
            parameters=[{
                "channel":          "can0",
                "controller_id":    0x00,
                "upload_id":        0x00002900,

                "gear_ratio":       9.0,
                "pole_pairs":       21.0,
                "joint_name":       "right_hip_joint",

                "rx_period":        0.002,
                "publish_rate":     500.0,

                "cmd_timeout":      0.2,
                "current_limit":    30.0,
                "init_countdown":   3,

                "enable_csv_logging": False,

                "state_topic":      "/right_motor/state",
                "cmd_topic":        "/right_motor/cmd_current",
                "init_topic":       "/right_motor/init_done",
                "acc_topic":        "/right_motor/acc",

            }],
        ),

        # =========================
        # LEFT GPE CONTROL
        # =========================
        Node(
            package="exo_control",
            executable="gpe_control_node",
            name="left_gpe",
            parameters=[{
                "state_topic":          "/left_motor/state",
                "init_topic":           "/left_motor/init_done",

                "phase_topic":          "/left_gait/phase",
                "heel_strike_topic":    "/left_gait/heel_strike",
                "walking_state_topic":  "/left_gait/is_walking",

                "publish_rate":         500.0,
                "window_size":          500,
                "enable_plot":          True,

                "use_steady_go_detection":      True,
                "go_vel_rms_threshold":         0.18,
                "stop_vel_rms_threshold":       0.06,
                "go_cooldown_time":             1.0,

                "csp_calibration_time":         5.0,
                "csp_k":                        1.5,

                "peak_velocity_threshold":      0.01,
                "hs_angle_ratio":               0.50,
                "hs_refractory_time":           0.45,

                "steady_window_size":           1500,
                "min_angle_range_for_walking":  0.06,
                "steady_min_peaks":             3,
                "steady_stride_cv_threshold":   0.25,
                "steady_peak_ratio":            0.75,

                "walking_window_size":          500,
                "go_count_threshold":           5,
                "stop_count_threshold":         2500,

                "enable_csv_logging":           True,
            }],
        ),

        # =========================
        # RIGHT GPE CONTROL
        # =========================
        Node(
            package="exo_control",
            executable="gpe_control_node",
            name="right_gpe",
            parameters=[{
                "state_topic":          "/right_motor/state",
                "init_topic":           "/right_motor/init_done",

                "phase_topic":          "/right_gait/phase",
                "heel_strike_topic":    "/right_gait/heel_strike",
                "walking_state_topic":  "/right_gait/is_walking",

                "publish_rate":         500.0,
                "window_size":          500,
                "enable_plot":          True,

                "use_steady_go_detection":      True,
                "go_vel_rms_threshold":         0.18,
                "stop_vel_rms_threshold":       0.03,
                "go_cooldown_time":             1.5,

                "csp_calibration_time":         5.0,
                "csp_k":                        1.5,

                "peak_velocity_threshold":      0.01,
                "hs_angle_ratio":               0.50,
                "hs_refractory_time":           0.45,

                "steady_window_size":           1500,
                "min_angle_range_for_walking":  0.2,
                "steady_min_peaks":             3,
                "steady_stride_cv_threshold":   0.15,
                "steady_peak_ratio":            0.75,

                "walking_window_size":          500,
                "go_count_threshold":           5,
                "stop_count_threshold":         2500,

                "enable_csv_logging":           True,
            }],
        ),

        # =========================
        # TORQUE PROFILE NODE
        # =========================
        Node(
            package="exo_control",
            executable="gpe_torque_profile_node",
            name="gpe_torque_profile",
            parameters=[{
                "left_phase_topic":       "/left_gait/phase",
                "right_phase_topic":      "/right_gait/phase",

                "left_walking_topic":     "/left_gait/is_walking",
                "right_walking_topic":    "/right_gait/is_walking",

                "left_init_topic":        "/left_motor/init_done",
                "right_init_topic":       "/right_motor/init_done",

                "left_cmd_topic":         "/left_motor/cmd_current",
                "right_cmd_topic":        "/right_motor/cmd_current",

                "publish_rate":           500.0,

                # 6D smooth trapezoid profile
                "extension_peak_torque":  0.0,
                "extension_peak_time":    10.9,
                "extension_rise_time":    15.7,

                "flexion_peak_torque":    3.0,
                "flexion_peak_time":      60.0,
                "flexion_rise_time":      15.0,

                # Motor constants
                "left_motor_kt":          0.095,
                "right_motor_kt":         0.105,
                "gear_ratio":             9.0,
                "transmission_efficiency": 0.9,

                # Direction / safety
                "left_direction":         -1.0,
                "right_direction":        -1.0,
                "current_limit":          10.0,

                "current_slew_rate":      28.5,
                "enable_slew_rate":       False,

                "enable_assistance":      True,
                "zero_when_not_walking":  True,
                "enable_csv_logging":     True,

                "extension_sign":         -1.0,
                "flexion_sign":           1.0,
            }],
        ),

    ])