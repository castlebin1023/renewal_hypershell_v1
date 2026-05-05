from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # =========================
        # LEFT DRIVER
        # =========================
        Node(
            package="exo_control",   # ← 네 패키지 이름
            executable="motor_driver_node",
            name="left_driver",
            parameters=[{
                "controller_id": 0x0,
                "channel": "can0",
                "upload_id": 0x00002900,
                "gear_ratio": 9.0,
                "pole_pairs": 21.0,
                "joint_name": "left_hip_joint",
                "rx_period": 0.002,
                "publish_rate": 500.0,
                "cmd_timeout": 0.2,
                "current_limit": 30.0,
                "init_countdown": 3,
                "effort_lpf_alpha": 0.1,

                "state_topic": "/left_motor/state",
                "cmd_topic": "/left_motor/cmd_current",
                "init_topic": "/left_motor/init_done"
            }]
        ),

        # =========================
        # RIGHT DRIVER
        # =========================
        Node(
            package="exo_control",
            executable="motor_driver_node",
            name="right_driver",
            parameters=[{
                "controller_id": 0x68,
                "channel": "can1",
                "upload_id": 0x00002968,
                "gear_ratio": 9.0,
                "pole_pairs": 21.0,
                "joint_name": "right_hip_joint",
                "rx_period": 0.002,
                "publish_rate": 100.0,
                "cmd_timeout": 0.2,
                "current_limit": 30.0,
                "init_countdown": 3,
                "effort_lpf_alpha": 0.1,

                "state_topic": "/right_motor/state",
                "cmd_topic": "/right_motor/cmd_current",
                "init_topic": "/right_motor/init_done"
            }]
        ),

        # =========================
        # LEFT ADMITTANCE
        # =========================
        Node(
            package="exo_control",
            executable="motor_control_node",
            name="left_controler",
            parameters=[{
                "state_topic": "/left_motor/state",
                "cmd_topic": "/left_motor/cmd_current",
                "init_topic": "/left_motor/init_done"
            }]
        ),

        # =========================
        # RIGHT ADMITTANCE
        # =========================
        Node(
            package="exo_control",
            executable="motor_control_node",
            name="right_controler",
            parameters=[{
                "state_topic": "/right_motor/state",
                "cmd_topic": "/right_motor/cmd_current",
                "init_topic": "/right_motor/init_done"
            }]
        ),

    ])