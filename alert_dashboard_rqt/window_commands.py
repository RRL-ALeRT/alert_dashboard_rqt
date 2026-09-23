#!/usr/bin/env python3
"""
Shared window command definitions for RQT dashboard plugins.
This file contains all tmux window names and their corresponding ROS2 launch commands.
"""

import os

# Environment variables
GEN3_IP = os.getenv("GEN3_IP")

# All window commands - shared between alert_dashboard_rqt and spot_estop_rqt
ALL_WINDOW_COMMANDS = {
    # Robot
    "discovery": "fastdds discovery --server-id 0",
    "estop": "ros2 run spot_driver_plus spot_estop.py",
    "spot_driver": "ros2 launch spot_driver_plus spot_launch.py",
    "kinova_python": "ros2 launch kortex_ros2_python manipulator_launch.py",
    # "kinova_js": "ros2 run kortex_controller_py joint_states_publisher",
    # "kinova_driver": f"ros2 launch kortex_bringup gen3.launch.py robot_ip:={GEN3_IP} dof:=6 launch_rviz:=false",
    "kinova_moveit": "xvfb-run -a ros2 launch gen3_85_moveit_config move_group.launch.py",
    "kinova_vision": "ros2 launch kinova_vision kinova_vision.launch.py",
    "realsenses": "ros2 launch rrl_launchers realsenses_launch.py",
    "ifm": "ros2 launch o3p_node multi_camera.launch.py",
    "livox_driver": "ros2 launch livox_ros_driver2 msg_MID360_launch.py",

    # Mobility
    "navigation": "ros2 launch bring_up_alert_nav alert_nav_launch.py",
    "octo_livox": "ros2 launch octomap_server octomap_livox_launch.py",
    "octo_spot": "ros2 launch octomap_server octomap_spot_launch.py",
    "octo_ifm": "ros2 launch octomap_server octomap_ifm_launch.py",
    "octo_exp": "ros2 launch octomap_server octomap_exp_launch.py",
    "octo_realsense": "ros2 launch octomap_server octomap_realsense_launch.py",
    # "frame_runner": "ros2 launch gpp_action_examples frame_runner_launch.py",
    #"nav_2d": "ros2 launch bring_up_alert_nav alert_2d_nav_launch.py",
    "explore": "ros2 launch octomap_server octomap_realsense_color_launch.py",
    #"octo_dex": "ros2 launch octomap_server octomap_dex_launch.py",

    # Dexterity
    "audio": "ros2 run audio_capture audio_capture_node --ros-args -p format:=wave -r __ns:=/nuc",
    #"audio_play": "ros2 run audio_play audio_play_node --ros-args -p format:=wave -r __ns:=/operator",
    "thermal_cam": "ros2 launch seek_thermal_ros thermal_publisher_launch.py",
    "hazmat_detection": "ros2 launch rrl_launchers victim_crate_launch.py",
    "motion_detection": "ros2 run spot_driver_plus motion_detection.py",
    # "hazmat_detection": "ros2 run spot_driver_plus rrl_qr eader_kinova.py",
    "auto_dex": "ros2 run auto_dex_nodes visual_servo",
    "dex_board": "ros2 launch rrl_launchers board_mapping_launch.py",
    "press_estop": "ros2 run auto_dex_nodes press_estop",

    # Additional
    "blocksworld_scan": "ros2 run world_info aruco_node",
    "blocksworld_gpp_wrapper": "ros2 run webots_spot gpp_blocksworld_server",
    "blocksworld_gpp_agent": "ros2 launch webots_spot blocksworld_launch.py",
    "save_octomap":"ros2 run octomap_server octomap_saver_node --ros-args -p octomap_path:=/home/max1/maze/maze.bt",

    # Exploration
    "exp_frontier": "ros2 run rrt_exploration frontier_opencv_detector.py",
    "exp_detection": "ros2 launch rrl_launchers exp_mapping_launch.py",
    "exp_save_map": "ros2 run hector_geotiff geotiff_saver",
    #"exp_frontier": "ros2 run alert_exploration mbf_exploration",
    "3d_exp": "ros2 run alert_exploration 3d_frontier_exploration",
    # Navigation
    "nav_3d_to_2d": "ros2 run spot_driver_plus plan_3d_path.py"
}
