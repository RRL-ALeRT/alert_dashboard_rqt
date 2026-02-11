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

    "kinova_python": "ros2 launch kortex_controller_py manipulator_launch.py",
    "kinova_driver": f"ros2 launch kortex_bringup gen3.launch.py robot_ip:={GEN3_IP} dof:=6 launch_rviz:=false",
    "kinova_moveit": "ros2 launch spot_gen3_moveit move_group.launch.py use_rviz:=false",
    "kinova_vision": "ros2 launch kinova_vision rgbd_launch.py",
    "realsenses": "ros2 launch rrl_launchers realsenses_launch.py",
    # Mobility
    "octo_realsenses": "ros2 launch octomap_server octomap_realsenses_launch.py",
    "octo_spot": "ros2 launch octomap_server octomap_spot_launch.py",
    "frame_runner": "ros2 launch gpp_action_examples frame_runner_launch.py",
    # Dexterity
    "audio_capture": "ros2 run audio_capture audio_capture_node --ros-args -p format:=wave -r __ns:=/nuc",
    "audio_play": "ros2 run audio_play audio_play_node --ros-args -p format:=wave -r __ns:=/operator",
    "thermal_cam": "ros2 launch seek_thermal_ros thermal_publisher_launch.py",
    "hazmat_detection": "ros2 run spot_driver_plus kinova_yolov8_openvino.py",
    # Additional
    "blocksworld_scan": "ros2 run world_info aruco_node",
    "blocksworld_gpp_wrapper": "ros2 run webots_spot gpp_blocksworld_server",
    "blocksworld_gpp_agent": "ros2 launch webots_spot blocksworld_launch.py",
    # Exploration
    "exp_frontier": "ros2 run rrt_exploration frontier_opencv_detector.py",
    "exp_detection": "ros2 launch rrl_launchers exp_mapping_launch.py",
    "exp_save_map": "ros2 run hector_geotiff geotiff_saver",
    # Navigation
    "nav_3d_to_2d": "ros2 run spot_driver_plus plan_3d_path.py"
}
