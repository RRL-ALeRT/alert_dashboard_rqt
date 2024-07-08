#!/usr/bin/env python3

import os
import sys

import rclpy
from ament_index_python import get_package_share_directory

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import (
    QWidget,
    QPushButton,
    QLabel,
    QToolButton,
)

from rqt_gui.main import Main

from alert_dashboard_rqt.tmux_utils import TmuxSSHWrapper, TmuxLocalWrapper

MAX_USER = os.getenv("MAX_USER")
MAX_IP = os.getenv("MAX_IP")
SESSION_NAME = "spot_session"
GEN3_IP = os.getenv("GEN3_IP")

ALL_WINDOW_COMMANDS = {
    # Robot
    "spot_driver": "ros2 launch spot_driver_plus spot_launch.py",
    "kinova_driver": f"ros2 launch kortex_bringup gen3.launch.py robot_ip:={GEN3_IP} dof:=6 launch_rviz:=false",
    "kinova_moveit": "ros2 launch spot_gen3_moveit move_group.launch.py use_rviz:=false",
    "kinova_vision": "ros2 launch kinova_vision rgbd_launch.py",
    "realsenses": "ros2 launch rrl_launchers realsenses_launch.py",
    # Mobility
    "octomap": "ros2 launch octomap_server octomap_velodyne_launch.py",
    "frame_runner": "ros2 launch gpp_action_examples frame_runner_launch.py",
    # Dexterity
    "audio_capture": "ros2 run audio_capture audio_capture_node --ros-args -p format:=wave -r __ns:=/nuc",
    "audio_play": "ros2 run audio_play audio_play_node --ros-args -p format:=wave -r __ns:=/operator",
    "thermal_cam": "ros2 launch seek_thermal_ros thermal_publisher_launch.py",
    "hazmat_detection": "ros2 run world_info object_detection_yolov5 hazmat /kinova_color",
    "motion_detection": "ros2 run spot_driver_plus motion_detection.py",
    # Additional
    "blocksworld_scan": "ros2 run world_info aruco_node",
    "blocksworld_gpp_wrapper": "ros2 run webots_spot gpp_blocksworld_server",
    "blocksworld_gpp_agent": "ros2 launch webots_spot blocksworld_launch.py",
}

if MAX_USER is None:
    print("MAX_USER environment variable is not set.")
    exit()
if MAX_IP is None:
    print("MAX_IP environment variable is not set.")
    exit()


class DashboardRqtPlugin(Plugin):
    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self.node = context.node

        ui_path = (
            get_package_share_directory("alert_dashboard_rqt")
            + "/resource/mainwindow.ui"
        )

        # Create a QWidget instance
        self._widget = QWidget()

        # Load the .ui file
        ui_file = ui_path
        loadUi(ui_file, self._widget)

        # Add the widget to the user interface
        context.add_widget(self._widget)

        # Dictionary of all buttons, labels. Connect them with respective callbacks as well
        self.bnl = {}
        for button in ALL_WINDOW_COMMANDS.keys():
            self.bnl[f"{button}_push"] = self._widget.findChild(
                QPushButton, f"{button}_push"
            )
            self.bnl[f"{button}_push"].toggled.connect(
                lambda checked, btn=button: self.button_push_cb(btn, checked)
            )

            self.bnl[f"{button}_tool"] = self._widget.findChild(
                QToolButton, f"{button}_tool"
            )
            self.bnl[f"{button}_tool"].clicked.connect(
                lambda checked, btn=button: self.toolbutton_cb(btn, checked)
            )

            self.bnl[f"{button}_label"] = self._widget.findChild(
                QLabel, f"{button}_label"
            )

        self.expected_active_windows = []
        self.node.create_timer(2, self.check_expected_windows)

        self.tmux = TmuxSSHWrapper(MAX_USER, MAX_IP, SESSION_NAME)
        # self.tmux = TmuxLocalWrapper(SESSION_NAME)
        self.tmux.new_session()

    def check_expected_windows(self):
        active_windows = self.tmux.get_active_windows()

        # Red radio button if any of the windows closed unexpectedly
        for expected_window in self.expected_active_windows:
            if expected_window in active_windows:
                self.bnl[f"{expected_window}_label"].setText("Running")
            else:
                self.bnl[f"{expected_window}_label"].setText("Stopped")

        # Auto press buttons if the program windows are already running
        for active_window in active_windows:
            if active_window not in ALL_WINDOW_COMMANDS.keys():
                continue

            if active_window not in self.expected_active_windows:
                self.expected_active_windows.append(active_window)
                self.bnl[f"{active_window}_push"].setChecked(True)

    def button_push_cb(self, button, checked):
        window_name = button

        if checked:
            # Only start the window and run the command if it's not already active
            if window_name not in self.expected_active_windows:
                self.tmux.temporary_window(
                    window_name,
                    ALL_WINDOW_COMMANDS[window_name],
                )
                self.expected_active_windows.append(window_name)
        else:
            # Kill window and remove the window_name from expected_active_windows
            self.tmux.kill_window(window_name)
            self.expected_active_windows.remove(window_name)
            self.bnl[f"{window_name}_label"].setText("")

    def toolbutton_cb(self, button, checked):
        window_name = button
        print(window_name)
        self.tmux.window_select(window_name)

    def shutdown_plugin(self):
        self.node.destroy_node()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


def main():
    """Run the plugin."""
    # Initialize the ROS node
    rclpy.init()

    # Create the plugin and run it
    sys.exit(
        Main().main(sys.argv, standalone="alert_dashboard_rqt.alert_dashboard_rqt")
    )


if __name__ == "__main__":
    main()
