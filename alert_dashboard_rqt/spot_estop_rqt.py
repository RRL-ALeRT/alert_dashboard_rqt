#!/usr/bin/env python3

import sys
import os
import subprocess
from functools import partial

import rclpy
from rclpy.qos import QoSProfile

from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, SetBool
from spot_msgs.msg import BatteryStateArray

from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtWidgets import (
    QWidget,
    QTabWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLabel,
    QScrollBar,
    QSlider,
)

from rqt_gui.main import Main

from alert_dashboard_rqt.moveit_action_client import MoveGroupActionClient
from alert_dashboard_rqt.tmux_utils import TmuxSSHWrapper, TmuxLocalWrapper

MAX_USER = os.getenv("MAX_USER")
MAX_IP = os.getenv("MAX_IP")
SESSION_NAME = "spot_session"
GEN3_IP = os.getenv("GEN3_IP")

ALL_WINDOW_COMMANDS = {
    # Robot
    "estop": "ros2 run spot_driver_plus spot_estop.py",
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

if MAX_IP is None:
    print("MAX_IP environment variable is not set.")
    exit()
if MAX_USER is None:
    print("MAX_USER environment variable is not set.")
    exit()


class EstopRqtPlugin(Plugin):
    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self.node = context.node

        # Create the publisher for controlling lights
        self.lights_pub = self.node.create_publisher(
            String, "lights_camera", QoSProfile(depth=1)
        )

        # Create the subscriber for status updates
        self.node.create_subscription(
            BatteryStateArray, "/status/battery_states", self.status_callback, 1
        )

        # Create service clients
        self.map_frame_reset_client = self.node.create_client(
            Trigger, "/reset_map_frame"
        )
        self.path_reset_client = self.node.create_client(Empty, "/reset_travelled_path")
        self.world_info_reset_client = self.node.create_client(
            Empty, "/reset_world_info"
        )
        self.octomap_reset_client = self.node.create_client(
            Empty, "/octomap_server/reset"
        )
        self.octomap_nav_reset_client = self.node.create_client(
            Empty, "/navigation/octomap_server/reset"
        )

        self.nav_3d_follow_path_client = self.node.create_client(
            SetBool, "/navigation/follow_path"
        )

        # Create the main widget and set up the layout
        self.widget = QWidget()
        layout = QVBoxLayout()
        self.widget.setLayout(layout)

        # Set minimum size for the widget
        self.widget.setMinimumSize(200, 200)

        ## TABS
        # Create the tab widget
        tab_widget = QTabWidget()

        # Create the buttons and layouts for each tab
        first_tab_layout = QVBoxLayout()
        second_tab_layout = QVBoxLayout()
        third_tab_layout = QVBoxLayout()

        ### FIRST TAB
        ## BUTTONS

        # Create a horizontal slider for Stop
        self.stop_slider = QSlider(Qt.Horizontal)
        self.stop_slider.setRange(0, 100)

        # Set the background color of the groove to red and add rounded corners
        self.stop_slider.setStyleSheet(
            """
            QSlider {
                min-height: 120px;
                max-height: 120px;
            }
            
            QSlider::groove:horizontal {
                background: red;
                height: 15px;
                border-radius: 5px;
                margin: 20px;
            }
            
            QSlider::handle:horizontal {
                background: white;
                width: 50px;
                border-radius: 25px;
                border: 1px solid #777;
                margin: -20px 0;
                subcontrol-origin: margin;
                subcontrol-position: center center;
            }
        """
        )

        # Connect the slider's sliderReleased signal to a slot (function)
        self.stop_slider.sliderReleased.connect(self.stop_slider_released)
        self.spot_driver_on = False
        self.initial_spot_driver_check = False

        # Add a QLabel for the text on the handle
        stop_label = QLabel("Estop")
        stop_label.setAlignment(Qt.AlignCenter)
        stop_label.setStyleSheet(
            "font: bold 20px; border-width: 5px; border-radius:20px; padding: 30px;"
        )

        drivers_button = QPushButton("Start")
        drivers_button.clicked.connect(self.run_ssh_command)
        drivers_button.setStyleSheet(
            "background-color: blue; font: bold 20px; border-width: 5px; border-radius:20px; padding: 20px"
        )

        # Create the label for displaying status
        self.status_label = QLabel("Remaining: Unknown")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font: bold 16px")

        first_row_buttons = QHBoxLayout()
        first_row_buttons.addWidget(stop_label)
        first_row_buttons.addWidget(self.stop_slider)

        second_row_buttons = QHBoxLayout()
        second_row_buttons.addWidget(drivers_button)

        first_tab_layout.addLayout(first_row_buttons)
        first_tab_layout.addLayout(second_row_buttons)
        first_tab_layout.addWidget(self.status_label)

        ### SECOND TAB

        hazmat_button = QPushButton("Hazmat")
        hazmat_button.clicked.connect(self.hazmat_command)
        hazmat_button.setStyleSheet(
            "background-color: pink; font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px"
        )

        world_reset_button = QPushButton("World Reset")
        world_reset_button.clicked.connect(self.world_reset_command)
        world_reset_button.setStyleSheet(
            "background-color: pink; font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px"
        )

        # Create a toggle button for audio communication
        two_way_audio = QPushButton("2WayAudio")
        two_way_audio.setCheckable(True)
        two_way_audio.setStyleSheet(
            "QPushButton { font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px }"
            "QPushButton:checked { background-color: green }"
            "QPushButton:unchecked { background-color: transparent }"
        )
        two_way_audio.clicked.connect(partial(self.toggle_two_way_audio, two_way_audio))

        ## SCROLLBARS
        # Create scroll bars for lights
        front_light = QScrollBar(Qt.Horizontal)
        left_light = QScrollBar(Qt.Horizontal)
        right_light = QScrollBar(Qt.Horizontal)

        ## Create a toggle button for running/stopping gologpp frame runner
        self.toggle_button = QPushButton("Frame Runner")
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(False)  # Set the initial state
        self.toggle_button.toggled.connect(self.toggle_frame_runner)
        self.toggle_button.setStyleSheet(
            "background-color: transparent; font: bold 15px; border-width: 5px; border-radius: 15px; padding: 15px"
        )
        second_tab_layout.addWidget(self.toggle_button)

        # Connect callbacks to scroll bars
        front_light.valueChanged.connect(self.front_light_value_change_callback)
        left_light.valueChanged.connect(self.left_light_value_change_callback)
        right_light.valueChanged.connect(self.right_light_value_change_callback)

        front_light.sliderReleased.connect(self.front_light_callback)
        left_light.sliderReleased.connect(self.left_light_callback)
        right_light.sliderReleased.connect(self.right_light_callback)

        # Add scroll areas to the layout
        second_tab_layout.addWidget(front_light)
        second_tab_layout.addWidget(left_light)
        second_tab_layout.addWidget(right_light)

        row_buttons = QHBoxLayout()
        row_buttons.addWidget(hazmat_button)
        row_buttons.addWidget(world_reset_button)
        row_buttons.addWidget(two_way_audio)

        second_tab_layout.addLayout(row_buttons)

        ### THIRD TAB (BLOCKSWORLD)
        # Create a toggle button to scan block positions
        self.toggle_blocks_scan = QPushButton("Scan Blocks")
        self.toggle_blocks_scan.setCheckable(True)
        self.toggle_blocks_scan.setChecked(False)  # Set the initial state
        self.toggle_blocks_scan.toggled.connect(self.blocks_scan)
        self.toggle_blocks_scan.setStyleSheet(
            "background-color: transparent; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
        )
        third_tab_layout.addWidget(self.toggle_blocks_scan)
        self.blocks_scan_checked = False

        self.node.create_subscription(String, "/block_loc", self.block_loc_update, 1)

        # Create a label for displaying status
        self.bw_status_label = QLabel("Unknown")
        self.bw_status_label.setAlignment(Qt.AlignCenter)
        self.bw_status_label.setStyleSheet("font: bold 12px")
        third_tab_layout.addWidget(self.bw_status_label)

        # Create a button to get manipulator above blocks
        self.moveit = MoveGroupActionClient(self.node)
        blocks_position_button = QPushButton("Position the manipulator")
        blocks_position_button.clicked.connect(self.blocks_manipulator_position)
        blocks_position_button.setStyleSheet(
            "background-color: green; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
        )
        third_tab_layout.addWidget(blocks_position_button)

        # Create a toggle button to solve blocksworld
        self.toggle_blocks_execute = QPushButton("Execute agent")
        self.toggle_blocks_execute.setCheckable(True)
        self.toggle_blocks_execute.setChecked(False)  # Set the initial state
        self.toggle_blocks_execute.toggled.connect(self.blocks_execute)
        self.toggle_blocks_execute.setStyleSheet(
            "background-color: transparent; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
        )
        third_tab_layout.addWidget(self.toggle_blocks_execute)

        ### FOURTH TAB (Exploration)
        # Create layouts for Exploration tab
        exploration_layout = QVBoxLayout()

        ## Widgets for Exploration tab
        exploration_slider = QSlider(Qt.Horizontal)
        exploration_slider.setRange(1, 10)
        exploration_slider.setStyleSheet(
            """
            QSlider::groove:horizontal {
                background: lightgray;
                height: 15px;
                border-radius: 5px;
                margin: 20px;
            }
            QSlider::handle:horizontal {
                background: white;
                border-radius: 25px;
                border: 1px solid #777;
                width: 50px;
                margin: -5px 0;
            }
            """
        )

        self.lap_number = 1
        self.exploration_slider_label = QLabel(f"Lap: {self.lap_number}")
        exploration_slider.valueChanged.connect(self.lap_value_update)
        self.exploration_slider_label.setAlignment(Qt.AlignCenter)

        exploration_toggle = QPushButton("Start/Stop")
        exploration_toggle.setCheckable(True)
        exploration_toggle.setChecked(False)
        exploration_toggle.setStyleSheet(
            """
            QPushButton {
                font: bold 12px;
                border: 1px solid #777;
                background-color: lightgray;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:checked {
                background-color: lightblue;
            }
            """
        )
        exploration_toggle.toggled.connect(self.toggle_exploration)

        exploration_reset_button = QPushButton("Reset Map")
        exploration_reset_button.setStyleSheet(
            """
            QPushButton {
                font: bold 12px;
                border: 1px solid #777;
                background-color: lightcoral;
                padding: 10px;
                border-radius: 5px;
            }
            """
        )
        exploration_reset_button.clicked.connect(self.world_reset_command)

        exploration_save_button = QPushButton("Save Maps")
        exploration_save_button.setStyleSheet(
            """
            QPushButton {
                font: bold 12px;
                border: 1px solid #777;
                background-color: lightseagreen;
                padding: 10px;
                border-radius: 5px;
            }
            """
        )
        exploration_save_button.clicked.connect(self.save_exp_maps)

        # Add widgets to layout
        exploration_layout.addWidget(exploration_slider)
        exploration_layout.addWidget(self.exploration_slider_label)
        exploration_layout.addWidget(exploration_toggle)
        exploration_layout.addWidget(exploration_reset_button)
        exploration_layout.addWidget(exploration_save_button)

        # Create widget and set layout
        exploration_tab_widget = QWidget()
        exploration_tab_widget.setLayout(exploration_layout)

        ### FIFTH TAB (Navigation)
        navigation_layout = QVBoxLayout()

        navigation_toggle = QPushButton("Start/Stop")
        navigation_toggle.setCheckable(True)
        navigation_toggle.setChecked(False)
        navigation_toggle.setStyleSheet(
            """
            QPushButton {
                font: bold 12px;
                border: 1px solid #777;
                background-color: lightgray;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:checked {
                background-color: lightblue;
            }
            """
        )
        navigation_toggle.toggled.connect(self.toggle_navigation)

        navigation_follow_path_button = QPushButton("Follow Path")
        navigation_follow_path_button.setStyleSheet(
            """
            QPushButton {
                font: bold 12px;
                border: 1px solid #777;
                background-color: lightcoral;
                padding: 10px;
                border-radius: 5px;
            }
            """
        )
        navigation_follow_path_button.clicked.connect(self.follow_path_command)

        # Add widgets to layout
        navigation_layout.addWidget(navigation_toggle)
        navigation_layout.addWidget(navigation_follow_path_button)

        # Create widget and set layout
        navigation_tab_widget = QWidget()
        navigation_tab_widget.setLayout(navigation_layout)

        ## FINAL BOX
        # Create tab widgets
        first_tab_widget = QWidget()
        second_tab_widget = QWidget()
        third_tab_widget = QWidget()

        # Set layouts for tab widgets
        first_tab_widget.setLayout(first_tab_layout)
        second_tab_widget.setLayout(second_tab_layout)
        third_tab_widget.setLayout(third_tab_layout)

        # Add tab widgets to the tab widget
        tab_widget.addTab(first_tab_widget, "BASIC")
        tab_widget.addTab(second_tab_widget, "PLUS")
        tab_widget.addTab(third_tab_widget, "BW")
        tab_widget.addTab(exploration_tab_widget, "EXP")
        tab_widget.addTab(navigation_tab_widget, "NAV")

        # Apply style to the tab widget
        tab_widget.setStyleSheet(
            """
            QTabWidget::pane {
                border: none;
            }

            QTabWidget::tab-bar {
                alignment: left;
            }
            """
        )

        layout.addWidget(tab_widget)
        layout.addWidget(self.status_label)

        # Add the widget to the plugin
        self._widget = self.widget
        context.add_widget(self._widget)

        self.front_value = 0
        self.left_value = 0
        self.right_value = 0

        self.tmux = TmuxSSHWrapper(MAX_USER, MAX_IP, SESSION_NAME)
        # self.tmux = TmuxLocalWrapper(SESSION_NAME)
        self.tmux.new_session()

    def toggle_frame_runner(self, checked):
        window_name = "frame_runner"
        if checked:  # Button is checked, start the command
            self.tmux.temporary_window(window_name, ALL_WINDOW_COMMANDS[window_name])
            self.toggle_button.setStyleSheet(
                "background-color: green; font: bold 15px; border-width: 5px; border-radius: 15px; padding: 15px"
            )
        else:  # Button is unchecked, stop the command
            self.toggle_button.setStyleSheet(
                "background-color: transparent; font: bold 15px; border-width: 5px; border-radius: 15px; padding: 15px"
            )
            self.tmux.kill_window(window_name)

    def toggle_two_way_audio(self, button):
        if button.isChecked():
            # Operator side
            ssh_command = "screen -S deck -X screen -t capture ros2 run audio_capture audio_capture_node --ros-args -p format:=wave -r __ns:=/operator"  # Sent audio
            subprocess.Popen(ssh_command, shell=True)
            ssh_command = "screen -S deck -X screen -t play ros2 run audio_play audio_play_node --ros-args -p format:=wave -r __ns:=/nuc"  # Received audio
            subprocess.Popen(ssh_command, shell=True)

            # Nuc side
            self.tmux.temporary_window(
                "audio_capture", ALL_WINDOW_COMMANDS["audio_capture"]
            )
            self.tmux.temporary_window("audio_play", ALL_WINDOW_COMMANDS["audio_play"])
        else:
            # Operator
            ssh_command = "screen -S deck -X -p capture kill"
            subprocess.Popen(ssh_command, shell=True)
            ssh_command = "screen -S deck -X -p play kill"
            subprocess.Popen(ssh_command, shell=True)

            # NUC
            self.tmux.kill_window("audio_capture")
            self.tmux.kill_window("audio_play")

    def front_light_value_change_callback(self, value):
        self.front_value = int((value + 1) * 255 / 100)
        if self.front_value < 4:
            self.front_value = 0

    def left_light_value_change_callback(self, value):
        self.left_value = int((value + 1) * 255 / 100)
        if self.left_value < 4:
            self.left_value = 0

    def right_light_value_change_callback(self, value):
        self.right_value = int((value + 1) * 255 / 100)
        if self.right_value < 4:
            self.right_value = 0

    def front_light_callback(self):
        msg = String()
        msg.data = f"camera_light:{self.front_value}"
        self.lights_pub.publish(msg)

    def left_light_callback(self):
        msg = String()
        msg.data = f"left_light:{self.left_value}"
        self.lights_pub.publish(msg)

    def right_light_callback(self):
        msg = String()
        msg.data = f"right_light:{self.right_value}"
        self.lights_pub.publish(msg)

    def stop_slider_released(self):
        window_name = "estop"
        if self.stop_slider.value() > 50 and not self.spot_driver_on:
            self.spot_driver_on = True
            self.stop_slider.setSliderPosition(100)  # Push slider to the right
            self.tmux.temporary_window(window_name, ALL_WINDOW_COMMANDS[window_name])
        elif self.spot_driver_on:
            self.spot_driver_on = False
            self.stop_slider.setSliderPosition(0)  # Push slider to the left
            self.tmux.kill_window(window_name)
        else:
            self.stop_slider.setSliderPosition(0)  # Push slider to the left

    def status_callback(self, msg: BatteryStateArray):
        if not self.spot_driver_on and not self.initial_spot_driver_check:
            self.initial_spot_driver_check = True
            self.spot_driver_on = True
            self.stop_slider.setSliderPosition(100)  # Push slider to the right

        time_left = msg.battery_states[0].estimated_runtime.sec // 60
        battery_percent = msg.battery_states[0].charge_percentage

        self.status_label.setText(
            f"Remaining:  {int(battery_percent)}% | {time_left} min"
        )

    def run_ssh_command(self):
        self.tmux.temporary_window(
            "spot_driver", ALL_WINDOW_COMMANDS["spot_driver"], 1
        )

        self.tmux.temporary_window(
            "kinova_driver", ALL_WINDOW_COMMANDS["kinova_driver"], 1
        )
        self.tmux.temporary_window(
            "kinova_moveit", ALL_WINDOW_COMMANDS["kinova_moveit"], 2
        )
        self.tmux.temporary_window(
            "kinova_vision", ALL_WINDOW_COMMANDS["kinova_vision"], 3
        )

        self.tmux.temporary_window("realsenses", ALL_WINDOW_COMMANDS["realsenses"], 4)
        self.tmux.temporary_window("octomap", ALL_WINDOW_COMMANDS["octomap"], 5)
        self.tmux.temporary_window("realsenses", ALL_WINDOW_COMMANDS["realsenses"], 6)

    def hazmat_command(self):
        self.tmux.temporary_window(
            "hazmat_detection", ALL_WINDOW_COMMANDS["hazmat_detection"]
        )

    def world_reset_command(self):
        if not self.map_frame_reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "service map_reset not available, skipping command"
            )
        else:
            self.map_frame_reset_client.call_async(Trigger.Request())

        if not self.path_reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "service path_reset not available, skipping command"
            )
        else:
            self.path_reset_client.call_async(Empty.Request())

        if not self.world_info_reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "service world_info_reset not available, skipping command"
            )
        else:
            self.world_info_reset_client.call_async(Empty.Request())

        if not self.octomap_reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "service octomap_reset not available, skipping command"
            )
        else:
            self.octomap_reset_client.call_async(Empty.Request())

        if not self.octomap_nav_reset_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "service navigation/octomap_reset not available, skipping command"
            )
        else:
            self.octomap_nav_reset_client.call_async(Empty.Request())
            
    # BLOCKSWORLD
    def block_loc_update(self, msg):
        if self.blocks_scan_checked:
            self.bw_status_label.setText(f"{msg.data}")
        else:
            self.bw_status_label.setText("Unknown")

    def blocks_scan(self, checked):
        window_name = "blocksworld_scan"
        if checked:
            self.blocks_scan_checked = True

            self.tmux.temporary_window(window_name, ALL_WINDOW_COMMANDS[window_name])

            self.toggle_blocks_scan.setStyleSheet(
                "background-color: green; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
            )
        else:  # Button is unchecked, stop the command
            self.blocks_scan_checked = False
            self.bw_status_label.setText("Unknown")
            self.toggle_blocks_scan.setStyleSheet(
                "background-color: transparent; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
            )

            self.tmux.kill_window(window_name)

    def blocks_manipulator_position(self):
        self.moveit.send_goal([0, 36, -84, 0, -60, 90])

    def blocks_execute(self, checked):
        if checked:
            self.tmux.temporary_window(
                "blocksworld_gpp_wrapper", ALL_WINDOW_COMMANDS["blocksworld_gpp_wrapper"]
            )
            self.tmux.temporary_window("blocksworld_gpp_agent", ALL_WINDOW_COMMANDS["blocksworld_gpp_agent"])

            self.toggle_blocks_execute.setStyleSheet(
                "background-color: green; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
            )
        else:  # Button is unchecked, stop the command
            self.bw_status_label.setText("Unknown")
            self.toggle_blocks_execute.setStyleSheet(
                "background-color: transparent; font: bold 12px; border-width: 5px; border-radius: 10px; padding: 10px"
            )

            self.tmux.kill_window("blocksworld_gpp_wrapper")
            self.tmux.kill_window("blocksworld_gpp_agent")

    # Exploration
    def toggle_exploration(self, checked):
        frontier_window_name = "exp_frontier"
        detection_window_name = "exp_detection"
        map_save_window_name = "exp_auto_save_map"

        map_autosaver_command = f"""
while true; do
  cd && mkdir -p exp_maps/autosaved && cd exp_maps/autosaved && {ALL_WINDOW_COMMANDS["exp_save_map"]} 103
  sleep 10
done
"""

        if checked:
            self.tmux.temporary_window(frontier_window_name, "sleep 5 && " + ALL_WINDOW_COMMANDS[frontier_window_name])
            self.tmux.temporary_window(detection_window_name, ALL_WINDOW_COMMANDS[detection_window_name])

            self.tmux.temporary_window(map_save_window_name, map_autosaver_command)
        else:
            self.tmux.kill_window(frontier_window_name)
            self.tmux.kill_window(detection_window_name)

            self.tmux.kill_window(map_save_window_name)

    def lap_value_update(self, value):
        # Update the label
        self.exploration_slider_label.setText(f"Lap: {value}")
        self.lap_number = value
    
    def save_exp_maps(self):
        map_save_window_name = "exp_save_map"
        command = f"cd && mkdir -p exp_maps/{self.lap_number} && cd exp_maps/{self.lap_number} && " + ALL_WINDOW_COMMANDS[map_save_window_name] + f" {self.lap_number}"
        self.tmux.temporary_window(map_save_window_name, command)

    # Navigation
    def toggle_navigation(self, checked):
        window_name = "nav_3d_to_2d"
        if checked:
            self.tmux.temporary_window(window_name, ALL_WINDOW_COMMANDS[window_name])
        else:
            self.tmux.kill_window(window_name)

    def follow_path_command(self):
        if not self.nav_3d_follow_path_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "service /navigation/follow_path not available, skipping command"
            )
        else:
            req = SetBool.Request()
            req.data = True
            self.nav_3d_follow_path_client.call_async(req)

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
    sys.exit(Main().main(sys.argv, standalone="my_rqt_plugin.my_rqt_plugin"))


if __name__ == "__main__":
    main()
