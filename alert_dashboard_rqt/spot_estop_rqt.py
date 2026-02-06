#!/usr/bin/env python3

import sys
import os
import subprocess
from functools import partial

import rclpy
from rclpy.qos import QoSProfile

from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, SetBool
try:
    from spot_msgs.msg import BatteryStateArray
except ImportError:
    BatteryStateArray = None

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
# Import shared window commands
from alert_dashboard_rqt.window_commands import ALL_WINDOW_COMMANDS

MAX_USER = os.getenv("MAX_USER")
MAX_IP = os.getenv("MAX_IP", "localhost")  # Default to localhost for local testing
SESSION_NAME = "spot_session"

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
        if BatteryStateArray is not None:
            self.node.create_subscription(
                BatteryStateArray, "/status/battery_states", self.status_callback, 1
            )

        # Create service clients
        self.map_frame_reset_client = self.node.create_client(
            Trigger, "/reset_map_frame"
        )

        self.spot_power_off_client = self.node.create_client(
            Trigger, "/power_off"
        )
        self.spot_power_on_client = self.node.create_client(
            Trigger, "/power_on"
        )
        self.toggle_led_client = self.node.create_client(
            SetBool, "/arduino_lights"
        )

        self.path_reset_client = self.node.create_client(Empty, "/reset_travelled_path")
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
        
        # Create label for estop window status
        self.estop_status_label = QLabel("")
        self.estop_status_label.setAlignment(Qt.AlignCenter)
        self.estop_status_label.setStyleSheet("font: bold 14px")

        first_row_buttons = QHBoxLayout()
        first_row_buttons.addWidget(stop_label)
        first_row_buttons.addWidget(self.stop_slider)

        second_row_buttons = QHBoxLayout()
        second_row_buttons.addWidget(drivers_button)

        first_tab_layout.addLayout(first_row_buttons)
        first_tab_layout.addWidget(self.estop_status_label)  # Add status above battery info
        first_tab_layout.addLayout(second_row_buttons)
        first_tab_layout.addWidget(self.status_label)

        ### SECOND TAB
        ## Create a toggle button for running/stopping gologpp frame runner
        self.toggle_button = QPushButton("Frame Runner")
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(False)  # Set the initial state
        self.toggle_button.toggled.connect(self.toggle_frame_runner)
        self.toggle_button.setStyleSheet(
            "background-color: transparent; font: bold 15px; border-width: 5px; border-radius: 15px; padding: 15px"
        )
        
        # Add status label for frame_runner
        self.frame_runner_status_label = QLabel("")
        self.frame_runner_status_label.setAlignment(Qt.AlignCenter)
        self.frame_runner_status_label.setStyleSheet("font: bold 12px")
        
        frame_runner_layout = QVBoxLayout()
        frame_runner_layout.addWidget(self.toggle_button)
        frame_runner_layout.addWidget(self.frame_runner_status_label)
        
        second_tab_layout.addLayout(frame_runner_layout)

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

        plus_second_row_buttons = QHBoxLayout()
        plus_second_row_buttons.addWidget(hazmat_button)
        plus_second_row_buttons.addWidget(world_reset_button)
        plus_second_row_buttons.addWidget(two_way_audio)

        second_tab_layout.addLayout(plus_second_row_buttons)

        spot_power_off_on_button = QPushButton("Spot Power Off/On")
        spot_power_off_on_button.clicked.connect(self.spot_power_off_on_command)
        spot_power_off_on_button.setStyleSheet(
            "background-color: pink; font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px"
        )

        # Create a toggle button for audio communication
        spot_lights = QPushButton("Lights")
        spot_lights.setCheckable(True)
        spot_lights.setStyleSheet(
            "QPushButton { font: bold 10px; border-width: 5px; border-radius:10px; padding: 10px }"
            "QPushButton:checked { background-color: green }"
            "QPushButton:unchecked { background-color: transparent }"
        )
        spot_lights.clicked.connect(partial(self.toggle_spot_lights, spot_lights))

        plus_third_row_buttons = QHBoxLayout()
        plus_third_row_buttons.addWidget(spot_power_off_on_button)
        plus_third_row_buttons.addWidget(spot_lights)

        second_tab_layout.addLayout(plus_third_row_buttons)

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

        # Use API client instead of SSH
        from alert_dashboard_rqt.tmux_api_client import TmuxAPIClient
        self.tmux = TmuxAPIClient(MAX_IP, SESSION_NAME)
        self.tmux.new_session()
        
        # Add timer to check estop status and reset slider if needed
        self.node.create_timer(2, self.check_estop_status)

    def check_estop_status(self):
            """Check if estop and frame_runner windows are running and sync UI state"""
            if not self.tmux.connected:
                self.estop_status_label.setText("⚠️ Disconnected from API")
                self.estop_status_label.setStyleSheet("color: orange; font: bold 14px")
                self.frame_runner_status_label.setText("⚠️ Disconnected")
                self.frame_runner_status_label.setStyleSheet("color: orange; font: bold 12px")
                return
            
            try:
                import requests
                response = requests.get(
                    f"{self.tmux.base_url}/api/windows",
                    headers=self.tmux.headers,
                    timeout=2
                )
                if response.status_code != 200:
                    self.estop_status_label.setText(f"⚠️ API Error: {response.status_code}")
                    self.estop_status_label.setStyleSheet("color: orange; font: bold 14px")
                    self.frame_runner_status_label.setText(f"⚠️ API Error")
                    self.frame_runner_status_label.setStyleSheet("color: orange; font: bold 12px")
                    return
                
                data = response.json()
                windows_status = {w["name"]: w for w in data.get("windows", [])}
                
                # Check estop window
                estop_window = windows_status.get("estop")
                
                if estop_window is None:
                    # Window doesn't exist - Reset slider
                    if self.spot_driver_on:
                        self.spot_driver_on = False
                        self.stop_slider.setSliderPosition(0)
                    
                    self.estop_status_label.setText("")  # Not started
                    self.estop_status_label.setStyleSheet("")

                elif not estop_window.get("has_process", False):
                    # Window exists but crashed - KEEP session alive (don't reset slider)
                    self.estop_status_label.setText("💥 E-Stop CRASHED")
                    self.estop_status_label.setStyleSheet("color: red; font-weight: bold; background-color: yellow; font: bold 14px; padding: 5px")
                    
                    # Ensure slider stays at 100 to reflect "Session Active" despite crash
                    if self.spot_driver_on and self.stop_slider.value() != 100:
                        self.stop_slider.setSliderPosition(100)

                else:
                    # Estop is running normally - ensure slider is at 100 and flag is set
                    if not self.spot_driver_on:
                        self.spot_driver_on = True
                        self.stop_slider.setSliderPosition(100)
                    elif self.stop_slider.value() != 100:
                        self.stop_slider.setSliderPosition(100)
                    
                    # Update status label
                    self.estop_status_label.setText("✓ E-Stop Running")
                    self.estop_status_label.setStyleSheet("color: green; font: bold 14px")
                
                # Check frame_runner window
                frame_runner_window = windows_status.get("frame_runner")
                
                if frame_runner_window is None:
                    # Window doesn't exist - uncheck toggle if checked
                    if self.toggle_button.isChecked():
                        self.toggle_button.setChecked(False)
                    self.frame_runner_status_label.setText("")
                    self.frame_runner_status_label.setStyleSheet("")
                elif not frame_runner_window.get("has_process", False):
                    # Window exists but crashed
                    self.frame_runner_status_label.setText("💥 CRASHED")
                    self.frame_runner_status_label.setStyleSheet("color: red; font-weight: bold; background-color: yellow; font: bold 12px; padding: 3px")
                else:
                    # Window is running
                    if not self.toggle_button.isChecked():
                        self.toggle_button.setChecked(True)
                    self.frame_runner_status_label.setText("✓ Running")
                    self.frame_runner_status_label.setStyleSheet("color: green; font: bold 12px")
                    
            except Exception as e:
                self.estop_status_label.setText(f"⚠️ Error: {str(e)[:50]}")
                self.estop_status_label.setStyleSheet("color: orange; font: bold 14px")
                self.frame_runner_status_label.setText("⚠️ Error")
                self.frame_runner_status_label.setStyleSheet("color: orange; font: bold 12px")

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

    def spot_power_off_on_command(self):
        if not self.spot_power_off_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "service power_off not available, skipping command"
            )
        else:
            self.spot_power_off_client.call_async(Trigger.Request())

        if not self.spot_power_on_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info(
                "service power_on not available, skipping command"
            )
        else:
            self.spot_power_on_client.call_async(Trigger.Request())

    def toggle_spot_lights(self, button):
        if button.isChecked():
            req = SetBool.Request()
            req.data = True

            if not self.toggle_led_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(
                    "service toggle_led not available, skipping command"
                )
            else:
                self.toggle_led_client.call_async(req)
        else:
            req = SetBool.Request()
            req.data = False
            
            if not self.toggle_led_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info(
                    "service toggle_led not available, skipping command"
                )
            else:
                self.toggle_led_client.call_async(req)

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

    def stop_slider_released(self):
        window_name = "estop"
        if self.stop_slider.value() > 50 and not self.spot_driver_on:
            self.spot_driver_on = True
            self.stop_slider.setSliderPosition(100)  # Push slider to the right
            #self.tmux.temporary_window("discovery", ALL_WINDOW_COMMANDS["discovery"])
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
            "kinova_python", ALL_WINDOW_COMMANDS["kinova_python"], 1
        )
        self.tmux.temporary_window(
            "kinova_vision", ALL_WINDOW_COMMANDS["kinova_vision"], 2
        )

        self.tmux.temporary_window("realsenses", ALL_WINDOW_COMMANDS["realsenses"], 3)
        self.tmux.temporary_window("octo_spot", ALL_WINDOW_COMMANDS["octo_spot"], 4)
        self.tmux.temporary_window("realsenses", ALL_WINDOW_COMMANDS["realsenses"], 5)

        self.tmux.temporary_window(
            "spot_driver", ALL_WINDOW_COMMANDS["spot_driver"], 6
        )


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
