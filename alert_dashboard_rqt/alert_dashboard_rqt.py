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

# Import shared window commands
from alert_dashboard_rqt.window_commands import ALL_WINDOW_COMMANDS


MAX_USER = os.getenv("MAX_USER")
MAX_IP = os.getenv("MAX_IP", "localhost")  # Default to localhost for local testing
SESSION_NAME = "spot_session"

if MAX_USER is None:
    print("MAX_USER environment variable is not set.")
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
            
            # Skip if button doesn't exist in UI
            if self.bnl[f"{button}_push"] is None:
                continue
                
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

        # Initialize with all windows expected to be active (toggles on by default)
        # Only include windows that have UI buttons
        self.expected_active_windows = [
            btn for btn in ALL_WINDOW_COMMANDS.keys() 
            if self.bnl.get(f"{btn}_push") is not None
        ]
        
        # Set all toggles to checked by default
        for button in self.expected_active_windows:
            self.bnl[f"{button}_push"].setChecked(True)
        
        self.node.create_timer(2, self.check_expected_windows)

        # Use API client instead of SSH
        from alert_dashboard_rqt.tmux_api_client import TmuxAPIClient
        self.tmux = TmuxAPIClient(MAX_IP, SESSION_NAME)
        
        self.tmux.new_session()

    def check_expected_windows(self):
        # Check connection status first
        if not self.tmux.connected:
            # Show disconnected status for all expected windows
            error_msg = f"⚠️ Disconnected"
            if self.tmux.last_error:
                error_msg += f": {self.tmux.last_error}"
            
            for window in self.expected_active_windows:
                self.bnl[f"{window}_label"].setText(error_msg)
                self.bnl[f"{window}_label"].setStyleSheet("color: orange")
            return
        
        # Get window list with process status
        try:
            import requests
            response = requests.get(
                f"{self.tmux.base_url}/api/windows",
                headers=self.tmux.headers,
                timeout=2
            )
            if response.status_code != 200:
                return
            
            data = response.json()
            windows_status = {w["name"]: w for w in data.get("windows", [])}
        except Exception as e:
            print(f"Failed to get window status: {e}")
            return

        # Check each expected window
        for expected_window in self.expected_active_windows[:]:  # Use slice to iterate over copy
            window_info = windows_status.get(expected_window)
            
            if window_info is None:
                # Window doesn't exist at all - uncheck toggle and clear label
                self.bnl[f"{expected_window}_push"].setChecked(False)
                self.bnl[f"{expected_window}_label"].setText("")
                self.bnl[f"{expected_window}_label"].setStyleSheet("")
                # Remove from expected list
                self.expected_active_windows.remove(expected_window)
            
            elif not window_info.get("has_process", False):
                # Window exists but no process running (CRASHED)
                # Only update if not already showing crash status
                if self.bnl[f"{expected_window}_label"].text() != "💥 CRASHED":
                    self.bnl[f"{expected_window}_label"].setText("💥 CRASHED")
                    self.bnl[f"{expected_window}_label"].setStyleSheet(
                        "color: red; font-weight: bold; background-color: yellow"
                    )
            
            else:
                # Window exists AND process is running
                self.bnl[f"{expected_window}_label"].setText("✓ Running")
                self.bnl[f"{expected_window}_label"].setStyleSheet("color: green")

        # Auto press buttons if the program windows are already running
        for window_name, window_info in windows_status.items():
            if window_name not in ALL_WINDOW_COMMANDS.keys():
                continue

            if window_name not in self.expected_active_windows and window_info.get("has_process", False):
                self.expected_active_windows.append(window_name)
                self.bnl[f"{window_name}_push"].setChecked(True)


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
            if window_name in self.expected_active_windows:
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
