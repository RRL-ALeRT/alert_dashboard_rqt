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
MAX_IP = os.getenv("MAX_IP", "localhost")
SESSION_NAME = "spot_session"

if MAX_USER is None:
    print("MAX_USER environment variable is not set.")
    sys.exit(1)

class DashboardRqtPlugin(Plugin):
    def __init__(self, context):
        """Initialize the plugin."""
        super().__init__(context)
        self.node = context.node

        ui_path = (
            get_package_share_directory("alert_dashboard_rqt")
            + "/resource/mainwindow.ui"
        )

        self._widget = QWidget()
        loadUi(ui_path, self._widget)
        context.add_widget(self._widget)

        # Dictionary to store references to UI elements
        self.bnl = {}
        
        for button in ALL_WINDOW_COMMANDS.keys():
            # 1. PUSH BUTTONS: These are toggles (On/Off)
            push_name = f"{button}_push"
            btn_push = self._widget.findChild(QPushButton, push_name)
            if btn_push:
                self.bnl[push_name] = btn_push
                # CRITICAL: This allows setChecked() and the toggled signal to work
                btn_push.setCheckable(True) 
                btn_push.toggled.connect(
                    lambda checked, btn=button: self.button_push_cb(btn, checked)
                )

            # 2. TOOL BUTTONS: These are momentary triggers (Focus/Select)
            tool_name = f"{button}_tool"
            btn_tool = self._widget.findChild(QToolButton, tool_name)
            if btn_tool:
                self.bnl[tool_name] = btn_tool
                # ToolButtons are NOT checkable; they just fire a click event
                btn_tool.clicked.connect(
                    lambda _, btn=button: self.toolbutton_cb(btn)
                )

            # 3. LABELS: For status display
            label_name = f"{button}_label"
            lbl = self._widget.findChild(QLabel, label_name)
            if lbl:
                self.bnl[label_name] = lbl

        # Initialize tracking list for windows we expect to be running
        self.expected_active_windows = []
        
        # Use API client for TMUX control
        from alert_dashboard_rqt.tmux_api_client import TmuxAPIClient
        self.tmux = TmuxAPIClient(MAX_IP, SESSION_NAME)
        self.tmux.new_session()

        # Timer to poll status every 2 seconds
        self.node.create_timer(2.0, self.check_expected_windows)

    def check_expected_windows(self):
        """Poll the robot to see which windows are actually running."""
        if not self.tmux.connected:
            error_msg = "⚠️ Disconnected"
            if self.tmux.last_error:
                error_msg += f": {self.tmux.last_error}"
            
            for window in ALL_WINDOW_COMMANDS.keys():
                lbl = self.bnl.get(f"{window}_label")
                if lbl:
                    lbl.setText(error_msg)
                    lbl.setStyleSheet("color: orange")
            return

        try:
            windows_metadata = self.tmux.get_windows_status()
            if windows_metadata is None:
                return
            
            # Map window names to their status info
            windows_status = {w["name"]: w for w in windows_metadata}
        except Exception as e:
            print(f"Failed to get window status: {e}")
            return

        for window_name in ALL_WINDOW_COMMANDS.keys():
            window_info = windows_status.get(window_name)
            lbl = self.bnl.get(f"{window_name}_label")
            btn = self.bnl.get(f"{window_name}_push")

            # Block signals temporarily to prevent infinite loop while we sync the UI state
            if btn:
                btn.blockSignals(True)

            if window_info is None:
                # Window is dead/not present
                if btn:
                    btn.setChecked(False)
                if lbl:
                    lbl.setText("")
                    lbl.setStyleSheet("")
                if window_name in self.expected_active_windows:
                    self.expected_active_windows.remove(window_name)
            
            elif not window_info.get("has_process", False):
                # Window exists but shell is empty (Crash)
                if btn:
                    btn.setChecked(True)
                if lbl:
                    lbl.setText("💥 CRASHED")
                    lbl.setStyleSheet("color: red; font-weight: bold; background-color: yellow")
            
            else:
                # Window is healthy
                if btn:
                    btn.setChecked(True)
                if lbl:
                    lbl.setText("✓ Running")
                    lbl.setStyleSheet("color: green")
                if window_name not in self.expected_active_windows:
                    self.expected_active_windows.append(window_name)

            # Re-enable signals after sync
            if btn:
                btn.blockSignals(False)

    def button_push_cb(self, button_name, checked):
        """Handle the On/Off toggle for a specific window."""
        if checked:
            if button_name not in self.expected_active_windows:
                self.tmux.temporary_window(
                    button_name,
                    ALL_WINDOW_COMMANDS[button_name],
                )
                self.expected_active_windows.append(button_name)
        else:
            self.tmux.kill_window(button_name)
            if button_name in self.expected_active_windows:
                self.expected_active_windows.remove(button_name)
            
            lbl = self.bnl.get(f"{button_name}_label")
            if lbl:
                lbl.setText("")

    def toolbutton_cb(self, button_name):
        """Handle focusing/selecting a window."""
        print(f"Selecting window: {button_name}")
        self.tmux.window_select(button_name)

    def shutdown_plugin(self):
        self.node.destroy_node()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

def main():
    """Run the plugin."""
    rclpy.init()
    sys.exit(
        Main().main(sys.argv, standalone="alert_dashboard_rqt.alert_dashboard_rqt")
    )

if __name__ == "__main__":
    main()
