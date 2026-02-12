#!/usr/bin/env python3
"""
tmux API Client - Drop-in replacement for TmuxSSHWrapper
Communicates with tmux API server via HTTP REST API
"""

import os
import threading
import time
from typing import List, Callable, Optional
import requests



class TmuxAPIClient:
    """
    API client for tmux session management.
    Drop-in replacement for TmuxSSHWrapper with same interface.
    """
    
    def __init__(self, robot_ip: str, session_name: str, port: int = 8000):
        """
        Initialize API client
        
        Args:
            robot_ip: IP address of robot PC
            session_name: tmux session name (for compatibility, not used)
            port: API server port
        """
        self.base_url = f"http://{robot_ip}:{port}"
        self.session_name = session_name
        self.connected = False
        self.last_error = None  # Track last error for UI display
        self._reconnect_thread = None
        self._should_reconnect = True
        
        # Try to connect in background (non-blocking)
        self._connect_async()
    
    def _connect_async(self):
        """Connect to API server in background thread with auto-retry"""
        def connect_loop():
            while self._should_reconnect:
                try:
                    # Test HTTP connection
                    response = requests.get(f"{self.base_url}/health", timeout=2)
                    if response.status_code == 200:
                        if not self.connected:  # Only print on state change
                            print(f"✓ Connected to tmux API at {self.base_url}")
                        self.connected = True
                        self.last_error = None
                    else:
                        if self.connected:  # Only print on state change
                            print(f"⚠️  tmux API returned status {response.status_code}")
                        self.connected = False
                        self.last_error = f"API returned status {response.status_code}"
                except Exception as e:
                    if self.connected:  # Only print on state change
                        print(f"⚠️  Lost connection to tmux API: {e}")
                    self.connected = False
                    self.last_error = str(e)
                
                # Wait before next connection attempt (5 seconds)
                time.sleep(5)
        
        self._reconnect_thread = threading.Thread(target=connect_loop, daemon=True)
        self._reconnect_thread.start()
    
    def new_session(self):
        """Create new session (no-op, session exists on server)"""
        pass
    
    def new_window(self, window_name: str):
        """
        Create a new window
        
        Args:
            window_name: Name of the window
        """
        if not self.connected:
            self.last_error = f"Not connected, cannot create window: {window_name}"
            print(f"⚠️  {self.last_error}")
            return
        
        try:
            # Just create window, don't send command yet
            # This matches the SSH wrapper behavior
            pass
        except Exception as e:
            self.last_error = f"Error creating window {window_name}: {e}"
            print(f"{self.last_error}")
    
    def temporary_window(self, window_name: str, command: str, delay: int = 1):
        """
        Create window and run command
        
        Args:
            window_name: Name of the window
            command: Command to execute
            delay: Delay before sending command (seconds)
        """
        if not self.connected:
            self.last_error = f"Not connected, cannot start window: {window_name}"
            print(f"⚠️  {self.last_error}")
            return
        
        try:
            response = requests.post(
                f"{self.base_url}/api/windows/{window_name}/start",
                json={"command": command, "delay": delay},
                timeout=5
            )
            if response.status_code == 200:
                self.last_error = None
                print(f"✓ Started window: {window_name}")
            else:
                self.last_error = f"Failed to start {window_name}: {response.text}"
                print(self.last_error)
        except Exception as e:
            self.connected = False
            self.last_error = f"Error starting {window_name}: {e}"
            print(self.last_error)
    
    def kill_window(self, window_name: str):
        """
        Kill a window
        
        Args:
            window_name: Name of the window to kill
        """
        if not self.connected:
            self.last_error = f"Not connected, cannot kill window: {window_name}"
            print(f"⚠️  {self.last_error}")
            return
        
        try:
            response = requests.delete(
                f"{self.base_url}/api/windows/{window_name}",
                timeout=5
            )
            if response.status_code == 200:
                self.last_error = None
                print(f"✓ Killed window: {window_name}")
            elif response.status_code == 404:
                # Window doesn't exist, that's fine
                self.last_error = None
                print(f"⚠️  Window {window_name} doesn't exist")
            else:
                self.last_error = f"Failed to kill {window_name}: {response.text}"
                print(self.last_error)
        except Exception as e:
            self.connected = False
            self.last_error = f"Error killing {window_name}: {e}"
            print(self.last_error)
    
    def window_command(self, window_name: str, command: str, delay: int = 0):
        """
        Send command to existing window
        
        Args:
            window_name: Name of the window
            command: Command to send
            delay: Delay before sending (seconds)
        """
        # For API, this is same as temporary_window
        self.temporary_window(window_name, command, delay)
    
    def window_cancel(self, window_name: str):
        """Send Ctrl-C to window (not implemented for API)"""
        print(f"⚠️  window_cancel not implemented for API")
    
    def window_select(self, window_name: str):
        """
        Select/focus a window
        
        Args:
            window_name: Name of the window to select
        """
        if not self.connected:
            return
            
        try:
            requests.post(
                f"{self.base_url}/api/windows/{window_name}/select",
                timeout=2
            )
        except Exception as e:
            print(f"Error selecting window {window_name}: {e}")
    
    def get_active_windows(self) -> List[str]:
        """
        Get list of active window names
        
        Returns:
            List of window names
        """
        if not self.connected:
            return []
        
        try:
            response = requests.get(
                f"{self.base_url}/api/windows",
                timeout=2
            )
            if response.status_code == 200:
                data = response.json()
                return [w["name"] for w in data["windows"]]
            else:
                print(f"Failed to get windows: {response.text}")
                return []
        except Exception as e:
            print(f"Error getting windows: {e}")
            self.connected = False
            return []
