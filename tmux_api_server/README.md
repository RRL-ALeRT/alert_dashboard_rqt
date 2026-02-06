# tmux API Server Installation and Setup

## Installation on Robot PC

### 1. Install Dependencies

```bash
cd /home/max/ros2_ws/src/alert_dashboard_rqt/tmux_api_server
pip3 install -r requirements.txt
```

### 2. Install Systemd Service

```bash
# Copy service file
sudo cp tmux-api-server.service /etc/systemd/system/

# Reload systemd
sudo systemctl daemon-reload

# Enable auto-start on boot
sudo systemctl enable tmux-api-server

# Start the service
sudo systemctl start tmux-api-server

# Check status
sudo systemctl status tmux-api-server
```

### 3. Verify API is Running

```bash
# Test health endpoint
curl http://localhost:8000/health

# Should return: {"status":"healthy","session":"spot_session"}
```

## Installation on Operator PC

### Test RQT Plugin

```bash
# Launch RQT
ros2 run rqt_gui rqt_gui

# Load the alert_dashboard_rqt plugin
# UI starts immediately (no blocking on SSH connection)
# Status labels show "⚠️ Disconnected" until API connects (within 5 seconds)
# Client auto-reconnects if connection is lost
```

## Usage

### Starting/Stopping Processes

- Click buttons in RQT as before
- **Crash detection**: Process crashes detected within 2 seconds via HTTP polling
- **Auto-reconnect**: Client reconnects automatically if connection lost (checks every 5 seconds)
- **Connection status**: Shows "⚠️ Disconnected" if robot unreachable

### Manual Debugging

SSH into robot and attach to tmux as before:

```bash
ssh max@robot-ip
tmux attach -t spot_session
```

All your tmux debugging workflow stays the same!

## Troubleshooting

### API Not Starting

```bash
# Check logs
sudo journalctl -u tmux-api-server -f

# Common issues:
# - Port 8000 already in use
# - Python dependencies not installed
# - tmux not installed
```

### RQT Shows "Disconnected"

```bash
# Check if API is running on robot
curl http://robot-ip:8000/health

# Check firewall
sudo ufw allow 8000/tcp
```

### Connection Issues

```bash
# Test HTTP connection manually
curl http://robot-ip:8000/health

# Test window status
curl http://robot-ip:8000/api/windows

# Should return JSON with window list and process status
```

## Uninstalling

```bash
# Stop and disable service
sudo systemctl stop tmux-api-server
sudo systemctl disable tmux-api-server

# Remove service file
sudo rm /etc/systemd/system/tmux-api-server.service
sudo systemctl daemon-reload
```

## Reverting to SSH

If you need to revert to SSH:

```python
# In alert_dashboard_rqt.py and spot_estop_rqt.py
# Comment out API client:
# from alert_dashboard_rqt.tmux_api_client import TmuxAPIClient
# self.tmux = TmuxAPIClient(MAX_IP, SESSION_NAME)

# Uncomment SSH wrapper:
from alert_dashboard_rqt.tmux_utils import TmuxSSHWrapper
self.tmux = TmuxSSHWrapper(MAX_USER, MAX_IP, SESSION_NAME)
```
