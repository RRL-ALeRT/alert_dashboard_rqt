# ALeRT Dashboard RQT

RQT plugins for ALeRT's Spot robot control and debugging.

## Plugins

1. **Spot EStop RQT** - Emergency stop and robot control
2. **Alert Dashboard RQT** - Debug console and process management

## Architecture

These plugins communicate with the robot PC via a **tmux API server** that provides:
- REST API for process control (start/stop/list)
- WebSocket for instant crash notifications
- Non-blocking connection (RQT starts immediately)
- Preserves tmux for manual debugging

### Components

- **`tmux_api_server/`** - FastAPI server (runs on robot PC)
- **`tmux_api_client.py`** - Python client library (used by RQT plugins)
- **`alert_dashboard_rqt.py`** - Main dashboard plugin
- **`spot_estop_rqt.py`** - EStop control plugin

## Setup

### Robot PC

Install and start the tmux API server:

```bash
cd tmux_api_server
pip3 install -r requirements.txt
sudo cp tmux-api-server.service /etc/systemd/system/
sudo systemctl enable --now tmux-api-server
```

See [`tmux_api_server/README.md`](tmux_api_server/README.md) for details.

### Operator PC

Install WebSocket client:

```bash
pip3 install websocket-client
```

Launch RQT:

```bash
ros2 run rqt_gui rqt_gui
```

## Features

- ✅ **Instant crash detection** - WebSocket notifications when processes die
- ✅ **Non-blocking startup** - RQT opens immediately, connects in background
- ✅ **Visual status indicators** - Color-coded labels (green/red/orange/yellow)
- ✅ **Manual debugging** - SSH into robot and attach to tmux as usual
- ✅ **Toggle support** - All existing toggle buttons work unchanged

## Environment Variables

Required on both operator and robot PC:

```bash
export MAX_USER="max"
export MAX_IP="192.168.1.100"  # Robot IP
export GEN3_IP="192.168.1.10"  # Kinova arm IP
```