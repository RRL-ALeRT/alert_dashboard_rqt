# tmux API Server - Production Configuration Guide

## Environment Variables

### Required

```bash
# tmux session name
export TMUX_SESSION_NAME="spot_session"
```

### Optional - Security

```bash
# API key for authentication (recommended for production)
# Generate with: openssl rand -hex 32
export TMUX_API_KEY="your-secret-api-key-here"
```

### Optional - Logging

```bash
# Log level: DEBUG, INFO, WARNING, ERROR, CRITICAL
export LOG_LEVEL="INFO"
```

### Optional - Performance

```bash
# Window monitor interval in seconds (default: 0.5)
export MONITOR_INTERVAL="0.5"
```

## Configuration Examples

### Development (No Auth)

```bash
# .env file for development
TMUX_SESSION_NAME=spot_session
LOG_LEVEL=DEBUG
MONITOR_INTERVAL=0.5
```

### Production (With Auth)

```bash
# .env file for production
TMUX_SESSION_NAME=spot_session
TMUX_API_KEY=a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6q7r8s9t0u1v2w3x4y5z6
LOG_LEVEL=INFO
MONITOR_INTERVAL=0.5
```

## Systemd Service with Environment

Update `/etc/systemd/system/tmux-api-server.service`:

```ini
[Unit]
Description=tmux API Server for Robot Control
After=network.target

[Service]
Type=simple
User=max
WorkingDirectory=/home/max/ros2_ws/src/alert_dashboard_rqt/tmux_api_server

# Environment variables
Environment="TMUX_SESSION_NAME=spot_session"
Environment="TMUX_API_KEY=your-secret-key-here"
Environment="LOG_LEVEL=INFO"
Environment="MONITOR_INTERVAL=0.5"

ExecStart=/usr/bin/python3 /home/max/ros2_ws/src/alert_dashboard_rqt/tmux_api_server/main.py
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

## Client Configuration

### With API Key

```python
from alert_dashboard_rqt.tmux_api_client import TmuxAPIClient

# Option 1: Pass API key directly
tmux = TmuxAPIClient(MAX_IP, SESSION_NAME, api_key="your-secret-key")

# Option 2: Use environment variable
# export TMUX_API_KEY="your-secret-key"
tmux = TmuxAPIClient(MAX_IP, SESSION_NAME)
```

### Without API Key (Development)

```python
# Just don't set TMUX_API_KEY on server or client
tmux = TmuxAPIClient(MAX_IP, SESSION_NAME)
```

## Monitoring

### Health Check

```bash
curl http://robot-ip:8000/health
```

Response:
```json
{
  "status": "healthy",
  "session": "spot_session",
  "session_exists": true,
  "window_count": 5,
  "uptime_seconds": 3600.5,
  "timestamp": "2026-02-06T17:45:00.123456"
}
```

### Metrics

```bash
curl http://robot-ip:8000/metrics
```

Response:
```json
{
  "uptime_seconds": 3600.5,
  "total_requests": 150,
  "total_errors": 2,
  "active_websockets": 1,
  "windows_started": 10,
  "windows_killed": 5,
  "windows_crashed": 3
}
```

## Logging

### View Logs

```bash
# Real-time logs
sudo journalctl -u tmux-api-server -f

# Last 100 lines
sudo journalctl -u tmux-api-server -n 100

# Logs from today
sudo journalctl -u tmux-api-server --since today
```

### Log Format

```
2026-02-06 17:45:00 | INFO     | tmux-api | Starting window | window=kinova_driver | command=ros2 launch kortex_bringup...
2026-02-06 17:45:05 | WARNING  | tmux-api | Window died | window=realsenses
2026-02-06 17:45:10 | ERROR    | tmux-api | Failed to start window | window=test | error=Session not found
```

## Security Best Practices

1. **Always use API key in production**
   ```bash
   export TMUX_API_KEY=$(openssl rand -hex 32)
   ```

2. **Restrict CORS origins** (edit `main.py`):
   ```python
   app.add_middleware(
       CORSMiddleware,
       allow_origins=["http://operator-pc:3000"],  # Specific origins only
       ...
   )
   ```

3. **Use firewall** (robot PC):
   ```bash
   sudo ufw allow from operator-ip to any port 8000
   ```

4. **Don't log API keys**:
   - Already handled: logs show `key=a1b2c3d4...` (truncated)

## Troubleshooting

### Configuration Errors

If server fails to start:
```bash
# Check logs
sudo journalctl -u tmux-api-server -n 50

# Common errors:
# - "Configuration error: log_level must be one of..."
#   Fix: Use valid log level (DEBUG/INFO/WARNING/ERROR/CRITICAL)
# - "Configuration error: monitor_interval must be greater than 0"
#   Fix: Set MONITOR_INTERVAL between 0.1 and 5.0
```

### Authentication Errors

Client gets 401/403:
```bash
# Check if API key is set on server
sudo systemctl show tmux-api-server | grep TMUX_API_KEY

# Check if client is using correct key
echo $TMUX_API_KEY
```

### Performance Tuning

```bash
# Slower polling (less CPU, slower crash detection)
export MONITOR_INTERVAL="1.0"

# Faster polling (more CPU, faster crash detection)
export MONITOR_INTERVAL="0.2"
```
