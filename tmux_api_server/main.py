#!/usr/bin/env python3
"""
tmux API Server - FastAPI service for managing tmux sessions
Provides REST API and WebSocket for robot process control
"""

import asyncio
import logging
import os
import sys
import time
from typing import Dict, List, Optional
from contextlib import asynccontextmanager
from datetime import datetime, UTC

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Header, Depends
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field, field_validator
import libtmux


# ============================================================================
# Configuration with validation
# ============================================================================

class Config(BaseModel):
    """Server configuration with validation"""
    session_name: str = Field(default="spot_session", min_length=1)
    api_key: Optional[str] = Field(default=None, min_length=8)
    log_level: str = Field(default="INFO")
    monitor_interval: float = Field(default=0.5, gt=0, le=5)
    
    @field_validator('log_level')
    @classmethod
    def validate_log_level(cls, v):
        valid_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
        if v.upper() not in valid_levels:
            raise ValueError(f"log_level must be one of {valid_levels}")
        return v.upper()
    
    @classmethod
    def from_env(cls):
        """Load config from environment variables"""
        return cls(
            session_name=os.getenv("TMUX_SESSION_NAME", "spot_session"),
            api_key=os.getenv("TMUX_API_KEY"),
            log_level=os.getenv("LOG_LEVEL", "INFO"),
            monitor_interval=float(os.getenv("MONITOR_INTERVAL", "0.5"))
        )


# Load and validate configuration
try:
    config = Config.from_env()
except Exception as e:
    print(f"Configuration error: {e}", file=sys.stderr)
    sys.exit(1)


# ============================================================================
# Structured Logging
# ============================================================================

class StructuredLogger:
    """Structured logging with context"""
    
    def __init__(self, name: str, level: str):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level))
        
        # Console handler with structured format
        handler = logging.StreamHandler()
        formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(name)s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
    
    def info(self, msg: str, **kwargs):
        extra = " | " + " | ".join(f"{k}={v}" for k, v in kwargs.items()) if kwargs else ""
        self.logger.info(f"{msg}{extra}")
    
    def warning(self, msg: str, **kwargs):
        extra = " | " + " | ".join(f"{k}={v}" for k, v in kwargs.items()) if kwargs else ""
        self.logger.warning(f"{msg}{extra}")
    
    def error(self, msg: str, **kwargs):
        extra = " | " + " | ".join(f"{k}={v}" for k, v in kwargs.items()) if kwargs else ""
        self.logger.error(f"{msg}{extra}")
    
    def debug(self, msg: str, **kwargs):
        extra = " | " + " | ".join(f"{k}={v}" for k, v in kwargs.items()) if kwargs else ""
        self.logger.debug(f"{msg}{extra}")


logger = StructuredLogger("tmux-api", config.log_level)


# ============================================================================
# Metrics
# ============================================================================

class Metrics:
    """Simple metrics tracking"""
    
    def __init__(self):
        self.start_time = time.time()
        self.request_count = 0
        self.error_count = 0
        self.websocket_connections = 0
        self.window_starts = 0
        self.window_kills = 0
        self.window_crashes = 0
    
    def record_request(self):
        self.request_count += 1
    
    def record_error(self):
        self.error_count += 1
    
    def record_websocket_connect(self):
        self.websocket_connections += 1
    
    def record_websocket_disconnect(self):
        self.websocket_connections -= 1
    
    def record_window_start(self):
        self.window_starts += 1
    
    def record_window_kill(self):
        self.window_kills += 1
    
    def record_window_crash(self):
        self.window_crashes += 1
    
    def get_stats(self) -> dict:
        uptime = time.time() - self.start_time
        return {
            "uptime_seconds": round(uptime, 2),
            "total_requests": self.request_count,
            "total_errors": self.error_count,
            "active_websockets": self.websocket_connections,
            "windows_started": self.window_starts,
            "windows_killed": self.window_kills,
            "windows_crashed": self.window_crashes
        }


metrics = Metrics()


# ============================================================================
# Authentication
# ============================================================================

async def verify_api_key(x_api_key: Optional[str] = Header(None)):
    """Verify API key if configured"""
    if config.api_key:
        if not x_api_key:
            logger.warning("Missing API key in request")
            raise HTTPException(status_code=401, detail="API key required")
        if x_api_key != config.api_key:
            logger.warning("Invalid API key", key=x_api_key[:8] + "...")
            raise HTTPException(status_code=403, detail="Invalid API key")
    return True


# ============================================================================
# Global State
# ============================================================================

server = libtmux.Server()
websocket_clients: List[WebSocket] = []


# ============================================================================
# Request/Response Models
# ============================================================================

class StartWindowRequest(BaseModel):
    command: str = Field(..., min_length=1)
    delay: int = Field(default=1, ge=0, le=10)


class WindowResponse(BaseModel):
    name: str
    exists: bool
    active: bool = False
    index: Optional[int] = None
    has_process: bool = False  # True if process running, False if just shell


class WindowListResponse(BaseModel):
    session: str
    windows: List[WindowResponse]


class HealthResponse(BaseModel):
    status: str
    session: str
    session_exists: bool
    window_count: int
    uptime_seconds: float
    timestamp: str


class MetricsResponse(BaseModel):
    uptime_seconds: float
    total_requests: int
    total_errors: int
    active_websockets: int
    windows_started: int
    windows_killed: int
    windows_crashed: int


# ============================================================================
# Helper Functions with Error Recovery
# ============================================================================

def get_session(create_if_missing: bool = True):
    """
    Get or create tmux session with error recovery.
    Ensures exactly ONE session with config.session_name exists.
    """
    try:
        # Find all sessions with our name
        all_sessions = server.sessions
        matching_sessions = [s for s in all_sessions if s.session_name == config.session_name]
        
        if len(matching_sessions) > 1:
            # Multiple sessions exist - kill extras, keep first one
            logger.warning(
                "Multiple sessions found, cleaning up",
                count=len(matching_sessions),
                session=config.session_name
            )
            for extra_session in matching_sessions[1:]:
                try:
                    extra_session.kill_session()
                    logger.info("Killed duplicate session", session=extra_session.session_name)
                except Exception as e:
                    logger.error("Failed to kill duplicate session", error=str(e))
            
            return matching_sessions[0]
        
        elif len(matching_sessions) == 1:
            # Exactly one session exists - perfect!
            return matching_sessions[0]
        
        else:
            # No session exists
            if create_if_missing:
                logger.info("Creating new tmux session", session=config.session_name)
                return server.new_session(config.session_name, attach=False)
            else:
                return None
    
    except Exception as e:
        logger.error("Failed to get/create session", error=str(e))
        raise HTTPException(status_code=500, detail=f"Session error: {str(e)}")



def get_window_status(window_name: str) -> WindowResponse:
    """Get status of a specific window with error handling"""
    try:
        session = get_session(create_if_missing=False)
        if not session:
            return WindowResponse(name=window_name, exists=False)
        
        window = session.find_where({"window_name": window_name})
        if window:
            return WindowResponse(
                name=window_name,
                exists=True,
                active=window.window_active,
                index=window.window_index
            )
    except Exception as e:
        logger.error("Error getting window status", window=window_name, error=str(e))
    
    return WindowResponse(name=window_name, exists=False)


async def notify_clients(event: dict):
    """Send event to all connected WebSocket clients with error handling"""
    dead_clients = []
    for client in websocket_clients:
        try:
            await client.send_json(event)
        except Exception as e:
            logger.debug("WebSocket send failed", error=str(e))
            dead_clients.append(client)
    
    # Remove dead clients
    for client in dead_clients:
        if client in websocket_clients:
            websocket_clients.remove(client)
            metrics.record_websocket_disconnect()


async def monitor_windows():
    """
    Background task to monitor process crashes in tmux panes.
    
    IMPORTANT: Windows are NEVER auto-closed. We only detect when a process
    inside a pane has died (crashed), but the window/pane stays open.
    """
    known_window_states = {}  # {window_name: {"has_process": bool}}
    consecutive_errors = 0
    max_consecutive_errors = 10
    first_run = True  # Flag to detect initial state
    
    logger.info("Starting window monitor", interval=config.monitor_interval)
    
    while True:
        try:
            session = get_session(create_if_missing=False)
            if not session:
                logger.warning("Session not found, waiting...")
                await asyncio.sleep(config.monitor_interval * 2)
                continue
            
            current_window_states = {}
            
            # Check each window's pane for running processes
            for window in session.windows:
                window_name = window.window_name
                
                # Get the active pane in this window
                pane = window.attached_pane
                
                # Check if pane has a running process (not just shell)
                # tmux reports pane_pid which is the shell PID
                # We check pane_current_command to see what's actually running
                try:
                    current_command = pane.pane_current_command
                    # If only bash/zsh/sh is running, there's no actual process
                    has_process = current_command not in ['bash', 'zsh', 'sh', 'fish']
                    
                    current_window_states[window_name] = {
                        "has_process": has_process,
                        "command": current_command
                    }
                except Exception as e:
                    logger.debug("Error checking pane", window=window_name, error=str(e))
                    current_window_states[window_name] = {"has_process": False, "command": "unknown"}
            
            # On first run, report initial state of all windows
            if first_run:
                for window_name, state in current_window_states.items():
                    if state["has_process"]:
                        logger.info(
                            "Initial state: process running",
                            window=window_name,
                            command=state["command"]
                        )
                        await notify_clients({
                            "event": "window_started",
                            "window": window_name,
                            "command": state["command"],
                            "timestamp": datetime.now(UTC).isoformat()
                        })
                    else:
                        # Window exists but no process - already crashed
                        logger.warning(
                            "Initial state: window crashed (no process)",
                            window=window_name
                        )
                        await notify_clients({
                            "event": "window_crashed",
                            "window": window_name,
                            "timestamp": datetime.now(UTC).isoformat()
                        })
                first_run = False
            else:
                # Normal monitoring: detect state changes
                for window_name, current_state in current_window_states.items():
                    previous_state = known_window_states.get(window_name)
                    
                    if previous_state is None:
                        # New window detected
                        if current_state["has_process"]:
                            logger.info(
                                "Window started with process",
                                window=window_name,
                                command=current_state["command"]
                            )
                            await notify_clients({
                                "event": "window_started",
                                "window": window_name,
                                "command": current_state["command"],
                                "timestamp": datetime.now(UTC).isoformat()
                            })
                        else:
                            # New window but no process (unusual)
                            logger.warning("New window with no process", window=window_name)
                            await notify_clients({
                                "event": "window_crashed",
                                "window": window_name,
                                "timestamp": datetime.now(UTC).isoformat()
                            })
                    
                    elif previous_state["has_process"] and not current_state["has_process"]:
                        # Process died (crashed) - window still exists but process is gone
                        logger.warning(
                            "Process crashed in window",
                            window=window_name,
                            previous_command=previous_state["command"]
                        )
                        metrics.record_window_crash()
                        await notify_clients({
                            "event": "window_crashed",
                            "window": window_name,
                            "timestamp": datetime.now(UTC).isoformat()
                        })
                    
                    elif not previous_state["has_process"] and current_state["has_process"]:
                        # Process restarted in existing window
                        logger.info(
                            "Process restarted in window",
                            window=window_name,
                            command=current_state["command"]
                        )
                        await notify_clients({
                            "event": "window_restarted",
                            "window": window_name,
                            "command": current_state["command"],
                            "timestamp": datetime.now(UTC).isoformat()
                        })
                
                # Detect manually closed windows (rare, but possible)
                closed_windows = set(known_window_states.keys()) - set(current_window_states.keys())
                for window_name in closed_windows:
                    logger.info("Window manually closed", window=window_name)
                    await notify_clients({
                        "event": "window_closed",
                        "window": window_name,
                        "timestamp": datetime.now(UTC).isoformat()
                    })
            
            known_window_states = current_window_states
            consecutive_errors = 0  # Reset error counter on success
            
        except Exception as e:
            consecutive_errors += 1
            logger.error(
                "Monitor error",
                error=str(e),
                consecutive_errors=consecutive_errors
            )
            
            if consecutive_errors >= max_consecutive_errors:
                logger.error("Too many consecutive errors, monitor stopping")
                break
        
        await asyncio.sleep(config.monitor_interval)


# ============================================================================
# Lifespan Management
# ============================================================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context for background monitoring"""
    logger.info("Starting tmux API server", config=config.model_dump())
    
    # Validate tmux session exists or can be created
    try:
        session = get_session(create_if_missing=True)
        logger.info("tmux session ready", session=session.session_name)
    except Exception as e:
        logger.error("Failed to initialize session", error=str(e))
        raise
    
    # Start background monitor
    task = asyncio.create_task(monitor_windows())
    
    yield
    
    # Cleanup
    logger.info("Shutting down tmux API server")
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass


# ============================================================================
# FastAPI Application
# ============================================================================

app = FastAPI(
    title="tmux API Server",
    version="1.0.0",
    lifespan=lifespan
)



# ============================================================================
# API Endpoints
# ============================================================================

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Comprehensive health check"""
    try:
        session = get_session(create_if_missing=False)
        session_exists = session is not None
        window_count = len(session.windows) if session else 0
        
        return HealthResponse(
            status="healthy" if session_exists else "degraded",
            session=config.session_name,
            session_exists=session_exists,
            window_count=window_count,
            uptime_seconds=round(time.time() - metrics.start_time, 2),
            timestamp=datetime.now(UTC).isoformat()
        )
    except Exception as e:
        logger.error("Health check failed", error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/metrics", response_model=MetricsResponse)
async def get_metrics():
    """Get server metrics"""
    return MetricsResponse(**metrics.get_stats())


@app.get("/api/windows", response_model=WindowListResponse, dependencies=[Depends(verify_api_key)])
async def list_windows():
    """List all active windows in the session with process status"""
    metrics.record_request()
    try:
        session = get_session(create_if_missing=False)
        if not session:
            return WindowListResponse(session=config.session_name, windows=[])
        
        windows = []
        for w in session.windows:
            # Check if window has running process
            try:
                pane = w.attached_pane
                current_command = pane.pane_current_command
                has_process = current_command not in ['bash', 'zsh', 'sh', 'fish']
            except Exception:
                has_process = False
            
            windows.append(
                WindowResponse(
                    name=w.window_name,
                    exists=True,
                    active=w.window_active,
                    index=w.window_index,
                    has_process=has_process
                )
            )
        
        logger.debug("Listed windows", count=len(windows))
        return WindowListResponse(session=config.session_name, windows=windows)
    except Exception as e:
        metrics.record_error()
        logger.error("Failed to list windows", error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/windows/{window_name}/status", response_model=WindowResponse, dependencies=[Depends(verify_api_key)])
async def get_window(window_name: str):
    """Get status of a specific window"""
    metrics.record_request()
    logger.debug("Getting window status", window=window_name)
    return get_window_status(window_name)


@app.post("/api/windows/{window_name}/start", dependencies=[Depends(verify_api_key)])
async def start_window(window_name: str, request: StartWindowRequest):
    """Start a new window with the given command"""
    metrics.record_request()
    try:
        session = get_session(create_if_missing=True)
        
        # Check if window already exists
        existing = session.find_where({"window_name": window_name})
        if existing:
            logger.info("Window already exists", window=window_name)
            return JSONResponse(
                status_code=200,
                content={
                    "success": True,
                    "message": "Window already exists",
                    "window": window_name
                }
            )
        
        # Create new window
        logger.info("Starting window", window=window_name, command=request.command[:50])
        window = session.new_window(window_name=window_name, attach=False)
        
        # Send command with delay
        if request.delay > 0:
            await asyncio.sleep(request.delay)
        
        pane = window.attached_pane
        pane.send_keys(request.command)
        
        metrics.record_window_start()
        logger.info("Window started successfully", window=window_name)
        
        return JSONResponse(
            status_code=200,
            content={
                "success": True,
                "message": "Window started",
                "window": window_name
            }
        )
    except Exception as e:
        metrics.record_error()
        logger.error("Failed to start window", window=window_name, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@app.delete("/api/windows/{window_name}", dependencies=[Depends(verify_api_key)])
async def kill_window(window_name: str):
    """Kill a window"""
    metrics.record_request()
    try:
        session = get_session(create_if_missing=False)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")
        
        window = session.find_where({"window_name": window_name})
        
        if not window:
            logger.warning("Window not found for kill", window=window_name)
            raise HTTPException(status_code=404, detail="Window not found")
        
        logger.info("Killing window", window=window_name)
        window.kill_window()
        metrics.record_window_kill()
        
        return JSONResponse(
            status_code=200,
            content={
                "success": True,
                "message": "Window killed",
                "window": window_name
            }
        )
    except HTTPException:
        raise
    except Exception as e:
        metrics.record_error()
        logger.error("Failed to kill window", window=window_name, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@app.websocket("/api/events")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket endpoint for real-time window events"""
    await websocket.accept()
    websocket_clients.append(websocket)
    metrics.record_websocket_connect()
    
    logger.info("WebSocket connected", total_clients=len(websocket_clients))
    
    try:
        # Send initial state
        session = get_session(create_if_missing=False)
        windows = [w.window_name for w in session.windows] if session else []
        await websocket.send_json({
            "event": "connected",
            "windows": windows,
            "timestamp": datetime.now(UTC).isoformat()
        })
        
        # Keep connection alive
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        logger.info("WebSocket disconnected normally")
    except Exception as e:
        logger.error("WebSocket error", error=str(e))
    finally:
        if websocket in websocket_clients:
            websocket_clients.remove(websocket)
            metrics.record_websocket_disconnect()
        logger.info("WebSocket cleaned up", total_clients=len(websocket_clients))


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level=config.log_level.lower())
