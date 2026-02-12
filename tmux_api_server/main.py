#!/usr/bin/env python3
"""
tmux API Server - FastAPI service for managing tmux sessions
Provides REST API for robot process control
"""

import asyncio
import logging
import os
import sys
import time
from typing import List, Optional
from contextlib import asynccontextmanager

from datetime import datetime, timezone

# Add compatibility for Python < 3.11 (where UTC was introduced to datetime)
try:
    from datetime import UTC
except ImportError:
    UTC = timezone.utc

from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field, field_validator
import libtmux


# ============================================================================
# Configuration with validation
# ============================================================================

class Config(BaseModel):
    """Server configuration with validation"""
    session_name: str = Field(default="spot_session", min_length=1)
    log_level: str = Field(default="INFO")
    
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
            log_level=os.getenv("LOG_LEVEL", "INFO")
        )


# Load and validate configuration
try:
    config = Config.from_env()
except Exception as e:
    print(f"Configuration error: {e}", file=sys.stderr)
    sys.exit(1)


# ============================================================================
# Logging
# ============================================================================

logging.basicConfig(
    level=getattr(logging, config.log_level),
    format='%(asctime)s | %(levelname)-8s | %(name)s | %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger("tmux-api")


# ============================================================================
# Metrics
# ============================================================================

class Metrics:
    """Simple metrics tracking"""
    
    def __init__(self):
        self.start_time = time.time()
        self.request_count = 0
        self.error_count = 0
        self.window_starts = 0
        self.window_kills = 0
    
    def record_request(self):
        self.request_count += 1
    
    def record_error(self):
        self.error_count += 1
    
    def record_window_start(self):
        self.window_starts += 1
    
    def record_window_kill(self):
        self.window_kills += 1
    
    def get_stats(self) -> dict:
        uptime = time.time() - self.start_time
        return {
            "uptime_seconds": round(uptime, 2),
            "total_requests": self.request_count,
            "total_errors": self.error_count,
            "windows_started": self.window_starts,
            "windows_killed": self.window_kills
        }


metrics = Metrics()





# ============================================================================
# Global State
# ============================================================================

server = libtmux.Server()


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
    has_process: bool = False


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
    windows_started: int
    windows_killed: int


# ============================================================================
# Helper Functions
# ============================================================================

SHELL_COMMANDS = {'bash', 'zsh', 'sh', 'fish', 'dash', 'tcsh', 'ksh', 'csh'}


def get_session(create_if_missing: bool = True):
    """Get or create tmux session"""
    try:
        matching = [s for s in server.sessions if s.session_name == config.session_name]
        
        if len(matching) > 1:
            logger.warning(f"Multiple sessions found ({len(matching)}), cleaning up")
            for extra in matching[1:]:
                extra.kill_session()
            return matching[0]
        
        if matching:
            return matching[0]
        
        if create_if_missing:
            logger.info(f"Creating new tmux session: {config.session_name}")
            return server.new_session(config.session_name, attach=False)
        
        return None
    except Exception as e:
        logger.error(f"Failed to get/create session: {e}")
        raise HTTPException(status_code=500, detail=f"Session error: {e}")


def get_window_status(window_name: str) -> WindowResponse:
    """Get status of a specific window"""
    try:
        session = get_session(create_if_missing=False)
        if not session:
            return WindowResponse(name=window_name, exists=False)
        
        window = session.find_where({"window_name": window_name})
        if window:
            try:
                pane = window.attached_pane
                has_process = pane.pane_current_command not in SHELL_COMMANDS
            except Exception:
                has_process = False
            
            return WindowResponse(
                name=window_name,
                exists=True,
                active=window.window_active,
                index=window.window_index,
                has_process=has_process
            )
    except Exception as e:
        logger.error(f"Error getting window status for {window_name}: {e}")
    
    return WindowResponse(name=window_name, exists=False)


# ============================================================================
# Lifespan
# ============================================================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan"""
    logger.info(f"Starting tmux API server (session: {config.session_name})")
    
    try:
        session = get_session(create_if_missing=True)
        logger.info(f"tmux session ready: {session.session_name}")
    except Exception as e:
        logger.error(f"Failed to initialize session: {e}")
        raise
    
    yield
    
    logger.info("Shutting down tmux API server")


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
    """Health check endpoint"""
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
        logger.error(f"Health check failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/metrics", response_model=MetricsResponse)
async def get_metrics():
    """Get server metrics"""
    return MetricsResponse(**metrics.get_stats())


@app.get("/api/windows", response_model=WindowListResponse)
async def list_windows():
    """List all windows in the session"""
    metrics.record_request()
    try:
        session = get_session(create_if_missing=False)
        if not session:
            return WindowListResponse(session=config.session_name, windows=[])
        
        windows = []
        for w in session.windows:
            try:
                pane = w.attached_pane
                has_process = pane.pane_current_command not in SHELL_COMMANDS
            except Exception:
                has_process = False
            
            windows.append(WindowResponse(
                name=w.window_name,
                exists=True,
                active=w.window_active,
                index=w.window_index,
                has_process=has_process
            ))
        
        logger.debug(f"Listed {len(windows)} windows")
        return WindowListResponse(session=config.session_name, windows=windows)
    except Exception as e:
        metrics.record_error()
        logger.error(f"Failed to list windows: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/windows/{window_name}/status", response_model=WindowResponse)
async def get_window(window_name: str):
    """Get status of a specific window"""
    metrics.record_request()
    logger.debug(f"Getting window status: {window_name}")
    return get_window_status(window_name)


@app.post("/api/windows/{window_name}/start")
async def start_window(window_name: str, request: StartWindowRequest):
    """Start a new window with the given command"""
    metrics.record_request()
    try:
        session = get_session(create_if_missing=True)
        
        existing = session.find_where({"window_name": window_name})
        if existing:
            logger.info(f"Window already exists: {window_name}")
            return JSONResponse(
                status_code=200,
                content={"success": True, "message": "Window already exists", "window": window_name}
            )
        
        logger.info(f"Starting window: {window_name}")
        window = session.new_window(window_name=window_name, attach=False)
        
        if request.delay > 0:
            await asyncio.sleep(request.delay)
        
        pane = window.attached_pane
        pane.send_keys(request.command)
        
        metrics.record_window_start()
        logger.info(f"Window started: {window_name}")
        
        return JSONResponse(
            status_code=200,
            content={"success": True, "message": "Window started", "window": window_name}
        )
    except Exception as e:
        metrics.record_error()
        logger.error(f"Failed to start window {window_name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.delete("/api/windows/{window_name}")
async def kill_window(window_name: str):
    """Kill a window"""
    metrics.record_request()
    try:
        session = get_session(create_if_missing=False)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")
        
        window = session.find_where({"window_name": window_name})
        if not window:
            logger.warning(f"Window not found: {window_name}")
            raise HTTPException(status_code=404, detail="Window not found")
        
        logger.info(f"Killing window: {window_name}")
        window.kill_window()
        metrics.record_window_kill()
        
        return JSONResponse(
            status_code=200,
            content={"success": True, "message": "Window killed", "window": window_name}
        )
    except HTTPException:
        raise
    except Exception as e:
        metrics.record_error()
        logger.error(f"Failed to kill window {window_name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/windows/{window_name}/select")
async def select_window(window_name: str):
    """Select/focus a window"""
    metrics.record_request()
    try:
        session = get_session(create_if_missing=False)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found")
        
        window = session.find_where({"window_name": window_name})
        if not window:
            logger.warning(f"Window not found for selection: {window_name}")
            raise HTTPException(status_code=404, detail="Window not found")
        
        logger.info(f"Selecting window: {window_name}")
        window.select_window()
        
        return JSONResponse(
            status_code=200,
            content={"success": True, "message": "Window selected", "window": window_name}
        )
    except HTTPException:
        raise
    except Exception as e:
        metrics.record_error()
        logger.error(f"Failed to select window {window_name}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level=config.log_level.lower())
