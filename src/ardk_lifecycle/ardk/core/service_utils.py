"""
Robust service call utilities for ARDK.

Provides deterministic wait patterns that don't rely on node.executor attribute.
Assumes executor thread is already spinning (FastAPI background thread pattern).

Key design principles:
- Separate service discovery timeout from call execution timeout
- Retries with exponential backoff for transient failures
- Timeouts raise TimeoutError for fault signaling, not silent failures
"""
import time
from typing import Any, Optional
from rclpy.node import Node


# =============================================================================
# Default Timeouts (Pi-friendly)
# =============================================================================

class Timeouts:
    """Configurable timeout defaults. Override for your hardware."""
    
    # Service discovery (boot/bringup can be slow)
    DISCOVERY_BOOT = 120.0       # During initial boot
    DISCOVERY_NORMAL = 30.0     # Normal operations
    
    # Execution timeouts
    LIFECYCLE_STARTUP = 90.0    # costmaps/plugins can be slow
    LIFECYCLE_SHUTDOWN = 30.0
    LOAD_MAP = 15.0
    SAVE_MAP = 30.0
    
    # Retry config
    DEFAULT_RETRIES = 2
    BACKOFF_BASE = 0.5  # seconds


# =============================================================================
# Core Utilities
# =============================================================================

def wait_future(fut, timeout_sec: float, poll_sec: float = 0.05) -> None:
    """
    Poll-based wait for a future with hard deadline.
    
    Does NOT check node.executor - assumes executor is spinning.
    Raises TimeoutError if deadline exceeded.
    
    Args:
        fut: The Future to wait on
        timeout_sec: Maximum time to wait
        poll_sec: Polling interval (default 50ms)
    """
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if fut.done():
            return
        time.sleep(poll_sec)
    raise TimeoutError(f"Future timed out after {timeout_sec:.1f}s")


def call_service(
    node: Node,
    client,
    req,
    *,
    wait_service_sec: float = Timeouts.DISCOVERY_NORMAL,
    call_timeout_sec: float = 30.0,
    retries: int = Timeouts.DEFAULT_RETRIES,
    backoff_base: float = Timeouts.BACKOFF_BASE,
    service_name: str = "service"
) -> Any:
    """
    Robust service call with discovery timeout, execution timeout, and retries.
    
    Args:
        node: ROS node for logging
        client: Service client (already created)
        req: Service request object
        wait_service_sec: Timeout for service discovery (can be long on boot)
        call_timeout_sec: Timeout for the actual call execution
        retries: Number of retry attempts (default 2)
        backoff_base: Base delay for exponential backoff (default 0.5s)
        service_name: Name for logging
    
    Returns:
        Service response
        
    Raises:
        TimeoutError: If service not available or call times out after retries
        RuntimeError: If service returns failure response
    """
    # Phase 1: Wait for service to exist
    if not client.wait_for_service(timeout_sec=wait_service_sec):
        raise TimeoutError(f"Service '{service_name}' not available after {wait_service_sec:.1f}s")
    
    # Phase 2: Call with retries
    last_exc: Optional[Exception] = None
    for attempt in range(retries + 1):
        if attempt > 0:
            backoff = backoff_base * (2 ** (attempt - 1))
            node.get_logger().info(f"Retry {attempt}/{retries} for {service_name} (backoff {backoff:.1f}s)")
            time.sleep(backoff)
        
        try:
            fut = client.call_async(req)
            wait_future(fut, call_timeout_sec)
            
            if fut.exception():
                raise fut.exception()
            
            return fut.result()
            
        except TimeoutError as e:
            last_exc = e
            node.get_logger().warn(f"Call to {service_name} timed out (attempt {attempt + 1}/{retries + 1})")
        except Exception as e:
            last_exc = e
            node.get_logger().warn(f"Call to {service_name} failed: {e} (attempt {attempt + 1}/{retries + 1})")
    
    # All retries exhausted
    raise TimeoutError(f"Service '{service_name}' failed after {retries + 1} attempts: {last_exc}")


def call_lifecycle_service(
    node: Node,
    client,
    req,
    *,
    service_name: str,
    is_startup: bool = True
) -> Any:
    """
    Convenience wrapper for lifecycle manager calls with appropriate timeouts.
    
    Args:
        node: ROS node
        client: Service client
        req: Request object
        service_name: For logging
        is_startup: True for STARTUP (longer timeout), False for SHUTDOWN
    """
    if is_startup:
        exec_timeout = Timeouts.LIFECYCLE_STARTUP
        discovery_timeout = Timeouts.DISCOVERY_BOOT
    else:
        exec_timeout = Timeouts.LIFECYCLE_SHUTDOWN
        discovery_timeout = Timeouts.DISCOVERY_NORMAL
    
    resp = call_service(
        node, client, req,
        wait_service_sec=discovery_timeout,
        call_timeout_sec=exec_timeout,
        service_name=service_name
    )
    
    if hasattr(resp, 'success') and not resp.success:
        raise RuntimeError(f"{service_name} returned success=False")
    
    return resp
