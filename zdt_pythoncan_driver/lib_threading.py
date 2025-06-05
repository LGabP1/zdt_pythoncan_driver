"""
Threading utility allowing to launch threads safely.
"""
import threading
from typing import Callable, Optional, Any

class Threader:
    """Class wrapping safe thread launch and handling."""

    def __init__(self,  do_finally: Optional[Callable] = None):
        """Class wrapping safe thread launch and handling.
        
        Args:
            do_finally (Callable, optional): 
        """
        self._lock = threading.Lock()
        self._running = False
        self._do_finally = do_finally

    def get_lock(self) -> threading.Lock:
        return self._lock

    def start_threaded(self, target: Callable, **args: Any):
        """Start threaded execution.
        
        Args:
            target (Callable): Callable to execute as threaded
            **args (Any): kwards arguments passed to the target callable"""
        if self._running is False:
            with self._lock:
                self._running = True
            self._threader(target, **args)

    def _threader(self, target: Callable, **args: Any) -> None:  # create a thread
        """Launch thread."""
        thr = threading.Thread(target=self._safe_thread, kwargs={"target": target, **args})
        thr.daemon = True
        thr.start()

    def _safe_thread(self, target: Callable, **args: Any) -> None:  # thread wrapper
        """Thread wrapper to handle exceptions."""
        try:
            target(**args)
        except KeyboardInterrupt:
            try:
                with self._lock:
                    self._running = False
            except Exception:
                pass
            pass
        except Exception as e:
            with self._lock:
                self._running = False
            raise e from e
        finally:
            if self._do_finally:
                self._do_finally()
    
    def stop(self) -> None:
        """Stop threaded execution."""
        with self._lock:
            self._running = False