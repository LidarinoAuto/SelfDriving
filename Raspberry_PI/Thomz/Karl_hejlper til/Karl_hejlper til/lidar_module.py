# lidar_module.py
"""
Thread-safe LIDAR sensor reader using RPLidar and a rolling buffer.
"""
import threading
import collections
import time
from rplidar import RPLidar
from logging_utils import setup_logger
import config

class LidarModule:
    def __init__(self):
        self._port = config.LIDAR_PORT
        self._baud = config.LIDAR_BAUDRATE
        self._buffer = collections.deque(maxlen=config.LIDAR_BUFFER_SIZE)
        self._lock = threading.Lock()
        self._running = False
        self.logger = setup_logger(self.__class__.__name__)

    def start(self):
        if not self._running:
            self._running = True
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
            self.logger.info("LIDAR thread started")

    def stop(self):
        self._running = False
        if hasattr(self, '_thread'):
            self._thread.join()
            self.logger.info("LIDAR thread stopped")

    def _run(self):
        try:
            with RPLidar(self._port, baudrate=self._baud) as lidar:
                for scan in lidar.iter_scans():
                    if not self._running:
                        break
                    distances = [d[2] for d in scan]
                    avg = sum(distances) / len(distances)
                    with self._lock:
                        self._buffer.append(avg)
        except Exception:
            self.logger.exception("Error in LIDAR thread")
        finally:
            self._running = False

    def get_distance(self) -> float:
        """Return the average of recent LIDAR scans in mm."""
        with self._lock:
            if not self._buffer:
                return float('nan')
            return sum(self._buffer) / len(self._buffer)