# logging_utils.py
"""
Logging setup using Python's built-in logging with rotation.
"""
import logging
from logging.handlers import RotatingFileHandler
import config

def setup_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = RotatingFileHandler(
            config.LOG_FILE,
            maxBytes=config.LOG_MAX_BYTES,
            backupCount=config.LOG_BACKUP_COUNT
        )
        fmt = logging.Formatter(
            '%(asctime)s [%(levelname)s] %(name)s: %(message)s'
        )
        handler.setFormatter(fmt)
        logger.setLevel(logging.DEBUG)
        logger.addHandler(handler)