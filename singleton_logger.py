import logging
import sys
from logging import StreamHandler, Formatter

class SingletonLogger:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, name="Nuclear Robo Logger", level=logging.DEBUG):
        if not self._initialized:
            self.logger = logging.getLogger(name)
            self.logger.setLevel(level)

            formatter = Formatter(
                "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S"
            )

            console_handler = StreamHandler(sys.stdout)
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)

            self._initialized = True

    def add_custom_stream(self, custom_stream):
        if not self._initialized:
            raise RuntimeError("Logger not initialized!")

        formatter = Formatter(
            "%(asctime)s - %(name)s - %(levelname)s - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )

        custom_handler = StreamHandler(custom_stream)
        custom_handler.setFormatter(formatter)
        self.logger.addHandler(custom_handler)

    def info(self, msg):
        self.logger.info(msg)

    def error(self, msg):
        self.logger.error(msg)

    def warning(self, msg):
        self.logger.warning(msg)

    def debug(self, msg):
        self.logger.debug(msg)

    def critical(self, msg):
        self.logger.critical(msg)