# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import os
import sys
from logging.handlers import TimedRotatingFileHandler
from pathlib import Path

class Logger:
    def __init__(self, name, log_dir="/home/arm/logs/turtle2_controller", level=logging.DEBUG):
        Path(log_dir).mkdir(parents=True, exist_ok=True)

        self._formatter = logging.Formatter(
            fmt='[%(asctime)s][%(levelname)s][%(filename)s:%(lineno)d][%(funcName)s][%(threadName)s] %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        self._log_counters = {}

        if not self.logger.handlers:
            self._setup_handlers(log_dir, name)

    def _setup_handlers(self, log_dir, name):
        file_handler = TimedRotatingFileHandler(
            filename=os.path.join(log_dir, f'{name}.logging'),
            when='H',
            interval=1,
            backupCount=24*7,
            encoding='utf-8'
        )
        file_handler.suffix = "%Y%m%d_%H.log"
        file_handler.setFormatter(self._formatter)

        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(self._formatter)

        self.logger.addHandler(file_handler)
        self.logger.addHandler(console_handler)

    def debug(self, msg, *args, **kwargs):
        self.logger.debug(msg, *args, **kwargs, stacklevel=2)

    def info(self, msg, *args, **kwargs):
        self.logger.info(msg, *args, **kwargs, stacklevel=2)

    def warning(self, msg, *args, **kwargs):
        self.logger.warning(msg, *args, **kwargs, stacklevel=2)

    def error(self, msg, *args, **kwargs):
        self.logger.error(msg, *args, **kwargs, stacklevel=2)

    def critical(self, msg, *args, **kwargs):
        self.logger.critical(msg, *args, **kwargs, stacklevel=2)

    def debug_every_n(self, n, msg, *args, **kwargs):
        self._log_every_n("debug", n, msg, *args, **kwargs)

    def info_every_n(self, n, msg, *args, **kwargs):
        self._log_every_n("info", n, msg, *args, **kwargs)

    def warning_every_n(self, n, msg, *args, **kwargs):
        self._log_every_n("warning", n, msg, *args, **kwargs)

    def error_every_n(self, n, msg, *args, **kwargs):
        self._log_every_n("error", n, msg, *args, **kwargs)

    def critical_every_n(self, n, msg, *args, **kwargs):
        self._log_every_n("critical", n, msg, *args, **kwargs)

    def _log_every_n(self, level, n, msg, *args, **kwargs):
        key = f"{level}:{msg}"
        self._log_counters[key] = self._log_counters.get(key, 0) + 1

        if self._log_counters[key] % n == 1:
            getattr(self.logger, level)(msg, *args, **kwargs, stacklevel=3)


logger = Logger("robocontrol", level=logging.INFO)
