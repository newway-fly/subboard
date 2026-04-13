# ============================================================
# Logging system
# All logs are buffered through a queue, then sent through USB
# Prevents print from blocking the main loop
# ============================================================

"""
log_system.py

Simple logging module (can be extended to support more log levels
or output to the USB file system).

In a MicroPython environment keep it as lightweight as possible.
"""

import sys

INFO = "INFO"
WARN = "WARN"
ERROR = "ERROR"
DEBUG = "DEBUG"

def log(level, *args):

    try:
        msg = " ".join(str(a) for a in args if not isinstance(a, BaseException))
        print("[{}] {}".format(level, msg))

        for a in args:
            if isinstance(a, BaseException):
                sys.print_exception(a)

    except Exception as e:
        print("[LOG ERROR]", e)
