import sys

DEBUG = 0
INFO = 1
NONE = 2

LEVEL = DEBUG

def log_level(level):
    global LEVEL
    LEVEL = level

def log(level, message, delay=0):
    global LEVEL
    if LEVEL > level:
        return
    def print_message():
        _print(message)
    if delay == 0:
        print_message()
    else:
        run(print_message, delay)

def debug(message, delay=0):
    log(DEBUG, message, delay)

def info(message, delay=0):
    log(INFO, message, delay)

def _print(s): # better print for asynchronous use
    sys.stdout.write(str(s) + '\r\n')
    sys.stdout.flush()