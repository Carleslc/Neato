import sys, tty, termios

from async import loop

def getch():
    """ Wait for key input and returns a character representation, blocking """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def key_listener(key, block, *args):
    """
    Run `block` with `args` when `key` is pressed, non-blocking,
    returns a Cancellable (when executed future executions are cancelled)
    """
    def listen():
        if getch() == key: # WARNING: getch malforms output if used asynchronously
            block(*args)
    return loop(listen)

def input_listener(text, block, *args):
    """
    Run `block` with `args` when `text` + Enter is pressed, non-blocking,
    returns a Cancellable (when executed future executions are cancelled)
    """
    def listen():
        if raw_input() == text:
            block(*args)
    return loop(listen)