from threading import Event, Thread, Timer

def run(block, delay=0, *args):
    """ Run `block` with `args` after `delay` seconds, non-blocking """
    t = Timer(delay, lambda: block(*args))
    t.daemon = True
    t.start()
    return t

def loop(block, interval=0, daemon=True, *args):
    """
    Repeat `block` with `args` parameters every `interval` seconds, non-blocking,
    returns a Cancellable (when executed future executions are cancelled)
    """
    e = Event()
    def do():
        while not e.wait(interval): # the first call is in `interval` secs
            block(*args)
    t = Thread(target=do)
    t.setDaemon(daemon)
    t.start()
    return Cancellable(e)

class Cancellable(object):

    def __init__(self, event):
        self.event = event
    
    def cancel(self):
        self.event.set()

    def is_cancelled(self):
        return self.event.isSet()

    def __call__(self):
        self.cancel()