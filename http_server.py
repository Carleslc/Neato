import SimpleHTTPServer

class HttpServerHandler(SimpleHTTPServer.SimpleHTTPRequestHandler):
    """ Class that overwrites the log_message method of SimpleHTTPRequestHandler in order to don't output to the console. """    
    def log_message(self, format, *args):
        pass