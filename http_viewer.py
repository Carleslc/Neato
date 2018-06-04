from multiprocessing import Process, Queue
import threading
import http_server
import SocketServer
import time
import random
import sys

class HttpViewer(object):
    """ Class that process the and show the points received from Neato. """

    def __init__(self, port, laser_queue, pose_queue):
        """
            Constructor, port where the web will be served
            laser_queue is the queue where the laser points has to be queued.
            pose_queue is the queue where the pose point has to be queued.
        """
        self.port = port
        self.exit = False

        self.mm_per_pixel = 20
        self.laser_queue = laser_queue
        self.pose_queue = pose_queue

        self.process = Process(target=self.run)
        self.process.start()


    def main_http_server(self):
        """ Start point of the thread that manages the http server. """
        Handler = http_server.HttpServerHandler
        self.httpd = SocketServer.TCPServer(("", self.port), Handler)
             
        self.httpd.serve_forever()


    def main_laser(self):
        """ Start point of the thread that gets the laser points """
        print "Laser thread"
        laser_points = []
        while True:
            new_points = self.laser_queue.get()
            laser_points.extend(new_points)

            print("LASER POINTS:", laser_points)

            if len(laser_points) > 1000:
                laser_points = laser_points[360:]
                print "Laser len:", len(laser_points)    

            write_points_json(laser_points, self.mm_per_pixel, "laserPoints", "laser.json")
        

    def main_pose(self):
        """ Start point of the thread that gets the pose points. """
        print "Pose thread"
        pose_points = []
        while True:
            new_points = self.pose_queue.get()
            pose_points.extend(new_points)

            if len(pose_points) > 5000:
                pose_points = pose_points[360:]

            write_points_json(pose_points, self.mm_per_pixel, "pose", "pose.json")


    def run(self):
        """ Method that start all the threads. """
        sys.stdout = open('http_log.txt', 'w')
        self.thread = threading.Thread(target=self.main_http_server)
        self.thread.start()
        
        self.thread_laser = threading.Thread(target=self.main_laser)
        self.thread_laser.start()
        
        self.thread_pose = threading.Thread(target=self.main_pose)
        self.thread_pose.start()
        
        while not self.exit:
            time.sleep(0.4)

    def quit(self):
        """ Stops execution of the web server. """
        self.exit = True
        if hasattr(self,'httpd'):
            self.httpd.shutdown()
        self.process.terminate()

def point_to_json(point, mm_per_pixel):
    print("POINT:", point)
    """ Returns a string with the point formated in jSON. """
    return '{ "x": ' + str(point[1] / mm_per_pixel) + ', "y": ' + str(point[0] / mm_per_pixel) + '}'

def write_points_json(points, mm_per_pixel, type, filename):
    """ Write a list of points in a jSON structured file. """

    json_string = '{\n "'+type+'": ['
    for point in points:
	print("Point:", point)
        json_string += point_to_json(point, mm_per_pixel) + ',\n'
    json_string = json_string[:-2]
    json_string += ']\n}'

    with open(filename, 'w') as file_camino:
        file_camino.write(json_string)

if __name__ == "__main__":
    """ Test program. """
    def laser_test():
        laser_points = []

        for i in range(360):
            x = random.randint(-400, 400)
            y = random.randint(-400, 400)
            laser_points.append((x,y))

        return laser_points

    def pose_test():
        x = random.randint(-400, 400)
        y = random.randint(-400, 400)
        
        return [(x,y)]

    laser_queue = Queue()
    pose_queue = Queue()
    viewer = HttpViewer(8001, laser_queue, pose_queue)

    time.sleep(1)

    n_laser = 0
    n_pose = 0

    while True:
        laser_queue.put(laser_test())
        pose_queue.put(pose_test())
        time.sleep(10.5)

        n_laser += 360
        n_pose += 1




