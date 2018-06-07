# -*- coding: utf-8 -*-

import sys
import time

import serial as serial_port

from log import *
from Laser import *
from async import run
from utils import *
from Odometry import Odometry
from math import sin, cos, atan, pi, sqrt, degrees, radians, atan2
from sets import Set
from multiprocessing import Queue
import http_viewer
from numpy import dot, transpose

GO_SUPER_FAST = 0

## NEATO ##

class NeatoMock(object):
    """
    Neato for testing purposes, does not works in real time but works as a Neato replacement
    when you does not have connection to the Neato robot
    """

    def __init__(self, laser=False, viewer=False, speed=100, delta_d=0, delta_th=0, theta_dot=0.0, x_ini=0, y_ini=0, noise_d=0.0001, noise_th=0.000001):
        _Neato_init(self, serial=False, viewer=viewer, default_speed=speed, laser=laser, odometry=Odometry(delta_d=delta_d, delta_th=delta_th, theta_dot=theta_dot, x_ini=x_ini, y_ini=y_ini, noise_d=noise_d, noise_th=noise_th))

    def is_mocked(self):
        return True

    def close(self):
        _Neato_close(self)

    def send(self, message, delay=0.1, show_time=False):
        self.sleep(delay)
        debug("Mocked " + message)

    def get_laser(self, configuration=priorityConfiguration, delay=0.5):
        self.send('GetLDSScan', delay=delay)
        if delay > 0:
            self.sleep(delay)
        return random_laser(configuration)

    def get_motors(self):
        self.send('GetMotors LeftWheel RightWheel')
        left, right = self._accL, self._accR
        if self.get_speed() != 0:
            ttc = self.time_to_complete()
            if ttc > 0:
                diffT = time.time() - self._start_motors
                debug("Elapsed seconds from start: %.2f" % diffT)
                progress = min(diffT, ttc) / ttc
                debug("Progress: %.2f" % progress)
                left = left + progress*self._L
                right = right + progress*self._R
        return left, right

    def get_speed(self):
        return self.odometry.speed

    def time_to_complete(self):
        speed = self.get_speed()
        return self._d / speed if speed != 0 else 0

    def set_motors(self, left, right, speed=None):
        if self.is_mocked():
            self.stop(reset_motors=False)
        if speed == None:
            speed = self.saved_speed
        self.odometry.speed = speed
        self._L = left
        self._R = right
        self._d = float(max(abs(left), abs(right)))
        delay = 0.1
        self._start_motors = time.time() + delay
        self.send('SetMotor LWheelDist %i RWheelDist %i Speed %i' % (left, right, speed), delay=delay)
        def resetLR():
            #debug('Finished Movement')
            if self.is_mocked():
                self.stop(reset_motors=False)
            elif hasattr(self, '_resetLR_timer'):
                self._resetLR_timer.cancel()
        self._resetLR_timer = run(resetLR, delay=max(self.time_to_complete() - delay, 0))

    def stop(self, reset_motors=True):
        if not self.is_stopped():
            #debug("Stop Update Odometry")
            if hasattr(self, '_resetLR_timer'):
                self._resetLR_timer.cancel()
            if reset_motors:
                self.enable_motors(False)
                self.enable_motors(True)
            self.update_odometry()
            self.odometry.theta_dot = 0.0
            self.odometry.speed = 0
            self._L = 0
            self._R = 0

    def is_stopped(self):
        return self.get_speed() == 0

    def rotation_motors(self, alfa, optimize=True, limit=0.001):
        s = sign(alfa) # 1: left (default), -1: right
        if alfa < 0:
            alfa = -alfa
        if optimize:
            alfa = alfa % 360
            if alfa > 180:
                alfa = 360 - alfa
                s = -s
        if is_zero(alfa, limit):
            return 0, 0
        info("Rotate %.2fº %s" % (alfa, 'LEFT' if s == 1 else 'RIGHT'))
        turn_dist = radians(alfa)*Neato.S
        return -s*turn_dist, s*turn_dist

    def rotate(self, alfa, optimize=True, limit=0.001):
        L, R = self.rotation_motors(alfa, optimize, limit)
        rotate = abs(L) > 0 and abs(R) > 0
        if rotate:
            self.set_motors(L, R)
            self.sleep(abs(L)/self.saved_speed)
            #debug("Sleep time: %i" % (abs(L)/self.saved_speed))
            #self.show_odometry("AFTER ROTATION")
        self.show_odometry()
        return rotate

    def predator_rotate(self, alfa, optimize=True, limit=0.001):
        L, R = self.rotation_motors(alfa, optimize, limit)
        rotate = abs(L) > 0 and abs(R) > 0
        if rotate:
            self.set_motors(L, R)
            self.sleep(abs(L)/300)
        return rotate

    def rotate_left(self, alfa, optimize=True, limit=0.001):
        return self.rotate(alfa, optimize, limit)

    def rotate_right(self, alfa, optimize=True, limit=0.001):
        return self.rotate(-alfa, optimize, limit)

    def move_forwards(self, d):
        info("Move %.2f mm" % d)
        self.set_motors(d, d)
        self.sleep(abs(d)/self.saved_speed)
        #self.show_odometry("AFTER MOVE")
        self.show_odometry()

    def move_backwards(self, d):
        self.rotate(180)
        self.move_forwards(-d)

    def move(self, *moves):
        for move in moves:
            move.apply(self)

    def orientate(self, laser, sector, limit=5):
        info("Orientate to %s" % sector.tag)
        self.rotate(sector.center(), limit)
        dist = laser[sector.tag].original_dist
        if not is_zero(dist):
            self.move_forwards(dist)

    def offset_from(self, alfa):
        return abs_alfa(self.alfa() - abs_alfa(alfa))

    def set_alfa(self, alfa, limit=0.001):
        alfa = -self.offset_from(alfa)
        debug("set_alfa's Alfa: %.2f" % alfa)
        return self.rotate(alfa, optimize=True, limit=limit)

    def alfa(self):
        return degrees(self.odometry.suma_theta)

    def alfa_optimized(self):
        alfa = self.alfa()
        return alfa if alfa <= 180 else 360 - alfa

    def show_odometry(self, msg=''):
        self.update_odometry()
        self.odometry.show(msg)

    def update_odometry(self):
        while self._updating:
            self.sleep(0.2)
        self._updating = True
        L, R = self.get_motors()
        self.odometry.update(L, R, self.get_speed())
        self._accL = self._accL + self.odometry.diffL
        self._accR = self._accR + self.odometry.diffR
        #debug("(AccL, AccR) = (%i, %i)" % (self._accL, self._accR))
        self._updating = False

    def beep(self, sound=1):
        self.send('PlaySound %i' % sound, 0.3)

    def enable_motors(self, enable=True):
        enabled = "Enable" if enable else "Disable"
        self.send('SetMotor RWheel%s LWheel%s' % (enabled, enabled), 0.2)
        if not enable:
            self.odometry.speed = 0

    def enable_laser(self, enable=True):
        if enable != self.enabled_laser:
            enabled = "On" if enable else "Off"
            self.send('SetLDSRotation ' + enabled, 0.2)
            self.enabled_laser = enable

    def test_mode(self, enable=True):
        enabled = "On" if enable else "Off"
        self.send('TestMode ' + enabled, 0.2)

    def run(self, block):
        """ Execute `block` until Ctrl^C is pressed """
        info("Press Ctrl^C to exit\n")
        def loop():
            while True:
                block()
        _close_on_ctrl_C(self, loop)

    def run_until_key(self, stop_key, block):
        """ Execute `block` until `stop_key` is pressed """
        from keyboard import getch
        info("Press %s to exit\n" % stop_key)
        while True:
            key = getch()
            if key == stop_key:
                self.close()
                return
            block(key)
    
    def run_once(self, block):
        """ Execute `block` once, closing neato if Ctrl^C is pressed """
        _close_on_ctrl_C(self, block)

    def viewer_start(self, port_web_server):
        self.laser_queue = Queue()
        self.odometry_queue = Queue()
        self.viewer = http_viewer.HttpViewer(port_web_server, self.laser_queue, self.odometry_queue)
        debug("Started Http Viewer at port %i" % port_web_server)

    def show_laser_viewer(self):
        laser_set = Set()
        laser = self.get_laser(commonConfiguration)
        def polar_to_cart(laser):
            cart = []
            for ray in laser.all(): #Polar 2 Cartesian in Robot Reference Frame
                if ray.is_error() == 0:
                    angulo = radians(ray.alfa)
                    x_r = int(ray.dist)*cos(angulo) 
                    y_r = int(ray.dist)*sin(angulo)
                    cart.append([x_r, y_r, 1])
            return cart

        def cart_to_world(coord):
            x = -self.odometry.x
            y = -self.odometry.y
            theta = self.odometry.suma_theta
            TAB = [[cos(theta),-sin(theta), x], [sin(theta), cos(theta), y], [0, 0, 1]]
            world = dot(TAB,transpose(coord))
            return transpose(world)

        values = polar_to_cart(laser)
        if len(values) > 0:
            values = cart_to_world(values)
            for i in values:
                        tuple = (int(i[0]), int(i[1]))
                        if tuple not in laser_set:
                            self.laser_queue.put([(i[0], i[1]), (100,100)])
                            laser_set.add(tuple)


class Neato(NeatoMock):
    """
    Subclass of NeatoMock, working on the real time Neato robot
    """

    S = 121.5; # millimeters

    def __init__(self, laser=False, viewer=False, speed=100, delta_d=0, delta_th=0, theta_dot=0.0, x_ini=0, y_ini=0, noise_d=0.0001, noise_th=0.000001):
        _Neato_init(self, serial=True, viewer=viewer, default_speed=speed, laser=laser, odometry=Odometry(delta_d=delta_d, delta_th=delta_th, theta_dot=theta_dot, x_ini=x_ini, y_ini=y_ini, noise_d=noise_d, noise_th=noise_th))
    
    def send(self, message, delay=0.1):
        return _envia(self.ser, message, delay, sleep=self.sleep)

    def get_laser(self, configuration=priorityConfiguration):
        """ Read the current values from the laser """
        return Laser(self.send('GetLDSScan').split('\r\n')[2:362], configuration)

    def get_motors(self):
        """ Ask to the robot for the current state of the motors. """
        msg = self.send('GetMotors LeftWheel RightWheel').split('\n')
        L = int(msg[4].split(',')[1])
        R = int(msg[8].split(',')[1])
        return L, R

    def is_mocked(self):
        return False

## MOVEMENT ##

class Movement: pass

class Wait(Movement):

    def __init__(self, delay):
        self.delay = delay # seconds

    def apply(self, neato):
        info("Wait %i seconds" % self.delay)
        neato.stop()
        neato.sleep(self.delay)

class Move(Movement):

    def __init__(self, distance):
        self.distance = distance # millimeters

    def apply(self, neato):
        neato.move_forwards(self.distance)

class Rotate(Movement):

    def __init__(self, alfa, optimize=True):
        self.alfa = alfa # degrees
        self.optimize = optimize

    def apply(self, neato):
        neato.rotate(self.alfa, self.optimize)

class Manhattan(Movement):

    def __init__(self, x, y):
        self.moves = []
        if x != 0:
            self.moves.append(Move(x))
        self.moves.append(Rotate(90))
        if y != 0:
            self.moves.append(Move(y))

    def apply(self, neato):
        neato.move(*self.moves)

class Euclide(Movement):

    def __init__(self, alfa, dist, optimize=True):
        self.moves = []
        if not is_zero(alfa):
            self.moves.append(Rotate(alfa, optimize))
        if not is_zero(dist):
            self.moves.append(Move(dist))

    def apply(self, neato):
        neato.move(*self.moves)

class EuclideToPosition(Movement):

    def __init__(self, x, y, optimize=True):
        # (x, y) regarding origin
        self.x = float(x)
        self.y = float(y)
        self.optimize = optimize

    def apply(self, neato):
        neato.stop()
        info("Go to position (x, y) = (%i, %i)" % (self.x, self.y))
        alfa = 0.0
        # (x, y) regarding neato position
        x = self.x - neato.odometry.x
        y = self.y - neato.odometry.y
        if not is_zero(x):
            alfa = degrees(atan2(y,x))
            #alfa = sign(alfa)*(90 - abs(alfa))
        #if not is_zero(x) and x < 0: # backwards
            #alfa = alfa + 180.0 # set forwards
        #alfa = mod(alfa - neato.alfa()) # coordinates offset
        alfa = alfa - neato.alfa()

        #neato.set_alfa(0)
        debug("now setting this Alfa: %.2f" % alfa)
        #neato.rotate(-neato.alfa)
        Euclide(alfa, sqrt(x**2 + y**2), self.optimize).apply(neato)

class Pose(Movement):

    def __init__(self, x, y, alfa, optimize=True):
        # (x, y, alfa) regarding origin
        self.position_move = EuclideToPosition(x, y, optimize)
        self.optimize = optimize
        self.alfa = alfa

    def apply(self, neato):
        self.position_move.apply(neato)
        info("Orientate to %.2f" % self.alfa)
        #neato.set_alfa(self.alfa)
        neato.rotate(self.alfa - neato.alfa())

class SecurePose(Movement):

    def __init__(self, x, y, alfa, k=100, v=0.5, optimize=True):
        # (x, y, alfa) regarding origin
        # k is the distance for a secure 'pass-through' point parallel to (x, y)
        # v is the speed reduction in percentage from the secure point to (x, y)
        secure_x = k*cos(radians(alfa)) + x
        secure_y = k*sin(radians(alfa)) + y
        self.secure_pose = Pose(secure_x, secure_y, alfa, optimize)
        self.v = v
        self.k = k
    
    def apply(self, neato):
        self.secure_pose.apply(neato)
        original_speed = neato.saved_speed
        neato.saved_speed = (1 - self.v)*neato.saved_speed

        laser = neato.get_laser(commonConfiguration)

        laser.summary(empties=True)

        laser_vec = np.array([laser.front_outer_left,laser.front_center_left,laser.front_center,laser.front_center_right,laser.front_outer_right,
            laser.back_outer_right,laser.back_center_right,laser.back_center,laser.back_center_left,laser.back_outer_left])
        laser_vec2 = np.array([laser.front_outer_left,laser.front_center_left,laser.front_center,laser.front_center_right,laser.front_outer_right,
            laser.back_outer_right,laser.back_center_right,laser.back_center,laser.back_center_left,laser.back_outer_left])

        nearest_val = 500000
        nearest = laser.front_center
        nearest2nd_val = 500000
        nearest2nd = laser.front_center
        for j in range(0,10):
            if laser_vec[j].original_dist != 0 and laser_vec[j].original_dist < nearest_val:
                nearest_val = laser_vec[j].original_dist
                nearest = laser_vec[j]
        for j in range(0,10):
            alpha = laser_vec[j].alfa;
            if laser_vec[j].alfa < 0:
                alpha = laser_vec[j].alfa + 360            
            if laser_vec[j].original_dist != 0 and laser_vec[j] != nearest and not is_zero(abs(alpha - nearest.alfa),limit=60) and laser_vec[j].original_dist < nearest2nd_val:
                nearest2nd_val = laser_vec[j].original_dist
                nearest2nd = laser_vec[j]

        debug("Nearest sector: " + nearest.tag)
        debug("Second nearest sector: " + nearest2nd.tag)

        def wanted_ray_alfa(rays, dist_f):
            distances = np.array(map(lambda ray: ray.dist, rays))
            match_dist = dist_f(distances)
            matches_indices = np.where(distances == match_dist)[0]
            match_rays = [rays[i] for i in matches_indices]
            if len(match_rays) == 1:
                return match_rays[0].alfa
            larger_sectors = map(lambda ray: ray.sector.degrees(), match_rays)
            return match_rays[larger_sectors.index(max(larger_sectors))].alfa

        alfa1 = wanted_ray_alfa(nearest.sector.rays(),min)
        alfa2 = wanted_ray_alfa(nearest2nd.sector.rays(),min)

        debug("alfa1: %i, alfa2 %i" % (alfa1,alfa2))
        if alfa1 < 0:
            alfa1 = alfa1 + 360
        if alfa2 < 0:
            alfa2 = alfa2 + 360
        alfa = mean_angle(alfa1,alfa2)
        debug("canviats de signe alfa1: %i, alfa2 %i i alfa mig resultant en degrees: %i" % (alfa1,alfa2,alfa))

        neato.sleep(2)

        neato.rotate(alfa + 180)

        LaserRay.DIST_LIMIT = 100

        iniLW = LaserRay.DIST_LIMIT
        iniRW = LaserRay.DIST_LIMIT

        MOVING_DIST = LaserRay.DIST_LIMIT

        k_front_outer_right = 1 * MOVING_DIST / 16
        k_front_center_right = 1 * MOVING_DIST / 2
        k_front_center = 2 * MOVING_DIST / 3
        k_front_center_left = 1 * MOVING_DIST / 2
        k_front_outer_left = 1 * MOVING_DIST / 16

        while laser.back_center.original_dist() < 0.8:
            laser = neato.get_laser(avoidingConfiguration)
            if laser.back_center.proximity_percent() > 0.95:
                break
            dL = -iniLW + (k_front_outer_right*laser.back_outer_right.proximity_percent() + k_front_center_right*laser.back_center_right.proximity_percent() + k_front_center*laser.back_center.proximity_percent())
            dR = -iniRW + (k_front_outer_left*laser.back_outer_left.proximity_percent() + k_front_center_left*laser.back_center_left.proximity_percent() + k_front_center*laser.back_center.proximity_percent()/2)
            neato.set_motors(left=dL, right=dR)

        neato.close()
        neato.saved_speed = original_speed

def envia(ser, message, delay=0.1, show_time=False):
    return _envia(ser, message, delay, show_time)

## INTERNAL METHODS ##

def _Neato_init(neato, default_speed=100, odometry=None, laser=False, viewer=False, serial=True):
    if serial:
        neato.ser = serial_port.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=GO_SUPER_FAST) # open serial port
    neato.sleep = _Neato_sleep_bound(neato)
    neato.saved_speed = default_speed
    neato._start_motors = 0
    neato._accL = 0
    neato._accR = 0
    neato._L = 0
    neato._R = 0
    neato._updating = False
    neato.test_mode()
    neato.beep()
    neato.enable_motors()
    neato.odometry = odometry if odometry != None else Odometry()
    L_ini, R_ini = neato.get_motors()
    neato._L = L_ini
    neato._R = R_ini
    neato.odometry.L = L_ini
    neato.odometry.R = R_ini
    neato.enabled_laser = False
    neato.enable_laser(laser)
    if viewer:
        neato.viewer_start(int(sys.argv[1]))
    neato.sleep(0.5)

def _Neato_close(neato):
    neato.beep(2)
    neato.enable_motors(False)
    neato.enable_laser(False)
    neato.test_mode(False)
    if hasattr(neato, 'viewer'):
        neato.viewer.quit()
    if not neato.is_mocked():
        neato.ser.close() # close serial port
    neato.sleep(0.3)
    info("Goodbye!")
    exit(0)

def _close_on_ctrl_C(neato, block):
    try:
        block()
    except KeyboardInterrupt:
        neato.close()

def _Neato_sleep(neato, delay):
    _close_on_ctrl_C(neato, lambda: time.sleep(delay))

def _Neato_sleep_bound(neato):
    return lambda delay: _Neato_sleep(neato, delay)

#waiting = False

def _envia(ser, message, delay=0.1, show_time=False, sleep=time.sleep):
    """
    Function to send commands to Neato robot.
    Parameters:
                message: Command to send
                delay: delay to recive data
    """
    #global waiting
    #if waiting:
    #    sleep(delay)

    #waiting = True
    debug(message)
    first_time = time.time()
    rbuffer = ''
    resp = ''
    ser.write(message + chr(10))
    sleep(delay) # giving a breath to pi
    
    while ser.inWaiting() > 0:
        resp = ser.readline()
        rbuffer = rbuffer + resp

    if show_time:
        t = time.time() - first_time
        debug("Elapsed %.2f seconds" % t)

    waiting = False
    return rbuffer