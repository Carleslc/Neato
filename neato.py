# -*- coding: utf-8 -*-

import sys
import time

import serial as serial_port

from log import *
from Laser import *
from async import run
from utils import mod, sign, is_zero, abs_alfa
from Odometry import Odometry
from math import sin, cos, atan, pi, sqrt, degrees, radians

GO_SUPER_FAST = 0

## NEATO ##

class NeatoMock(object):
    """
    Neato for testing purposes, does not works in real time but works as a Neato replacement
    when you does not have connection to the Neato robot
    """

    def __init__(self, laser=False, speed=100, delta_d=0, delta_th=0, theta_dot=0.0, x_ini=0, y_ini=0, noise_d=0.0001, noise_th=0.000001):
        _Neato_init(self, serial=False, default_speed=speed, laser=laser, odometry=Odometry(delta_d=delta_d, delta_th=delta_th, theta_dot=theta_dot, x_ini=x_ini, y_ini=y_ini, noise_d=noise_d, noise_th=noise_th))

    def is_mocked(self):
        return not hasattr(self, 'ser')

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
            debug('Finished Movement')
            if self.is_mocked():
                self.stop(reset_motors=False)
        self._resetLR_timer = run(resetLR, delay=max(self.time_to_complete() - delay, 0))

    def stop(self, reset_motors=True):
        if not self.is_stopped():
            debug("Stop Update Odometry")
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

    def rotation_motors(self, alfa, optimize=True):
        s = sign(alfa) # 1: left (default), -1: right
        if alfa < 0:
            alfa = -alfa
        if optimize:
            alfa = alfa % 360
            if alfa > 180:
                alfa = 360 - alfa
                s = -s
        info("Rotate %.2fº %s" % (alfa, 'LEFT' if s == 1 else 'RIGHT'))
        turn_dist = radians(alfa)*Neato.S
        return -s*turn_dist, s*turn_dist

    def rotate(self, alfa, optimize=True):
        L, R = self.rotation_motors(alfa, optimize)
        self.set_motors(L, R)
        self.sleep(abs(L)/self.saved_speed)
        debug("Sleep time: %i" % (abs(L)/self.saved_speed))
        self.show_odometry("AFTER ROTATION")

    def rotate_left(self, alfa, optimize=True):
        self.rotate(alfa, optimize)

    def rotate_right(self, alfa, optimize=True):
        self.rotate(-alfa, optimize)

    def move_forwards(self, d):
        info("Move %.2f mm" % d)
        self.set_motors(d, d)
        self.sleep(abs(d)/self.saved_speed)
        self.show_odometry("AFTER MOVE")

    def move_backwards(self, d):
        self.move_forwards(-d)

    def move(self, *moves):
        for move in moves:
            move.apply(self)

    def orientate(self, laser, sector):
        info("Orientate to %s" % sector.tag)
        self.rotate_right(sector.center())
        self.move_forwards(laser[sector.tag].proximity())

    def offset_from(self, alfa):
        return abs_alfa(self.alfa() - abs_alfa(alfa))

    def set_alfa(self, alfa, limit=0.001):
        alfa = -self.offset_from(alfa)
        debug("Alfa: %.2f" % alfa)
        rotate = not is_zero(alfa, limit)
        if rotate:
            self.rotate(alfa, optimize=True)
        return rotate

    def alfa(self):
        return degrees(self.odometry.suma_theta)

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
        debug("(AccL, AccR) = (%i, %i)" % (self._accL, self._accR))
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

class Neato(NeatoMock):
    """
    Subclass of NeatoMock, working on the real time Neato robot
    """

    S = 121.5; # millimeters

    def __init__(self, laser=False, speed=100, delta_d=0, delta_th=0, theta_dot=0.0, x_ini=0, y_ini=0, noise_d=0.0001, noise_th=0.000001):
        _Neato_init(self, serial=True, default_speed=speed, laser=laser, odometry=Odometry(delta_d=delta_d, delta_th=delta_th, theta_dot=theta_dot, x_ini=x_ini, y_ini=y_ini, noise_d=noise_d, noise_th=noise_th))
    
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
            alfa = degrees(atan(x/y))
            debug("Atan %.2f" % alfa)
            alfa = sign(alfa)*(90 - abs(alfa))
        if not is_zero(x) and x < 0: # backwards
            alfa = alfa + 180.0 # set forwards
        alfa = mod(alfa - neato.alfa()) # coordinates offset
        debug("Alfa: %.2f" % alfa)
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
        neato.set_alfa(self.alfa)

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
        neato.move_backwards(self.k)
        neato.saved_speed = original_speed

def envia(ser, message, delay=0.1, show_time=False):
    return _envia(ser, message, delay, show_time)

## INTERNAL METHODS ##

def _Neato_init(neato, default_speed=100, odometry=None, laser=False, serial=True):
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
    neato.sleep(0.5)

def _Neato_close(neato):
    neato.beep(2)
    neato.enable_motors(False)
    neato.enable_laser(False)
    neato.test_mode(False)
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