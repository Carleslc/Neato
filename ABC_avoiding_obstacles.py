# -*- coding: utf-8 -*-

import serial

from async import loop
from neato import *

exhaustive = False
LaserRay.DIST_LIMIT = 350
iniLW = LaserRay.DIST_LIMIT
iniRW = LaserRay.DIST_LIMIT

MOVING_DIST = LaserRay.DIST_LIMIT

# Avoiding obstacles parameters
k_front_outer_right = 1 * MOVING_DIST / 16
k_front_center_right = 1 * MOVING_DIST / 2
k_front_center = 2 * MOVING_DIST / 3
k_front_center_left = 1 * MOVING_DIST / 2
k_front_outer_left = 1 * MOVING_DIST / 16

def info_laser():
    if exhaustive:
        laser.show(empties=True)
    laser.summary(empties=True)

def get_avoiding_distances():
    dL = iniLW - (k_front_outer_right*laser.front_outer_right.proximity_percent() + k_front_center_right*laser.front_center_right.proximity_percent() + k_front_center*laser.front_center.proximity_percent())
    dR = iniRW - (k_front_outer_left*laser.front_outer_left.proximity_percent() + k_front_center_left*laser.front_center_left.proximity_percent() + k_front_center*laser.front_center.proximity_percent()/2)
    return dL, dR

def avoid_obstacles():
    dL, dR = get_avoiding_distances()
    neato.set_motors(left=dL, right=dR)

def avoid_obstacles_with_original_direction(): # 1,5 punts esquivar objecte mantenint direccio, 2 punts si manté la mateixa linia
    global alfaIni, yIni
    dL, dR = get_avoiding_distances()
    if dL == iniLW and dR == iniRW: # there are no obstacles
        # fix orientation
        orientate = neato.set_alfa(alfaIni, limit=20)
        if orientate:
            debug("Rotated to initial alfa")
        # fix direction
        yOffset = yIni - neato.odometry.y
        if not is_zero(yOffset, limit=50):
            debug("OFFSET Y %.2f" % yOffset)
            if yOffset > 0:
                dR = dR + yOffset
            else:
                dL = dL - yOffset
    neato.set_motors(left=dL, right=dR)
    neato.update_odometry()

def follow_wall():
    meeting_distances_config()

def meet_object():
    meeting_distances_config()
    dL = iniLW + (laser.front_outer_right.proximity()*16 + laser.front_center_right.proximity()*8)
    dR = iniRW + (laser.front_outer_left.proximity()*16 + laser.front_center_left.proximity()*8)
    neato.set_motors(left=dL, right=dR)

def meet_object_v2():
    meeting_distances_config()
    near = laser.nearest(average=False)
    debug("Nearest: " + near.tag)
    debug("Alfa: %.2f" % near.alfa)
    #neato.set_alfa(near.alfa, limit=5)
    if not is_zero(near.dist + Laser.OFFSET):
    	pass
        #self.move_forwards(dist)
    neato.update_odometry()
    neato.sleep(2)

def meeting_distances_config():
    LaserRay.DIST_LIMIT = 4000
    iniLW = LaserRay.DIST_LIMIT
    iniRW = LaserRay.DIST_LIMIT

if __name__ == "__main__":
    log_level(DEBUG)

    global neato, alfaIni, yIni
    neato = Neato(speed=100, laser=True)

    alfaIni = neato.alfa()
    yIni = neato.odometry.y
    debug("Alfa INI: %.2f" % alfaIni)
    debug("y INI: %.2f" % yIni)

    neato.sleep(2)

    def run_with_laser(f, laser_conf):
        def execute():
            global laser
            laser = neato.get_laser(laser_conf)
            info_laser()
            f()
        return execute

    #loop(info, interval=3) # display info every 3 seconds (in addition to other calls)

    neato.run(run_with_laser(avoid_obstacles_with_original_direction, avoidingConfiguration))