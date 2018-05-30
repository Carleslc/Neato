# -*- coding: utf-8 -*-

import serial

from async import loop
from neato import *

exhaustive = False

def info_laser():
    if exhaustive:
        laser.show(empties=True)
    laser.summary(empties=True)

def info():
    info_laser()
    neato.odometry.show()

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
    if is_zero(iniLW - dL, limit=50) and is_zero(iniRW - dR, limit=50): # there are no obstacles
        speed = MAX_SPEED
        # fix direction
        yOffset = yIni - neato.odometry.y
        offset = -neato.offset_from(alfaIni)
        rL, rR = neato.rotation_motors(offset, limit=5)
        debug("rL/rR = %.2f / %.2f" % (rL, rR))
        alfaDistL = (float(rL)/MAX_ALFA * dL)
        alfaDistR = (float(rR)/MAX_ALFA * dR)
        debug("alfaDistL/R = %.2f / %.2f" % (alfaDistL, alfaDistR))
        dL = dL + alfaDistL
        dR = dR + alfaDistR
        if not is_zero(yOffset, limit=10):
            debug("OFFSET Y %.2f" % yOffset)
            if yOffset > 0: # estàs a la dreta
                dR = dR + yOffset
                #dL = dL - yOffset
            else: # estàs a l'esquerra
                #dR = dR - abs(yOffset)
                dL = dL + abs(yOffset)
        """# fix orientation
            orientate = neato.set_alfa(gamma, limit=5)
            if orientate:
                debug("Rotated to initial alfa")"""
    else:
        speed = MAX_SPEED/3

    neato.set_motors(left=dL, right=dR, speed=speed)
    neato.update_odometry()

def follow_wall():
    meeting_distances_config()

def meet_object():
    dL = iniLW + (laser.front_outer_right.proximity()*16 + laser.front_center_right.proximity()*8)
    dR = iniRW + (laser.front_outer_left.proximity()*16 + laser.front_center_left.proximity()*8)
    neato.set_motors(left=dL, right=dR)

def meet_object_v2():
    near1 = laser.nearest(average=False)
    near2 = neato.get_laser(laser_conf).nearest(average=False)
    near3 = neato.get_laser(laser_conf).nearest(average=False)

    near = near3
    nearMedDist = medianOfThree(near1.dist, near2.dist, near3.dist)
    if nearMedDist == near1.dist:
        near = near1
    elif nearMedDist == near2.dist:
        near = near2

    debug("Nearest: " + near.tag)
    debug("Alfa: %.2f" % near.alfa)
    neato.set_alfa(near.alfa, limit=5)
    if not is_zero(near.dist + Laser.OFFSET, limit=50):
        neato.move_forwards(near.dist + Laser.OFFSET)
    neato.update_odometry()
    neato.sleep(5)

def avoid_obstacles_config():
    global iniLW, iniRW, MOVING_DIST, MAX_ALFA, MAX_SPEED, speed, k_front_outer_right, k_front_center_right, k_front_center, k_front_center_left, k_front_outer_left
    LaserRay.DIST_LIMIT = 450
    iniLW = LaserRay.DIST_LIMIT
    iniRW = LaserRay.DIST_LIMIT

    MOVING_DIST = LaserRay.DIST_LIMIT
    MAX_ALFA = pi*Neato.S
    MAX_SPEED = 250
    speed = MAX_SPEED

    # Avoiding obstacles parameters
    k_front_outer_right = 1 * MOVING_DIST / 16
    k_front_center_right = 1 * MOVING_DIST / 2
    k_front_center = 2 * MOVING_DIST / 3
    k_front_center_left = 1 * MOVING_DIST / 2
    k_front_outer_left = 1 * MOVING_DIST / 16

def meeting_distances_config():
    global speed, iniLW, iniRW
    speed = 200
    LaserRay.DIST_LIMIT = 4000
    iniLW = LaserRay.DIST_LIMIT
    iniRW = LaserRay.DIST_LIMIT

if __name__ == "__main__":
    log_level(DEBUG)

    global neato, alfaIni, yIni, laser_conf

    meeting_distances_config()

    neato = Neato(speed=speed, laser=True)

    alfaIni = neato.alfa()
    yIni = neato.odometry.y
    debug("Alfa INI: %.2f" % alfaIni)
    debug("y INI: %.2f" % yIni)

    neato.sleep(2)

    def run_with_laser(f):
        def execute():
            global laser
            laser = neato.get_laser(laser_conf)
            f()
        return execute

    loop(info, interval=5) # display info every 3 seconds (in addition to other calls)

    laser_conf = commonConfiguration
    neato.run(run_with_laser(meet_object_v2))