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

def get_avoiding_distances(laser):
    dL = iniLW - (k_front_outer_right*laser.front_outer_right.proximity_percent() + k_front_center_right*laser.front_center_right.proximity_percent() + k_front_center*laser.front_center.proximity_percent())
    dR = iniRW - (k_front_outer_left*laser.front_outer_left.proximity_percent() + k_front_center_left*laser.front_center_left.proximity_percent() + k_front_center*laser.front_center.proximity_percent()/2)
    return dL, dR

def avoid_obstacles(laser):
    dL, dR = get_avoiding_distances(laser)
    neato.set_motors(left=dL, right=dR)

def avoid_obstacles_with_original_direction(laser): # 1,5 punts esquivar objecte mantenint direccio, 2 punts si manté la mateixa linia
    global alfaIni, yIni
    dL, dR = get_avoiding_distances(laser)
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

def follow_wall_config():
    global wall_dist_limit
    avoid_obstacles_config()
    wall_dist_limit = 100
    LaserRay.DIST_LIMIT = 1500
    k_front_outer_right = 1 * MOVING_DIST
    k_front_center_right = 2 * MOVING_DIST
    k_front_center = 1 * MOVING_DIST
    k_front_center_left = 1 * MOVING_DIST
    k_front_outer_left = 1 * MOVING_DIST

def scan_wall_ray():
    return neato.get_laser(findWallConfiguration).get_wall().nearest()

def find_wall():
    wall = scan_wall_ray()
    debug("Wall: %s" % str(wall))
    neato.rotate(wall.alfa)
    debug("Distance to wall: %.2f" % wall.dist)
    dist = wall.dist - wall_dist_limit
    neato.move_forwards(dist)
    neato.rotate_right(90)

def distances_follow_wall(laser, wall):
    go_left = 0
    go_right = 0
    debug("Distance to wall: %.2f" % wall.dist)
    debug("GO TO WALL")
    if wall.sector.is_left():
        debug("GO LEFT")
        go_left = wall.dist - wall_dist_limit# + k_front_outer_right*laser.front_outer_right.dist
    elif wall.sector.is_right():
        debug("GO RIGHT")
        go_right = wall.dist - wall_dist_limit# + k_front_outer_left*laser.front_outer_left.proximity_percent()
    dL = iniLW - go_left
    dR = iniRW - go_right
    return dL, dR

def follow_wall(laser):
    dL, dR = distances_follow_wall(laser, scan_wall_ray())
    neato.set_motors(left=dL, right=dR)

def meet_object(laser):
    dL = iniLW + (laser.front_outer_right.proximity()*16 + laser.front_center_right.proximity()*8)
    dR = iniRW + (laser.front_outer_left.proximity()*16 + laser.front_center_left.proximity()*8)
    neato.set_motors(left=dL, right=dR)

def meet_object_v2(laser):
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
    debug("rotate Alfa: %.2f" % near.alfa)
    neato.rotate(near.alfa, limit=5)
    if not is_zero(near.dist + Laser.OFFSET, limit=50):
        neato.move_forwards(near.dist + Laser.OFFSET)
    neato.update_odometry()
    #neato.sleep(5)

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

def run(config):
    log_level(DEBUG)

    global neato, alfaIni, yIni, laser_conf

    config.var_conf()

    neato = Neato(speed=speed, laser=True)

    neato.sleep(2)

    alfaIni = neato.alfa()
    yIni = neato.odometry.y
    debug("Alfa INI: %.2f" % alfaIni)
    debug("y INI: %.2f" % yIni)

    neato.sleep(2)

    def run_with_laser(f):
        def execute():
            global laser
            laser = neato.get_laser(laser_conf)
            f(laser)
        return execute

    loop(info, interval=5) # display info every 3 seconds (in addition to other calls)

    laser_conf = config.laser_conf
    if config.ini != None:
        neato.run_once(config.ini)
    neato.run(run_with_laser(config.f))