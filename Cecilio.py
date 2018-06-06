# -*- coding: utf-8 -*-

import serial

from async import loop
from neato import *

exhaustive = False

def info_laser():
    if exhaustive:
        laser.show(empties=True)
    laser.summary(empties=True)

def show_info():
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
    global findWallLimit, follow_wall_direction, wall_dist_limit, wall_dist_limit_min, k_front_outer_right, k_front_center_right, k_front_center, k_front_center_left, k_front_outer_left, front_prox_percent, calibration, iniLW, iniRW
    avoid_obstacles_config()
    follow_wall_direction = 1 # 1: LEFT, -1: RIGHT
    wall_dist_limit_min = 100
    wall_dist_limit = 500
    front_prox_percent = 0.7
    calibration = 200
    findWallLimit = 500
    LaserRay.DIST_LIMIT = 1500
    iniLW = 600
    iniRW = 600
    MOVING_DIST = 450
    k_front_outer_right = 1 * MOVING_DIST / 16
    k_front_center_right = 1 * MOVING_DIST / 16
    k_front_center = 7 * MOVING_DIST / 10
    k_front_center_left = 1 * MOVING_DIST / 16
    k_front_outer_left = 1 * MOVING_DIST / 16

def race_config():
    global MAX_SPEED, speed, findWallLimit, wall_dist_limit, wall_dist_limit_min, k_front_outer_right, k_front_center_right, k_front_center, k_front_center_left, k_front_outer_left, front_prox_percent, calibration, iniLW, iniRW
    avoid_obstacles_config()
    wall_dist_limit_min = 100
    wall_dist_limit = 500
    front_prox_percent = 0.7
    calibration = 200
    findWallLimit = 500
    LaserRay.DIST_LIMIT = 1500
    iniLW = 600
    iniRW = 600
    MOVING_DIST = 450
    MAX_SPEED = 300
    speed = MAX_SPEED
    k_front_outer_right = 1 * MOVING_DIST / 16
    k_front_center_right = 1 * MOVING_DIST / 16
    k_front_center = 7 * MOVING_DIST / 10
    k_front_center_left = 1 * MOVING_DIST / 16
    k_front_outer_left = 1 * MOVING_DIST / 16

def scan_wall_ray():
    laserLimit = LaserRay.DIST_LIMIT
    LaserRay.DIST_LIMIT = findWallLimit
    wallRay = neato.get_laser(findWallConfiguration).get_wall().nearest()
    LaserRay.DIST_LIMIT = laserLimit
    return wallRay

def scan_wall_ray_direction():
    return neato.get_laser(findWallConfiguration).sectors.find(follow_wall_direction*90).nearest()

def scan_wall_race_direction():
    return neato.get_laser(findWallConfiguration).left.sector

def find_wall():
    global wall_alfa
    wall = scan_wall_ray()
    debug("Wall ray: %s" % str(wall))
    neato.rotate(wall.alfa)
    debug("Distance to wall: %.2f" % wall.original_dist)
    neato.move_forwards(wall.dist - wall_dist_limit_min)
    neato.rotate(-follow_wall_direction*90)
    wall = scan_wall_ray()
    debug("Wall Alfa: %.2f" % wall.alfa)

def distances_follow_wall(laser, wall):
    go_right = go_left = 0
    def follow_wall(direction):
        go_right, go_left = neato.rotation_motors(wall.alfa - follow_wall_direction*90)
        debug("quant intenta rotar (rL, rR): (%i, %i)" % (go_right, go_left))
        """if direction == 'front_center_right':
            go_right = go_right + abs(wall.original_dist - calibration)
        else:
            go_left = go_left + abs(wall.original_dist - calibration)"""
        front_center = laser.front_center.proximity_percent()
        front_center_dir = laser[direction].proximity_percent()
        front_prox = front_prox_percent if is_zero(front_center) or is_zero(front_center_dir) else front_prox_percent*1.5
        debug("front_center %.2f" % front_center)
        debug("front_center_dir %.2f" % front_center_dir)
        debug("front_prox %.2f" % front_prox)
        obstacle = (front_center + front_center_dir) > front_prox
        if obstacle:
            if direction == 'front_center_right':
                go_right = -(k_front_center*front_center + k_front_center_left*front_center_dir)
            else:
                go_left = -(k_front_center*front_center + k_front_center_right*front_center_dir)
            debug("AVOID OBSTACLE")
        return go_right, go_left
    debug("Wall: %s" % str(wall))
    debug("Distance to wall: %i" % wall.original_dist)
    if wall.dist < wall_dist_limit:
        debug("WALL ON A SIDE")
        #offset = -neato.offset_from(wall.alfa - follow_wall_direction*90)
        if wall.sector.is_left():
            debug("GO LEFT")
            go_right, go_left = follow_wall('front_center_left')
        elif wall.sector.is_right():
            debug("GO RIGHT")
            go_right, go_left = follow_wall('front_center_right')
        debug("go_left, go_right: %i, %i" % (go_left, go_right))
        dL = iniLW + go_right
        dR = iniRW + go_left
        return dL, dR
    else:
        debug("GO TO THE CLOSEST WALL")
        find_wall()
        return 0, 0

def follow_nearest_wall(laser):
    dL, dR = distances_follow_wall(laser, scan_wall_ray())
    neato.set_motors(left=dL, right=dR)

def follow_wall(laser):
    dL, dR = distances_follow_wall(laser, scan_wall_ray_direction())
    neato.set_motors(left=dL, right=dR)

def distances_race(laser, wall_sector):
    go_right = go_left = 0
    wall_far = wall_sector.farthest()
    wall_near = wall_sector.nearest()
    debug("Wall: %s" % str(wall_sector))
    debug("Distance to wall: %i" % wall_near.original_dist)
    go_right, go_left = neato.rotation_motors(wall_near.alfa - 90)
    debug("quant intenta rotar (rL, rR): (%i, %i)" % (go_right, go_left))
    go_left = go_left + max(0, wall_far.dist - calibration)
    debug("Farthest distance to wall: %i" % wall_far.dist)
    front_center = laser.front_center.proximity_percent()
    front_center_dir = laser.front_center_left.proximity_percent()
    front_prox = front_prox_percent if is_zero(front_center) or is_zero(front_center_dir) else front_prox_percent*1.5
    debug("front_center %.2f" % front_center)
    debug("front_center_dir %.2f" % front_center_dir)
    debug("front_prox %.2f" % front_prox)
    obstacle = (front_center + front_center_dir) > front_prox
    if obstacle:
        go_left = -(k_front_center*front_center + k_front_center_right*front_center_dir)
        debug("AVOID OBSTACLE")
    debug("go_left, go_right: %i, %i" % (go_left, go_right))
    dL = iniLW + go_right
    dR = iniRW + go_left
    return dL, dR

def race(laser):
    dL, dR = distances_race(laser, scan_wall_race_direction())
    neato.set_motors(left=dL, right=dR, speed=speed)

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

def prey_config():
    global MOVING_DIST, MAX_SPEED, speed, k_front_outer_right, k_front_center_right, k_front_center, k_front_center_left, k_front_outer_left, k_n
    LaserRay.DIST_LIMIT = 1000

    MOVING_DIST = LaserRay.DIST_LIMIT
    MAX_SPEED = 300
    speed = MAX_SPEED

    # Avoiding obstacles parameters
    k_front_outer_right = 1 * MOVING_DIST / 8
    k_front_center_right = 1 * MOVING_DIST
    k_front_center = 4 * MOVING_DIST / 3
    k_front_center_left = 1 * MOVING_DIST
    k_front_outer_left = 1 * MOVING_DIST / 8

    k_n = 0.2

def predator_config():
    global MOVING_DIST, MAX_SPEED, speed, k_front_outer_right, k_front_center_right, k_front_center, k_front_center_left, k_front_outer_left, k_n
    LaserRay.DIST_LIMIT = 1500

    MOVING_DIST = LaserRay.DIST_LIMIT
    MAX_SPEED = 100
    speed = 300

    # Avoiding obstacles parameters
    k_front_outer_right = 1 * MOVING_DIST / 8
    k_front_center_right = 1 * MOVING_DIST
    k_front_center = 4 * MOVING_DIST / 3
    k_front_center_left = 1 * MOVING_DIST
    k_front_outer_left = 1 * MOVING_DIST / 8

    k_n = 0.2

def prey(laser):
    #escape from the front (removed ini and interchanged dL and dR)
    """
    fdR = -(k_front_outer_right*laser.front_outer_right.proximity_percent() + k_front_center_right*laser.front_center_right.proximity_percent() + k_front_center*laser.front_center.proximity_percent())
    fdL = -(k_front_outer_left*laser.front_outer_left.proximity_percent() + k_front_center_left*laser.front_center_left.proximity_percent() + k_front_center*laser.front_center.proximity_percent()/2)
    debug("front dL, dR: %i, %i" % (fdL, fdR))
    #escape from the back (removed ini and interchanged dL and dR)
    bdR = k_front_outer_right*laser.back_outer_right.proximity_percent() + k_front_center_right*laser.back_center_right.proximity_percent() + k_front_center*laser.back_center.proximity_percent()
    bdL = k_front_outer_left*laser.back_outer_left.proximity_percent() + k_front_center_left*laser.back_center_left.proximity_percent() + k_front_center*laser.back_center.proximity_percent()/2
    debug("back dL, dR: %i, %i" % (bdL, bdR))
    """

    front_left = pow(k_front_outer_left,laser.front_outer_left.proximity_percent()+k_n) + pow(k_front_center_left,laser.front_center_left.proximity_percent()+k_n)
    front_right = pow(k_front_outer_right,laser.front_outer_right.proximity_percent()+k_n) + pow(k_front_center_right,laser.front_center_right.proximity_percent()+k_n)
    fdR = -(front_left/2 + front_right)
    fdL = -(front_right/2 + front_left)
    debug("front dL, dR: %i, %i" % (fdL, fdR))

    back_left = pow(k_front_outer_left,laser.back_outer_left.proximity_percent()+k_n) + pow(k_front_center_left,laser.back_center_left.proximity_percent()+k_n)
    back_right = pow(k_front_outer_right,laser.back_outer_right.proximity_percent()+k_n) + pow(k_front_center_right,laser.back_center_right.proximity_percent()+k_n)
    bdR = -max(0, back_right - pow(k_front_center,laser.back_center.proximity_percent()+k_n+0.1))
    bdL = -max(0, back_left - pow(k_front_center,laser.back_center.proximity_percent()+k_n+0.1)/2)
    debug("back dL, dR: %i, %i" % (bdL, bdR))

    L = fdL + bdL
    R = fdR + bdR
    if not is_zero(abs(L) + abs(R), limit=200):
        neato.set_motors(left=L, right=R)

def neato_detection_vote(laser, count=5):
    i = 0
    votation = dict() # int(alfa/5): ([alfa], count)
    for i in range(count):
        detected, alfa = laser.detect_neato()
        if detected:
            alfa5 = int(alfa/5)
            default_alfa = votation[alfa5] if alfa5 in votation else list()
            default_alfa.append(alfa)
            votation[alfa5] = default_alfa
        if i < count - 1: # not last
            laser = neato.get_laser(laser_conf)
    info("Votes: %s" % str(votation))
    if len(votation) == 0:
        return False, 0
    else:
        votes = max(map(lambda alfas: len(alfas), votation.values()))
        voted_alfas = next(alfas for alfas in votation.values() if len(alfas) == votes)
        voted_alfa = sum(voted_alfas)/len(voted_alfas)
        return True, voted_alfa

def predator(laser):
    detected, alfa = neato_detection_vote(laser)
    if detected:
        info("Neato detected at %s" % laser.sectors.find(alfa).tag.upper())

        """a_alfa = abs_alfa(alfa)
        if a_alfa > 90 and a_alfa < 270:
            neato.rotate(a_alfa)
        else:
            dR = 0#k_front_outer_right*laser.front_outer_right.proximity_percent() + k_front_center_right*laser.front_center_right.proximity_percent() + k_front_center*laser.front_center.proximity_percent()
            dL = 0#k_front_outer_left*laser.front_outer_left.proximity_percent() + k_front_center_left*laser.front_center_left.proximity_percent() + k_front_center*laser.front_center.proximity_percent()/2

            rL, rR = neato.rotation_motors(alfa)

            dL = dL + rL
            dR = dR + rR

            neato.set_motors(left=dL, right=dR)"""
        neato.rotate(alfa)
        neato.move_forwards(500)
    else:
        debug("Not detected")

def run(config):
    log_level(INFO)

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
        def execute(key=None):
            global laser
            laser = neato.get_laser(laser_conf)
            f(laser)
        return execute

    #loop(show_info, interval=5) # display info every 3 seconds (in addition to other calls)

    laser_conf = config.laser_conf
    if config.ini != None:
        neato.run_once(config.ini)
    neato.run(run_with_laser(config.f))