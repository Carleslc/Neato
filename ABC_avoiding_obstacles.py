# -*- coding: utf-8 -*-

import serial

from async import loop
from neato import *

exhaustive = False
LaserRay.DIST_LIMIT = 350
iniLW = LaserRay.DIST_LIMIT
iniRW = LaserRay.DIST_LIMIT
min_distance = 100

MOVING_DIST = LaserRay.DIST_LIMIT
MAX_ROTATION_DIST = pi*Neato.S

k_front_outer_right = 1 * MOVING_DIST / 16
k_front_center_right = 1 * MOVING_DIST / 2
k_front_center = 2 * MOVING_DIST / 3
k_front_center_left = 1 * MOVING_DIST / 2
k_front_outer_left = 1 * MOVING_DIST / 16

def info_laser():
    if exhaustive:
        debug("Show laser")
        laser.show(empties=True)
    laser.summary()

def esquivar_obstacles():
	dL, dR = get_avoiding_distances()
	neato.set_motors(left=dL, right=dR)

def get_avoiding_distances():
    dL = iniLW - (k_front_outer_right*laser.front_outer_right.proximity_percent() + k_front_center_right*laser.front_center_right.proximity_percent() + k_front_center*laser.front_center.proximity_percent())
    dR = iniRW - (k_front_outer_left*laser.front_outer_left.proximity_percent() + k_front_center_left*laser.front_center_left.proximity_percent() + k_front_center*laser.front_center.proximity_percent()/2)
    return dL, dR

def esquivar_obstacles_pero_tornant_a_l_angle_original():
	#1,5 punts esquivar objecte mantenint direccio, 2punts si manté la mateixa linia
    global alfaIni
    orientate = False
    dL, dR = get_avoiding_distances()
    if dL == iniLW and dR == iniRW:
        orientate = neato.set_alfa(alfaIni, limit=5)
    	debug("Rotated to initial alfa")
    neato.set_motors(left=dL, right=dR)
    neato.update_odometry()
    info("Alfa: %.2f" % neato.alfa())

def follow_wall():
	meeting_distances_config()


def meet_object():
	#can be done detecting a MAX with the laser (should be done like this?)
	meeting_distances_config()
	dL = iniLW + (laser.front_outer_right.proximity()*16 + laser.front_center_right.proximity()*8)
	dR = iniRW + (laser.front_outer_left.proximity()*16 + laser.front_center_left.proximity()*8)
	neato.set_motors(left=dL, right=dR)

def meeting_distances_config():
	LaserRay.DIST_LIMIT = 500
	iniLW = LaserRay.DIST_LIMIT
	iniRW = LaserRay.DIST_LIMIT

if __name__ == "__main__":
    log_level(DEBUG)

    global neato, alfaIni
    neato = Neato(speed=100, laser=True)

    alfaIni = neato.alfa()
    debug("Alfa INI: %.2f" % alfaIni)

    def run_with_laser(f):
        def execute():
            global laser
            laser = neato.get_laser(avoidingConfiguration)
            info_laser()
            f()
        return execute

    #loop(info, interval=3) # display info every 3 seconds (in addition to other calls)

    neato.run(run_with_laser(meet_object))