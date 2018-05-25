import serial

from async import loop
from neato import *

exhaustive = False
LaserRay.DIST_LIMIT = 360
iniLW = LaserRay.DIST_LIMIT
iniRW = LaserRay.DIST_LIMIT
min_distance = 100

MOVING_DIST = LaserRay.DIST_LIMIT
MAX_ROTATION_DIST = pi*Neato.S

k_front_outer_right = 0.5 * MOVING_DIST / 2
k_front_center_right = 1 * MOVING_DIST / 2
k_front_center = 2 * MOVING_DIST / 2
k_front_center_left = 1 * MOVING_DIST / 2
k_front_outer_left = 0.5 * MOVING_DIST / 2

def info_laser():
    if exhaustive:
        debug("Show laser")
        laser.show(empties=True)
    laser.summary()

def esquivar_obstacles():
    neato.set_motors(left=iniLW - (k_front_outer_right*laser.front_outer_right.proximity_percent() + k_front_center_right*laser.front_center_right.proximity_percent() + k_front_center*laser.front_center.proximity_percent()),# + ((actual/180)==0)*(actual%180/18.0),
                         right=iniRW - (k_front_outer_left*laser.front_outer_left.proximity_percent() + k_front_center_left*laser.front_center_left.proximity_percent()))

def get_rotation_distances():
    dL = iniLW - (k_front_outer_right*laser.front_outer_right.proximity_percent() + k_front_center_right*laser.front_center_right.proximity_percent() + k_front_center*laser.front_center.proximity_percent())
    dR = iniRW - (k_front_outer_left*laser.front_outer_left.proximity_percent() + k_front_center_left*laser.front_center_left.proximity_percent())
    return dL, dR

def esquivar_obstacles_pero_tornant_a_l_angle_original():
    global alfaIni
    orientate = False
    dL, dR = get_rotation_distances()
    if dL == iniLW and dR == iniRW:
        debug("ORIENTATE TO INITIAL ALFA")
        orientate = neato.set_alfa(alfaIni, limit=1)
    neato.set_motors(left=dL, right=dR)
    neato.update_odometry()
    info("Alfa: %.2f" % neato.alfa())

if __name__ == "__main__":
    log_level(DEBUG)

    global neato, alfaIni
    neato = Neato(speed=100, laser=True)

    alfaIni = neato.alfa()
    debug("Alfa INI: %.2f" % alfaIni)

    def run_with_laser(f):
        def execute():
            global laser
            laser = neato.get_laser(commonConfiguration)
            info_laser()
            f()
        return execute

    #loop(info, interval=3) # display info every 3 seconds (in addition to other calls)

    neato.run(run_with_laser(esquivar_obstacles_pero_tornant_a_l_angle_original))