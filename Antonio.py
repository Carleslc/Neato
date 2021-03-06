from log import *
from math import pi, radians
from neato import Neato, NeatoMock, Pose, SecurePose

S = Neato.S

tiempo = 20

# DIRECTION
FORWARDS = 0
BACKWARDS = 1

go_to_zero = Pose(x=0, y=0, alfa=0)

direccion = FORWARDS

def move_with_key(tecla):
    global direccion
    debug("Key: " + tecla)
    neato.update_odometry() #this update is needed!
    neato.show_odometry("KEY PRESSED")
    neato.odometry_queue.put([(-neato.odometry.x,-neato.odometry.y),(100,100)]) #y i x perque el viewer ho imprimeix com vol
    neato.show_laser_viewer()


    speed = neato.odometry.speed

    if direccion == BACKWARDS:
        speed = -speed

    if tecla == '8' or tecla == '2':
        if tecla == '8':
            speed = speed + (neato.saved_speed - 100)
        else:
            speed = speed - (neato.saved_speed - 100)

        if speed >= 0:
            direccion = FORWARDS
        else:
            direccion = BACKWARDS

        if speed == 0:
            neato.stop()
        else:
            distancia_R = (((speed * pow(-1, direccion)) + (S * neato.odometry.theta_dot)) * tiempo) * pow(-1, direccion)
            distancia_L = (((speed * pow(-1, direccion)) + (-S * neato.odometry.theta_dot)) * tiempo) * pow(-1, direccion)

            neato.set_motors(distancia_L, distancia_R, speed*pow(-1, direccion))

    elif tecla == '4' or tecla == '6':

        if tecla == '4':
            neato.odometry.theta_dot = neato.odometry.theta_dot + (pi/10.0)
        else:
            neato.odometry.theta_dot = neato.odometry.theta_dot - (pi/10.0)

        distancia_R = (((speed * pow(-1, direccion)) + (S * neato.odometry.theta_dot)) * tiempo) * pow(-1, direccion)
        distancia_L = (((speed * pow(-1, direccion)) + (-S * neato.odometry.theta_dot)) * tiempo) * pow(-1, direccion)

        neato.set_motors(distancia_L, distancia_R, speed*pow(-1, direccion))

    elif tecla == '5':
        direccion = FORWARDS
        neato.stop()

    elif tecla == '0':
        neato.move(go_to_zero)

    elif tecla == '9':
        neato.move(go_to_charge)

    if tecla == '8' or tecla == '2' or tecla == '6' or tecla == '4' or tecla == '5':
        if direccion == FORWARDS:
            debug("Direction: forward")
        else:
            debug("Direction: backward")

    #debug("After key")
    #neato.show_odometry("AFTER KEY")
    info('\n')

if __name__ == "__main__":
    global neato, go_to_charge
    go_to_charge = SecurePose(x=int(sys.argv[4]), y=int(sys.argv[5]), alfa=int(sys.argv[6]), k=500)
    log_level(DEBUG)
    neato = Neato(speed=150, laser=True, viewer=True, x_ini=int(sys.argv[1]), y_ini=int(sys.argv[2]), suma_theta=radians(int(sys.argv[3])))
    neato.sleep(3)
    #neato.move(Manhattan(charging_station_x, charging_station_y), EuclideToPosition(0, 0))
    neato.run_until_key('q', move_with_key)