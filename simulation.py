import numpy as np
from math import *
import time
from robot import Robot
from kalman import Kalman
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot



def get_motion_params():
    v = 2.0  # [m/s]
    yawrate = -pi/12  # [rad/s]
    u = np.array([v, yawrate])
    return u

def run_simulation_pos(xm_noise, ym_noise, yawm_noise, step_prop, title):
    print("Start!")

    DT = 1.0  # time tick [s]
    SIM_TIME = 50.0  # simulation time [s]

    elapsed = 0.0
    world_size = 1000

    # inicializo el robot
    robot = Robot(world_size, DT, "loco")
    robot.set(30.0, 50.0, 0.0)
    robot.set_noise(0.0, 0.0, xm_noise, ym_noise, yawm_noise) # no hay ruido de control
    print(f'Inicialización del robot:\n{robot}')

    # inicializo filtro de kalman, siempre con varianza 0 para modelo
    init_state = np.array([30.0, 50.0, 0.0, 1.0])
    ekf = Kalman(init_state, DT, xm_noise, ym_noise, yawm_noise, 0.0, 0.0, 0.0)
    
    # el robot se vuelve a mover una distancia y un ángulo
    # dados por la velocidad lineal y angular multiplicadas por
    # el intervalo de tiempo
    u = get_motion_params() # velocidad lineal y angular
    f, y = DT*u

    z = robot.sense() # medición con ruido
    Xprev = ekf.estimate_next_pos(z, u) # prediccíon de la posición
    robot.move(f, y)

    iteraciones = 0

    while SIM_TIME >= elapsed:
        start_time = time.time()

        iteraciones = iteraciones + 1

        z = robot.sense() # medición con ruido

        Xpred = ekf.estimate_next_pos(z, u) # prediccíon de la posición
        
        dist = np.linalg.norm(Xprev[:2] - Xpred[:2])
        yaw = Xprev[2] - Xpred[2]
        print(robot)
        print(f'# Próxima posición: ({Xpred[0]}, {Xpred[1]})\n  Paso: {dist}m\tYaw: {yaw}')
        
        # termino cuando la aproximación de paso
        # está dentro de un error de 1e-6
        if abs(dist - f) < step_prop*f:
            print(f'{iteraciones} iteraciones')
            print(f'El robot se mueve en steps de {dist}')
            plt.cla()
            break

        Xprev = Xpred
        
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(robot.x, robot.y, marker=(3, 0, (robot.orientation*180/pi)-90), markersize=15, linestyle='None', color='blue')
        plt.axis("equal")
        plt.title(title)
        plt.grid(True)
        plt.pause(0.001)

        elapsed += DT
        sleep_time = 1 - (time.time() - start_time)
        time.sleep(sleep_time)

        robot.move(f, y)

    print('Finished!\n')



def run_simulation_catch(xm_noise, ym_noise, yawm_noise):
    print("Start!")

    DT = 1.0  # time tick [s]
    #SIM_TIME = 100.0  # simulation time [s]

    elapsed = 0.0
    world_size = 1000
    ticks = 0
    dist_delta = world_size

    #El robot 2 tiene que atrapar al robot 1
    # inicializo el robot 1
    robot1 = Robot(world_size, DT, "1")
    robot1.set(30.0, 50.0, pi/2)
    robot1.set_noise(0.0, 0.0, xm_noise, ym_noise, yawm_noise) # no hay ruido de control
    print(f'Inicialización del robot:\n{robot1}')
    u1 = np.array([1.0, -pi/12])

    # inicializo el robot 2
    robot2 = Robot(world_size, DT, "2")
    robot2.set(60.0, 60.0, pi/2)
    robot2.set_noise(0.0, 0.0, xm_noise, ym_noise, yawm_noise) # no hay ruido de control
    print(f'Inicialización del robot:\n{robot2}')
    
     # el robot 2 se mueve al doble de velocidad que el robot 1 (asumimos que 2.0 es velocidad máxima)
    # la velocidad se irá reduciendo para que se ajuste a la del robot 1
    # hasta converger y tengan ambos el mismo patrón de movimiento
    u2 = np.array([2.0, 0.0])

    # inicializo filtro de kalman sin  ruido
    init_state = np.array([30.0, 50.0, pi/2, 1.0])
    ekf = Kalman(init_state, DT, xm_noise, ym_noise, yawm_noise, 0.0, 0.0, 0.0)
    
    # los parámetros de movimiento para el robot 1 son constantes
    f1, y1 = DT*u1 

    z = robot1.sense()
    Xpred = ekf.estimate_next_pos(z, u1)
    robot1.move(f1, y1)

    # fijo al robot 1 como target del robot
    u2, dist_delta = robot2.point_objetive(z, u2)

    while True:
        start_time = time.time()

        # actualización robot 1
        z = robot1.sense() 
        Xpred = ekf.estimate_next_pos(z, u1) 
        
       
        # actualización robot 2
        # solo necesito las coordenas x,y que me llegan del robot 1
        u2, dist_delta = robot2.next_move(Xpred, u2, dist_delta)
        print(f'conontrol robot 2: {u2}')
        f2, y2 = u2*DT  # la velocidad es parte de la simulación, no un parámetro del robot
        robot2.move(f2, y2)

        print(robot1)
        print(robot2)

        catched, ticks = robot_atrapado(robot1, robot2, ticks)

        #si los robots se mantienen a una distancia menor a 0.001
        # durante 3 segundos se considera atrapado al robot 1
        if catched and ticks == 3:
            print(f'El robot fue capturado!\n')
            break
       
        elapsed += DT
        sleep_time = 1 - (time.time() - start_time)
        time.sleep(sleep_time)

        robot1.move(f1, y1)


        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(robot1.x, robot1.y, marker=(3, 0, (robot1.orientation*180/pi)-90), markersize=15, linestyle='None', color='blue')
        plt.plot(robot2.x, robot2.y, marker=(3, 0, (robot2.orientation*180/pi)-90), markersize=15, linestyle='None', color='green')
        plt.title("Problema 3 y 4")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)

    print('\nFinished!')


def robot_atrapado(r1, r2, ticks):
    x1 = r1.get_true_pos()
    x2 = r2.get_true_pos()
    d = np.linalg.norm(x1 - x2, ord=2)
    if abs(d) < 0.5:
        if ticks <= 3:
            return True, ticks + 1
    else:
        return False, 0