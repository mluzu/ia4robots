import random
import numpy as np
from math import pi, cos, sin, asin

class Robot:

    def __init__(self, world_size, DT, name = None):
        self.name = name
        self.world_size = world_size
        self.DT = DT

        # estado inicial
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi

        # parametros de ruido
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_x_noise = 0.0
        self.sense_y_noise = 0.0
        self.sense_yaw_noise = 0.0
    
    def __repr__(self):
        return 'Estado del robot %s: x=%.6s y=%.6s orient=%.6s' % (self.name, str(self.x), str(self.y), str(self.orientation))

    
    def set(self, new_x, new_y, new_orientation):
            if new_x < 0 or new_x >= self.world_size:
                raise ValueError
            if new_y < 0 or new_y >= self.world_size:
                raise ValueError
            if new_orientation < 0 or new_orientation >= 2 * pi:
                raise ValueError
            self.x = float(new_x)
            self.y = float(new_y)
            self.orientation = float(new_orientation)

    def set_noise(self, new_f_noise, new_t_noise, xm_noise, ym_noise, yawm_noise):
        # makes it possible to change the noise parameters
        # this is often useful in particle filters
        self.forward_noise = float(new_f_noise)
        self.turn_noise = float(new_t_noise)
        self.sense_x_noise = float(xm_noise)
        self.sense_x_noise = float(ym_noise)
        self.sense_yaw_noise = float(yawm_noise)
    
    def get_true_pos(self):
        return np.array([self.x, self.y])
    
    def sense(self):
        x = self.x + random.gauss(0.0, self.sense_x_noise)
        y = self.y + random.gauss(0.0, self.sense_y_noise)
        yaw = self.orientation + random.gauss(0.0, self.sense_yaw_noise)
        vel = 1.0 # no medimos velocidad
        return np.array([x, y, yaw, vel])

    def move(self, forward, turn):
        67
        # turn, and add randomness to the turning command
        orientation = self.orientation + float(turn) + random.gauss(0.0, self.turn_noise)
        orientation %= (2*pi)
        self.orientation = orientation
        
        # move, and add randomness to the motion command
        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        self.x = (self.x + (cos(orientation) * dist)) % self.world_size
        self.y = (self.y + (sin(orientation) * dist)) % self.world_size
    
    def orientation_vector(self):
        x = cos(self.orientation) 
        y = sin(self.orientation)
        return np.array([x, y])


    def point_objetive(self, Xpred, u):
        """
        función que inicializa el control del robot
        dada la posición del objetivo
        """
        max_speed = 2.0 # fijamos la velocidad máxima del robot
        new_u = np.zeros(2)
        Xobj = Xpred[:2]
        Xtrue = np.array([self.x, self.y])
        dir_vec = Xobj - Xtrue
        dir_vec_mag = np.linalg.norm(dir_vec, ord=2)
        or_vec = self.orientation_vector()
        
        # el robot se dirige al objetivo a máxima velocidad hasta
        # se ponse dentro de un radio de acción de 10 m
        if dir_vec_mag > 10:
            new_u[0] = max_speed
        else:
            return self.next_move(Xpred, u, dir_vec_mag)

        new_u[1] = -get_angle(dir_vec, or_vec)

        return new_u, dir_vec_mag


    def next_move(self, Xpred, u, target_dist_prev):
        max_speed = 2.0 # fijamos la velocidad máxima del robot
        new_u = np.zeros(2)
        Xobj = Xpred[:2]
        Xtrue = np.array([self.x, self.y])
        dir_vec = Xobj - Xtrue # vector director de la recta que une los dos puntos (robots)
        dir_vec_mag = np.linalg.norm(dir_vec, ord=2) # magnitud de la distancia
        dist_delta = target_dist_prev - dir_vec_mag # diferencia entre la distancia en t y la distancia en t + 1
        ang = get_angle(dir_vec, self.orientation_vector())
        print(dist_delta)

        """
        Si el robot está dentro del radio de acción empieza a ajustar
        velocidad y águlo. Mientras está fuera del rango de acción mantiene
        máxima velocidad. Si está a una distancia de 0.01 se mantiene la velocidad
        regulada hasta ese instante. Dentro del radio de acción el robot 
        """

        if dir_vec_mag > 10.0:
            new_u[0] = max_speed
        elif dir_vec_mag < 10.0:
            if dist_delta < 0:
                inc = max_speed * np.exp(abs(dist_delta))
                print(f'velocidad aumenta {inc}')
                new_u[0] = (u[0] + inc) % max_speed # si agrandó la distancia aumento la velocidad 
            else:
                decr = np.exp(-abs(10*dir_vec_mag))
                print(f'velocidad disminuye {decr}')
                new_u[0] = u[0] - decr # si achicó la distancia disminuyo la velocidad
        
        """
        el ángulo que debo rotar es el que existe entre la dirección
        del robot 2 y el vector que lo une con el robot 1 (dir_vec). Pero
        mientras esté fuera del radio de acción solo corrijo dirección si dicho
        ángulo pasa de 30 grados.
        """
        if dir_vec_mag > 10.0:
            if abs(ang) > 0.523:
                new_u[1] = ang
            else:
                new_u[1] = 0.0 # porque ya fijé la dirección hacia el objetivo
        else:
            new_u[1] = ang
        return new_u, dir_vec_mag

def get_angle(v1, v2):
    """
    usando producto vectorial puedo saber el signo que necesito que tenga el
    ángulo.Transformo los vectores para que se encuentren en el plano
    z = 0 en R3. En este caso es fácil conocer la orientación del vector producto con
    el signo de la coordenada z.
    """
    v1 = np.array([v1[0], v1[1], 0.0])
    v2 = np.array([v2[0], v2[1], 0.0])
    cross_prod = np.cross(v1, v2)
    norms = np.linalg.norm(v1, 2) * np.linalg.norm(v2, 2)
    cross_prod_norm = np.linalg.norm(cross_prod, 2)
    ang = asin(cross_prod_norm/norms)
    if cross_prod[2] < 0:
        return ang
    else:
        return -ang
    