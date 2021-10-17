import numpy as np
from math import *

# Hago un filtro de Kalman para estimar el estado (posicion) en t + 1 del robot
# dados el estado, el movimiento y la medición en t.

class Kalman:

    def __init__(self, init_state, DT, xy_qvar, yaw_qvar, vel_qvar, xy_rvar, yaw_rvar, vel_rvar):

        self.Xt = init_state
        
        # inicializo la varianza estimada
        self.Sigma_t = np.eye(4)

        # matriz de transición
        self.A = np.identity(4)
        
        # observation model
        self.C = np.identity(4)

        self.DT = np.float64(DT)
    
        # matriz de covarianza de medición
        self.Q = np.diag([xy_qvar, xy_qvar, yaw_qvar, vel_qvar]) 

        # matriz de covarianza de proceso
        self.R = np.diag([xy_rvar, xy_rvar, yaw_rvar, vel_rvar])  
      

    def estimate_estate(self, u):
        B = np.array([
                        [self.DT * cos(self.Xt[2]), 0],
                        [self.DT * sin(self.Xt[2]), 0],
                        [0.0, self.DT],
                        [1.0, 0.0]
                    ])
        x =  np.dot(self.A, self.Xt) + np.dot(B, u)
        return x

    def innovation(self, Xbel, z):
        """
        es la diferencia entre la medición actual
        y la medición estimada
        """
        zEst = z - np.dot(self.C, Xbel)
        return zEst
        
    def estimate_sigma(self, Xbel, u):
        """
        estima en t actual la varianza en t+1 
        """
        A = self.linear_model(Xbel, u)
        sigma = A @ self.Sigma_t @ A.T +  self.R
        return sigma

    def predict_sigma(self, Sigma_bel, K):
        """
        predice la varianza en t+1 una una vez calculada la ganancia
        """
        sigma = (np.identity(4) - K @ self.C) @ Sigma_bel
        return sigma
            
    def kalman_gain(self, Sigma_bel):
        S = self.Q + self.C @ Sigma_bel @ self.C.T
        k = Sigma_bel @ self.C.T @ np.linalg.inv(S)
        return k

    def predict_state(self, Xbel, K, z):
        """
        usa la ganancia de kalman para predicir
        la posición en t+1 a partir de la estimación
        """
        x = Xbel + np.dot(K, self.innovation(Xbel, z))
        return x
    
    def linear_model(self, Xbel, u): 
        yaw = Xbel[2]
        v = u[0]
        jF = np.array([
                [1.0, 0.0, -self.DT * v * sin(yaw), self.DT * cos(yaw)],
                [0.0, 1.0, self.DT * v * cos(yaw), self.DT * sin(yaw)],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0]
            ], np.float64)

        return jF

    def estimate_next_pos(self, z, u):
    
        # calculo estado estimado con posición actual y control
        Xbel = self.estimate_estate(u) 

        # calculo varianza estimada
        sigma_bel = self.estimate_sigma(Xbel, u) # Sigma_t/t

        # calculo ganancia de kalman
        K =  self.kalman_gain(sigma_bel)

        # calculo el estado para t+1 (predicción)
        Xpred = self.predict_state(Xbel, K, z)

        # calculo varianza con la nueva estimación
        self.Sigma_t = self.predict_sigma(sigma_bel, K)  #Sigma_t+1/t+1

        #guardo y retorno la predicción
        self.Xt = Xpred
        return Xpred


