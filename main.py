
from math import pi
from simulation import run_simulation_catch ,run_simulation_pos

def main():
    print("PROBLEMA 1")
    print("Predicción de la próxima posición del robot con sensores y control sin ruido")
    run_simulation_pos(0.0, 0.0, 0.0, 1e-12,'problema 1') # sin ruido de medición

    print("PROBLEMA 2")
    print("Predicción de la próxima posición del robot con sensores con ruido y control sin ruido")
    run_simulation_pos(0.02, 0.02, 0.01*(2*pi), 1e-3, 'problema 2') # con ruido de medición

    # resuelvo los puntos 3 y 4 en la misma simulación
    print("PROBLEMA 3 y 4")
    print("Atrapar al robot")
    run_simulation_catch(0.02, 0.02, 0.01*(2*pi)) # con ruido de medición

if __name__ == '__main__':
    main()