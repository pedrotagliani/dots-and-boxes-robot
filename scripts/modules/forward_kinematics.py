from spatialmath import *
from spatialmath.base import *
from math import pi, degrees, radians
import numpy as np
import sympy as sp

# Variables relacionadas a cada una de las longitudes intervinientes en los cálculos (todo en centímetros)
l1 = 14.085
l2 = 12.725
l3 = 10.222
l4 = 11.200
l5 = 8.400

def forward_kinematics(q): # Los valores de las variables articulares están en grados

    # Se pasan los ángulos de grados a radianes
    q1 = radians(q[0])
    q2 = radians(q[1])
    q3 = radians(q[2])
    q4 = radians(q[3])

    # Con la tabla de parámetros DH ya completada, se arman cada una de las MTH:
    A01 = SE3.Rz(q1  - radians(90)) * SE3.Tz(l1) * SE3.Tx(0) * SE3.Rx(radians(90))
    A12 = SE3.Rz(q2) * SE3.Tz(0) * SE3.Tx(l2) * SE3.Rx(0)
    A23 = SE3.Rz(q3  - radians(126)) * SE3.Tz(0) * SE3.Tx(l3) * SE3.Rx(0)
    A34 = SE3.Rz(q4) * SE3.Tz(0) * SE3.Tx(l4) * SE3.Rx(0)
    A45 = SE3.Ty(-l5) # Se traslada hacia la punta del marcador
    T = A01 * A12 * A23 * A34 * A45

    return T

if __name__ == '__main__':
    # Ejemplo de aplicación
    q1 = 90.0
    q2 = 80.0
    q3 = 0.0
    q4 = 120.0

    q = [q1, q2, q3, q4]

    T1 = forward_kinematics(q)

    print(T1)