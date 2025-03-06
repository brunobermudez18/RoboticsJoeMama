import sympy as sp
from sympy.matrices import rot_axis3

# Para poder graficar
import matplotlib.pyplot as plt
import numpy as np

# Para generar la matriz DH
from spatialmath import *
from spatialmath.base import *

# Definir los símbolos
theta, d, a, alpha = sp.symbols('theta, d, a, alpha')

# Matriz RzTzTxRx
TDH = sp.Matrix( trotz(theta) @ transl(0,0,d) @ transl(a,0,0) @ trotx(alpha))

sp.pprint(TDH)
print(type(TDH))

# Definir los valores de theta individuales
theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = sp.symbols('theta_1, theta_2, theta_3, theta_4, theta_5, theta_6')

# Sustitución de valores según la tabla DH
T01 = TDH.subs({d: 0.680, a: 0.200, alpha: -sp.pi/2})
T01 = T01.subs({theta: theta_1})
sp.pprint(T01)

# Sustitución para el segundo frame con su offset
T12 = TDH.subs({d: 0, a: 0.890, alpha: 0})  # theta_2 - sp.pi/2
T12 = T12.subs({theta: theta_2})
sp.pprint(T12)
