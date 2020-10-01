import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
from rocketpy import Function

# Ennvironment
rho = 1.09

# Desired Parameters
timeMax = 3
thetaDotMax = 9 #given in radians per second
thetaDotDotMax = (0 - thetaDotMax)/timeMax

# Europa
J = 0.077
V0 = 25
r = 127/2000
Lr = 2 * r
Ar = np.pi * r**2

# Fin set
N = 4
root = 0.15
tip = 0.06
span = 0.15
delta = 2 * np.pi / 180

# Canards set 
n = 4
Cr = 60 / 1000
Ct = 60 / 1000
s = 100 / 1000
alfa = 11 * np.pi / 180

YtCanard = (r + s/3 * ((Cr + 2 * Ct) / (Cr + Ct))) # it is the Canard's "arm"
YtFins = (r + span/3 * ((root + 2 * tip) / (root + tip))) # it is the Fin's "arm"

def cnalfa(cnalfa0, Cr, Ct, s, r, N):
    Aref = np.pi * r**2
    Af = (Cr + Ct) * s / 2; # fin area
    AR= 2 * (s**2) / Af
    gamac = np.arctan( (Cr - Ct) / (2 * s) ); # mid chord angle
    FD = 2 * np.pi * AR / (cnalfa0 *np.cos(gamac))
    return cnalfa0 * FD * (Af/Aref) * np.cos(gamac) * (N/2) / (2 + FD * ( 1 + (4/FD**2) )**0.5) 


NACA0012 = genfromtxt('Data/Airfoils/NACA0012_Cl_em_radianos.csv', delimiter = ',')
#cnalfa0_func = Function(NACA0012)

# Aplies number of fins to lift coefficient data
dataCanard = [ [cl[0], cnalfa(cl[1], Cr, Ct, s, r, n)] for cl in NACA0012 ]
cldataCanard = Function(dataCanard, 'Alpha (rad)', 'Cl', interpolation='linear', extrapolation = 'natural') # Cnalfa1 for each canard

dataFins = [ [cl[0], cnalfa(cl[1], root, tip, span, r, N)] for cl in NACA0012 ]
cldataFins = Function(dataFins, 'Alpha (rad)', 'Cl', interpolation='linear', extrapolation = 'natural')

A_min = -2 * thetaDotDotMax * J * Lr / (rho * V0**2 * Ar) +  cldataFins(delta) * YtFins
print('A_min =', A_min)

A = cldataCanard(alfa) * YtCanard
print('A =', A)

plt.plot([[cl[0]] for cl in dataCanard], [[cl[1] * YtCanard] for cl in dataCanard] , [[cl[0]] for cl in dataFins], [[cl[1] * YtFins] for cl in dataFins])
plt.title('Cm fins vs Cm canards')
plt.xlabel('Alfa')
plt.ylabel('Cm')
plt.show()
#print(cldataCanard.differentiate(x=0, dx=1e-2))
#cldataCanard.plot()
