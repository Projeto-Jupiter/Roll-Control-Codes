import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rocketpy import Function
from mpl_toolkits.mplot3d import Axes3D

df = pd.read_csv(r'Lift coeff completo.csv')

df.dropna(inplace = True)

# Ennvironment
rho = 1.09

# Desired Parameters
timeMax = 3
thetaDotMax = 9 #given in radians per second
thetaDotDotMax = (0 - thetaDotMax)/timeMax

# Europa
J = 0.077
r = 127/2000

# Fin set
N = 4
root = 0.1
tip = 0.02
span = 0.09
delta = 2 * np.pi / 180

# Canards set 
n = 4
Cr = 50 / 1000
Ct = 50 / 1000
s = 110 / 1000
arm = 10/1000
alfa = 11 * np.pi / 180

class Canards:
    def __init__(self, N, root, tip, span, radius, arm=0):
        self.N = N
        self.root = root
        self.tip = tip
        self.span = span
        self.radius = radius
        self.arm = arm
        self.Aref =np.pi * radius**2    
        self.Afins = (root + tip) * span / 2; # fin area
        self.AR= 2 * (span**2) / self.Afins #Aspec ratio
        self.gamac = np.arctan( (root - tip) / (2 * span) ); # mid chord angle
        self.YtFins = (radius + span/3 * ((root + 2 * tip) / (root + tip))) # it is the Fin's center of pressure location

    def setCnalfa0(self, data):
        '''data must be a pandas dataframe'''
        data.dropna(inplace = True)
        self.Cnalfa0 = data

class Compare(Canards):
    def __init__(self, canard1, canard2):
        self.canard1 = canard1
        self.canard2 = canard2

    def calculate_cnalfa(self, cnalfa0, canard):
        FD = 2 * np.pi * canard.AR / (cnalfa0 *np.cos(canard.gamac))
        return canard.N/2 * cnalfa0 * FD * (canard.Afins/canard.Aref) * np.cos(canard.gamac) * (canard.N/2) / (2 + FD * ( 1 + (4/FD**2) )**0.5) 

    def plot_lift_curve(self, speed = 0.10):
        Airfoil1 = np.array(self.canard1.Cnalfa0["M=" + str(speed)])
        Airfoil2 = np.array(self.canard2.Cnalfa0["M=" + str(speed)])

        # Aplies number of fins to lift coefficient data
        angles1 = [ i/2 for i in range(len(Airfoil1)) ]
        dataCanard1 = [ self.calculate_cnalfa(float(Airfoil1[i]), self.canard1) for i in range(len(Airfoil1)) ]
        angles2 = [ i/2 for i in range(len(Airfoil2)) ]
        dataCanard2 = [ self.calculate_cnalfa(float(Airfoil2[i]), self.canard2) for i in range(len(Airfoil2)) ]


        plt.plot(angles1, dataCanard1 , angles2, dataCanard2)
        plt.title('Cm fins vs Cm canards')
        plt.xlabel('Alfa')
        plt.ylabel('Cm')
        plt.show()

    def calculateA(self, speed, stall_angle, fin_delta_angle, J, thetaDotDotMax, rho = 1.09):
        if speed % 0.10 == 0:
            Airfoil1 = np.array(self.canard1.Cnalfa0["M=" + str(speed) + str(0)])
            Airfoil2 = np.array(self.canard2.Cnalfa0["M=" + str(speed) + str(0)])
        else:
            Airfoil1 = np.array(self.canard1.Cnalfa0["M=" + str(speed)])
            Airfoil2 = np.array(self.canard2.Cnalfa0["M=" + str(speed)])

        dataCanard1 = [ [i/2, self.calculate_cnalfa(float(Airfoil1[i]), self.canard1)] for i in range(len(Airfoil1)) ]
        dataCanard2 = [  [i/2, self.calculate_cnalfa(float(Airfoil2[i]), self.canard2)] for i in range(len(Airfoil2)) ]

        # Aplies number of fins to lift coefficient data
        cndataCanard1 = Function(dataCanard1, 'Alpha (rad)', 'Cl', interpolation='linear', extrapolation = 'natural') # Cnalfa1 for each canard
        cndataCanard2 = Function(dataCanard2, 'Alpha (rad)', 'Cl', interpolation='linear', extrapolation = 'natural')

        A_min = -2 * thetaDotDotMax * J * (self.canard2.radius * 2) / (rho * (speed * 343)**2 * self.canard2.Aref) +  cndataCanard2(fin_delta_angle) * self.canard2.YtFins
        print('A_min =', A_min)

        A = cndataCanard1(stall_angle) * self.canard1.YtFins
        print('A =', A)


Cana = Canards(n, Cr, Ct, s, r, arm)
Cana.setCnalfa0(df)
Fin = Canards(N, root, tip, span, r)
Fin.setCnalfa0(df)

Comp = Compare(Cana, Fin)

#Comp.plot_lift_curve()
Comp.calculateA(0.10, alfa, delta, J, thetaDotDotMax, rho)
