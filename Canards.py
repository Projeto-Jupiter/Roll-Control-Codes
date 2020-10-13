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
root = 100 / 1000
tip = 20 / 1000
span = 90 / 1000
delta = 3

# Canards set 
n = 4
Cr = 40 / 1000
Ct = 40 / 1000
s = 60 / 1000
arm = 10/1000
alfa = 11

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
        return canard.N/2 * cnalfa0 * FD * (canard.Afins/canard.Aref) * np.cos(canard.gamac) / (2 + FD * ( 1 + (4/FD**2) )**0.5) 

    def plot_coeff_curves(self, speed = 0.10, mode = 'lift'):
        if speed % 0.10 == 0:
            Airfoil1 = np.array(self.canard1.Cnalfa0["M=" + str(speed) + str(0)])
            Airfoil2 = np.array(self.canard2.Cnalfa0["M=" + str(speed) + str(0)])
        else:
            Airfoil1 = np.array(self.canard1.Cnalfa0["M=" + str(speed)])
            Airfoil2 = np.array(self.canard2.Cnalfa0["M=" + str(speed)])
        if mode == 'lift':
            # Aplies number of fins to lift coefficient data
            angles1 = [ i/2 for i in range(len(Airfoil1)) ]
            dataCanard1 = [ self.calculate_cnalfa(float(Airfoil1[i]), self.canard1) for i in range(len(Airfoil1)) ]
            angles2 = [ i/2 for i in range(len(Airfoil2)) ]
            dataCanard2 = [ self.calculate_cnalfa(float(Airfoil2[i]), self.canard2) for i in range(len(Airfoil2)) ]


            plt.plot(angles1, dataCanard1 , angles2, dataCanard2)
            plt.title('Cl fins vs Cl canards')
            plt.xlabel('Alfa')
            plt.ylabel('Cl')
            plt.show()

        if mode == 'forcing':
            # Aplies number of fins to lift coefficient data
            angles1 = [ i/2 for i in range(len(Airfoil1)) ]
            dataCanard1 = [ self.canard1.YtFins * self.calculate_cnalfa(float(Airfoil1[i]), self.canard1) / (2 * self.canard2.radius) for i in range(len(Airfoil1)) ]
            angles2 = [ i/2 for i in range(len(Airfoil2)) ]
            dataCanard2 = [ self.canard2.YtFins * self.calculate_cnalfa(float(Airfoil2[i]), self.canard2) / (2 * self.canard2.radius) for i in range(len(Airfoil2)) ]
            

            plt.plot(angles1, dataCanard1 , angles2, dataCanard2)
            plt.title('Clf fins vs Clf canards')
            plt.xlabel('Delta')
            plt.ylabel('Clf')
            plt.show()

    def calculateA(self, speed, stall_angle, fin_delta_angle, J, thetaDotDotMax, thetaDot0 = 0, dynamicPressure = 3.105e+04):
        '''The canards set must come before the fins set'''
        if speed % 0.10 == 0:
            Airfoil1 = np.array(self.canard1.Cnalfa0["M=" + str(speed) + str(0)])
            Airfoil2 = np.array(self.canard2.Cnalfa0["M=" + str(speed) + str(0)])
        else:
            Airfoil1 = np.array(self.canard1.Cnalfa0["M=" + str(speed)])
            Airfoil2 = np.array(self.canard2.Cnalfa0["M=" + str(speed)])

        dataCanard1 = [ [i/2, self.calculate_cnalfa(float(Airfoil1[i]), self.canard1)] for i in range(len(Airfoil1)) ]
        dataCanard2 = [  [i/2, self.calculate_cnalfa(float(Airfoil2[i]), self.canard2)] for i in range(len(Airfoil2)) ]

        # Converts lift coefficient data to a Function object
        cndataCanard1 = Function(dataCanard1, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural') # Cnalfa1 for each canard set
        cndataCanard2 = Function(dataCanard2, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural')

        ClfCanards = 2 * self.canard1.YtFins * self.calculate_cnalfa(float(Airfoil1[2 * stall_angle]), self.canard1) / (2 * self.canard1.radius) 
        
        c1 = ((self.canard1.YtFins) /  2) * (self.canard1.radius**2) * self.canard1.span
        c2 = ((self.canard1.root + 2 * self.canard1.tip) / 3) * self.canard1.radius * (self.canard1.span**2)
        c3 = ((self.canard1.root + 3 * self.canard1.tip) / 12) * (self.canard1.span**3);
        trapezoidal_constant_canards = c1 + c2 + c3

        CldCanards = self.canard1.N * cndataCanard1.differentiate(x = 1e-2, dx = 1e-3) * thetaDot0 * trapezoidal_constant_canards/ (self.canard1.Aref * self.canard1.radius * 2 * speed * 343)
        A = 100 * (ClfCanards + CldCanards)

        # Repeat the process to the Amin
        ClfFins = 2 * self.canard2.YtFins * self.calculate_cnalfa(float(Airfoil2[2 * fin_delta_angle]), self.canard2) / (2 * self.canard2.radius) 
        
        b1 = ((self.canard2.YtFins) /  2) * (self.canard2.radius**2) * self.canard2.span
        b2 = ((self.canard2.root + 2 * self.canard2.tip) / 3) * self.canard2.radius * (self.canard2.span**2)
        b3 = ((self.canard2.root + 3 * self.canard2.tip) / 12) * (self.canard2.span**3);
        trapezoidal_constant_fins = b1 + b2 + b3

        CldFins = self.canard2.N * cndataCanard2.differentiate(x = 1e-2, dx = 1e-3) * thetaDot0 * trapezoidal_constant_fins/ (self.canard2.Aref * self.canard2.radius * 2 * speed * 343)
        
        Amin = 100 * (-J * thetaDotDotMax / (dynamicPressure * self.canard2.Aref * 2 * self.canard2.radius) + ClfFins - CldFins)

        print("A = ", A)
        print("Amin = ", Amin)

Cana = Canards(n, Cr, Ct, s, r, arm)
Cana.setCnalfa0(df)
Fin = Canards(N, root, tip, span, r)
Fin.setCnalfa0(df)

Comp = Compare(Cana, Fin)

#Comp.plot_coeff_curves(mode='forcing')
Comp.calculateA(speed=0.10, stall_angle= alfa, fin_delta_angle= delta, J=J, thetaDotDotMax = thetaDotDotMax, dynamicPressure=3.105e+04)
