import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from rocketpy import Function

df = pd.read_csv(r'Lift coeff completo.csv')

df.dropna(inplace = True)

# Ennvironment
rho = 1.09

# Desired Parameters
timeMax = 1
thetaDotMax = 36 #given in radians per second
thetaDotDotMax = (0 - thetaDotMax)/timeMax

# Europa
J = 0.077
r = 127/2000

# Fin set
N = 3
root = 77 / 1000
tip = 47 / 1000
span = 104 / 1000
delta = 2 # Angulo de ataque para as aletas

# Canards set 
n = 3
Cr = 77 / 1000
Ct = 47 / 1000 #O tip é do tamanho do root para aproveitar que quanto mais longe do foguete, maior é o braço do momento
s = 104 / 1000
arm = 0/1000 # Braço entre a aleta e a fuselagem
alfa = 2 # angulo de ataque máximo para as canards

class Canards:
    '''
    N = numero de aletas
    Root = Root chordd
    Tip = Tip Chord
    Span = Span da aleta
    radius = raio do fouguete
    arm = o braço da aleta
    Aref = área de referência do foguete
    Afins = área das aletas (ou canards)
    AR = razão de aspecto
    gamac = algum ângulo calcualdo com o OPK
    YtFins = centro de pressão das aletas
    '''
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
        '''data must be a pandas dataframe
        Retirado da tabela, propriedades de lift do aerofólio'''
        data.dropna(inplace = True)
        self.Cnalfa0 = data

class Compare:
    def __init__(self, canard1, canard2):
        self.canard1 = canard1
        self.canard2 = canard2

    def calculate_cnalfaT(self, cnalfa0, canard):
        # Check expressions 2.12 and 2.14

        FD = 2 * np.pi * canard.AR / (cnalfa0 * np.cos(canard.gamac))
        return cnalfa0 * FD * (canard.Afins/canard.Aref) * np.cos(canard.gamac) / (2 + FD * ( 1 + (4/FD**2) )**0.5) 

    def plot_coeff_curves(self, speed = 0.10, mode = 'lift'):
        if (10 * speed) % 1 == 0:
            Airfoil1 = np.array(self.canard1.Cnalfa0["M=" + str(speed) + str(0)])
            Airfoil2 = np.array(self.canard2.Cnalfa0["M=" + str(speed) + str(0)])
        else:
            Airfoil1 = np.array(self.canard1.Cnalfa0["M=" + str(speed)])
            Airfoil2 = np.array(self.canard2.Cnalfa0["M=" + str(speed)])
        if mode == 'lift':
            # Aplies number of fins to lift coefficient data
            angles1 = [ i/2 for i in range(len(Airfoil1)) ]
            dataCanard1 = [ self.calculate_cnalfaT(float(Airfoil1[i]), self.canard1) for i in range(len(Airfoil1)) ]
            angles2 = [ i/2 for i in range(len(Airfoil2)) ]
            dataCanard2 = [ self.calculate_cnalfaT(float(Airfoil2[i]), self.canard2) for i in range(len(Airfoil2)) ]


            plt.plot(angles1, dataCanard1 , angles2, dataCanard2)
            plt.title('Cl fins vs Cl canards')
            plt.xlabel('Alfa')
            plt.ylabel('Cl')
            plt.show()

        if mode == 'forcing':
            # Aplies number of fins to lift coefficient data
            angles1 = [ i/2 for i in range(len(Airfoil1)) ]
            dataCanard1 = [ self.canard1.YtFins * self.calculate_cnalfaT(float(Airfoil1[i]), self.canard1) / (2 * self.canard2.radius) for i in range(len(Airfoil1)) ]
            angles2 = [ i/2 for i in range(len(Airfoil2)) ]
            dataCanard2 = [ self.canard2.YtFins * self.calculate_cnalfaT(float(Airfoil2[i]), self.canard2) / (2 * self.canard2.radius) for i in range(len(Airfoil2)) ]
            

            plt.plot(angles1, dataCanard1 , angles2, dataCanard2)
            plt.title('Clf fins vs Clf canards')
            plt.xlabel('Delta')
            plt.ylabel('Clf')
            plt.show()

    def calculateA(self, speed, stall_angle, fin_delta_angle, J, thetaDotDotMax, thetaDot0 = 0, dynamicPressure = 3.105e+04):
        '''The canards set must come before the fins set'''
        # condition to find the index at the panda dataset
        Airfoil1 = np.array(self.canard1.Cnalfa0["M={:.2f}".format(speed)])
        Airfoil2 = np.array(self.canard2.Cnalfa0["M={:.2f}".format(speed)])

        data0Canard1 = [ [i/2, float(Airfoil1[i])] for i in range(len(Airfoil1)) ]
        data0Canard2 = [  [i/2, float(Airfoil2[i])] for i in range(len(Airfoil2)) ]

        # Generates the Cnalfa0 as a Function object
        cn0dataCanard1 = Function(data0Canard1, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural') # Cnalfa1 for each canard set
        cn0dataCanard2 = Function(data0Canard2, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural')

        # Map the cnalfaT for the 3d fin, taking as base the cnalfa0 of the airfoil and the fin parameters
        dataCanard1 = [ [i/2, self.calculate_cnalfaT(float(Airfoil1[i]), self.canard1)] for i in range(len(Airfoil1)) ]
        dataCanard2 = [  [i/2, self.calculate_cnalfaT(float(Airfoil2[i]), self.canard2)] for i in range(len(Airfoil2)) ]

        # Converts lift coefficient data to a Function object
        cndataCanard1 = Function(dataCanard1, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural') # Cnalfa1 for each canard set
        cndataCanard2 = Function(dataCanard2, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural')

        # Calculate the canards lift coefficient - formula 2.28
        ClfCanards = 2 * self.canard1.YtFins * self.calculate_cnalfaT(float(Airfoil1[2 * stall_angle]), self.canard1) / (2 * self.canard1.radius)
        #ClfCanards = 2 * self.canard1.YtFins * cndataCanard1.differentiate(x = 1e-2, dx = 1e-1) / (2 * self.canard1.radius) 
        
        # Calculation of constants for the damping coefficient - formula 2.32
        c1 = ((self.canard1.YtFins) /  2) * (self.canard1.radius**2) * self.canard1.span
        c2 = ((self.canard1.root + 2 * self.canard1.tip) / 3) * self.canard1.radius * (self.canard1.span**2)
        c3 = ((self.canard1.root + 3 * self.canard1.tip) / 12) * (self.canard1.span**3)
        trapezoidal_constant_canards = c1 + c2 + c3

        # Está errado, precisa corrigir para a nova fórmula qeu encontramos
        thetaDot0 = 1
        CldCanards = self.canard1.N * cn0dataCanard1.differentiate(x = 1e-2, dx = 1e-1) * thetaDot0 * trapezoidal_constant_canards/ (self.canard1.Aref * self.canard1.radius * 2 * speed * 343)
        
        print("CldC = ", CldCanards)
        print("ClfC = ", ClfCanards)
        A = 100 * (ClfCanards + CldCanards)

        # Repeat the process to the Amin
        ClfFins = 2 * self.canard2.YtFins * self.calculate_cnalfaT(float(Airfoil2[2 * fin_delta_angle]), self.canard2) / (2 * self.canard2.radius) 
        
        b1 = ((self.canard2.YtFins) /  2) * (self.canard2.radius**2) * self.canard2.span
        b2 = ((self.canard2.root + 2 * self.canard2.tip) / 3) * self.canard2.radius * (self.canard2.span**2)
        b3 = ((self.canard2.root + 3 * self.canard2.tip) / 12) * (self.canard2.span**3)
        trapezoidal_constant_fins = b1 + b2 + b3

        CldFins = self.canard2.N * cn0dataCanard2.differentiate(x = 1e-2, dx = 1e-3) * thetaDot0 * trapezoidal_constant_fins/ (self.canard2.Aref * self.canard2.radius * 2 * speed * 343)
        

        Amin = 100 * (-J * thetaDotDotMax / (dynamicPressure * self.canard2.Aref * 2 * self.canard2.radius) + ClfFins - CldFins)

        print("A = ", A)
        print("Amin = ", Amin)

Cana = Canards(n, Cr, Ct, s, r, arm)
Cana.setCnalfa0(df)
Fin = Canards(N, root, tip, span, r)
Fin.setCnalfa0(df)

Comp = Compare(Cana, Fin)

#Comp.plot_coeff_curves(mode='forcing', speed=0.7)
Comp.calculateA(speed = 0.6, stall_angle= 6, fin_delta_angle= delta, J=J, thetaDotDotMax = thetaDotDotMax, thetaDot0=0, dynamicPressure=3.105e+04)
