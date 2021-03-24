import pandas as pd
import numpy as np 
import math
import matplotlib.pyplot as plt
from rocketpy import Function



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
        self.Aref =math.pi * radius**2    
        self.Afins = (root + tip) * span / 2; # fin area
        self.AR= 2 * (span**2) / self.Afins #Aspec ratio
        self.gamac = math.atan( (root - tip) / (2 * span) ); # mid chord angle
        self.YtFins = (radius + arm + span/3 * ((root + 2 * tip) / (root + tip))) # it is the Fin's center of pressure location

    def setCN0(self, data):
        '''data must be a pandas dataframe
        Retirado da tabela, propriedades de lift do aerofólio'''
        data.dropna(inplace = True)
        self.CN0 = data

class Compare:
    def __init__(self, canard1, canard2):
        self.canard1 = canard1
        self.canard2 = canard2

    def calculate_cnalfa1(self, CN0, canard):
        if CN0 == 0:
            return 0
        # Check expressions 2.12 and 2.14

        FD = 2 * math.pi * canard.AR / (CN0 * math.cos(canard.gamac))
        return CN0 * FD * (canard.Afins/canard.Aref) * math.cos(canard.gamac) / (2 + FD * math.sqrt( 1 + (4/FD**2) )) 

    def plot_coeff_curves(self, speed = 0.10, mode = 'lift'):
        if (10 * speed) % 1 == 0:
            Airfoil1 = np.array(self.canard1.CN0["M=" + str(speed) + str(0)])
            Airfoil2 = np.array(self.canard2.CN0["M=" + str(speed) + str(0)])
        else:
            Airfoil1 = np.array(self.canard1.CN0["M=" + str(speed)])
            Airfoil2 = np.array(self.canard2.CN0["M=" + str(speed)])
        if mode == 'lift':
            # Generate a list with the attack angle with a step of 0.5. 
            angles1 = [ i/2 for i in range(len(Airfoil1)) ]

            # Generate a list with the CNt (total normal force coefficient). Note that it is already multiplied by the canard angle
            dataCanard1 = [ 2 * self.calculate_cnalfa1(float(Airfoil1[i]), self.canard1) for i in range(len(Airfoil1)) ] # The 2 in the calculation comes from formula 2.13 when considering the wind flow hitting the cannards perpendicularly
            angles2 = [ i/2 for i in range(len(Airfoil2)) ]
            dataCanard2 = [ (self.canard2.N/2) * self.calculate_cnalfa1(float(Airfoil2[i]), self.canard2) for i in range(len(Airfoil2)) ]


            plt.plot(angles1, dataCanard1 , angles2, dataCanard2)
            plt.title('Cl fins vs Cl canards')
            plt.xlabel('Alfa')
            plt.ylabel('Cl')
            plt.show()

        if mode == 'forcing':
            # Aplies number of fins to lift coefficient data
            angles1 = [ i/2 for i in range(len(Airfoil1)) ]
            dataCanard1 = [ self.canard1.YtFins * self.canard1.N * self.calculate_cnalfa1(float(Airfoil1[i]), self.canard1) / (2 * self.canard1.radius) for i in range(len(Airfoil1)) ]
            angles2 = [ i/2 for i in range(len(Airfoil2)) ]
            dataCanard2 = [ self.canard2.YtFins * self.canard2.N * self.calculate_cnalfa1(float(Airfoil2[i]), self.canard2) / (2 * self.canard2.radius) for i in range(len(Airfoil2)) ]
            

            plt.plot(angles1, dataCanard1 , angles2, dataCanard2)
            plt.title('Clf fins vs Clf canards')
            plt.xlabel('Delta')
            plt.ylabel('Clf')
            plt.show()

    def calculateA(self, speed, stall_angle, fin_delta_angle, J, thetaDotDotMax, thetaDot0 = 0, DynamicPressure = 101325, compare = False, speedMin = 0.1, speedMax = 0.8, passo = 0.1, tempo = 3):
        '''The canards set must come before the fins set'''

        print('---------------------------------------------------------------------')
        # Condition to find the index at the panda dataset
        Airfoil1 = np.array(self.canard1.CN0)
        Airfoil2 = np.array(self.canard2.CN0)
        stall_angle = stall_angle * math.pi / 180
        fin_delta_angle = fin_delta_angle * math.pi / 180

        data0Canard1 = [ [float(Airfoil1[i][0]), float(Airfoil1[i][1])] for i in range(len(Airfoil1)) ]
        data0Canard2 = [  [float(Airfoil1[i][0]), float(Airfoil2[i][1])] for i in range(len(Airfoil2)) ]

        # Generates the CN0 as a Function object
        cn0dataCanard1 = Function(data0Canard1, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural') # Cnalfa1 for each canard set
        cn0dataCanard2 = Function(data0Canard2, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural')

        # Map the cnalfa1 for the 3d fin, taking as base the CN0 of the airfoil and the fin parameters
        dataCanard1 = [ [Airfoil1[i][0], self.calculate_cnalfa1(float(Airfoil1[i][1]), self.canard1)] for i in range(len(Airfoil1)) ]
        dataCanard2 = [  [Airfoil2[i][0], self.calculate_cnalfa1(float(Airfoil2[i][1]), self.canard2)] for i in range(len(Airfoil2)) ]

        # Converts lift coefficient data to a Function object
        cndataCanard1 = Function(dataCanard1, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural') # Cnalfa1 for each canard set
        cndataCanard2 = Function(dataCanard2, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural')

        # Calculate the canards lift coefficient - formula 2.28
        MaxClfCanards = self.canard1.N * self.canard1.YtFins * cndataCanard1(stall_angle) / (2 * self.canard1.radius)
        ClfCanardsDerivative = self.canard1.N * self.canard1.YtFins * cndataCanard1.differentiate(x = 1e-2, dx = 1e-1) / (2 * self.canard1.radius) 
        
        # Calculation of constants for the damping coefficient - formula 2.32
        c1 = ((self.canard1.YtFins) /  2) * (self.canard1.radius**2) * self.canard1.span
        c2 = ((self.canard1.root + 2 * self.canard1.tip) / 3) * self.canard1.radius * (self.canard1.span**2)
        c3 = ((self.canard1.root + 3 * self.canard1.tip) / 12) * (self.canard1.span**3)
        trapezoidal_constant_canards = c1 + c2 + c3

        # Esse resultado é relativo porque depende da velocidade, ou seja, dependendo do ponto o damping muda...
        V0 = speed * 343
        CldCanards = self.canard1.N * cn0dataCanard1.differentiate(x = 1e-2, dx = 1e-1) * thetaDot0 * trapezoidal_constant_canards/ (self.canard1.Aref * 2 * self.canard1.radius * V0)
        CldCanardsDerivative = self.canard1.N * cn0dataCanard1.differentiate(x = 1e-2, dx = 1e-1) * trapezoidal_constant_canards/ (self.canard1.Aref * 2 * self.canard1.radius * V0)

        print("ClfC at given angle = {:.3f}".format(MaxClfCanards))
        print("CldC at given omega and speed = {:.3f}".format(CldCanards))
        print()
        print("ClfC Derivative with Delta = {:.3f}".format(ClfCanardsDerivative))
        print("CldC Derivative with Omega at given speed = {:.3f}".format(CldCanardsDerivative))
        
        A = 100 * (MaxClfCanards + abs(CldCanards))

        # Repeat the process to the Amin
        ClfFins = self.canard2.N * self.canard2.YtFins * cndataCanard2(fin_delta_angle) / (2 * self.canard2.radius) 
        ClfFinsDerivative = self.canard2.N * self.canard2.YtFins * cndataCanard2.differentiate(x = 1e-2, dx = 1e-1) / (2 * self.canard2.radius) 

        b1 = ((self.canard2.YtFins) /  2) * (self.canard2.radius**2) * self.canard2.span
        b2 = ((self.canard2.root + 2 * self.canard2.tip) / 3) * self.canard2.radius * (self.canard2.span**2)
        b3 = ((self.canard2.root + 3 * self.canard2.tip) / 12) * (self.canard2.span**3)
        trapezoidal_constant_fins = b1 + b2 + b3

        CldFins = self.canard2.N * cn0dataCanard2.differentiate(x = 1e-2, dx = 1e-1) * thetaDot0 * trapezoidal_constant_fins/ (self.canard2.Aref * self.canard2.radius * 2 * V0)
        CldFinsDerivative = self.canard2.N * cn0dataCanard2.differentiate(x = 1e-2, dx = 1e-1) * trapezoidal_constant_fins/ (self.canard2.Aref * self.canard2.radius * 2)
        
        print()
        print("ClfA at given angle = {:.3f}".format(ClfFins))
        print("ClfA Derivative with Delta= {:.3f}".format(ClfFinsDerivative))
        print()
        print("CldA at given omega and speed = {:.3f}".format(CldFins))
        print("CldA Derivative with Omega at given speed = {:.3f}".format(CldFinsDerivative))
        

        Amin = 100 * ( -((J * thetaDotDotMax) / (DynamicPressure * self.canard2.Aref * 2 * self.canard2.radius)) + ClfFins - CldFins)

        print()
        print("A = {:.2f}".format(A))
        print("Amin = {:.2f}".format(Amin))

        if compare:
            LA = []
            LAmin = []
            LSpeed = []
            
            V = speedMin

            while V <= speedMax:
                DynamicPressure = (343 * V)**2 * rho/2
                omega = self.canard2.Aref * np.sqrt(abs(1-V)) * self.canard2.YtFins * cndataCanard2(fin_delta_angle) * V * 343 / ( 2 * np.pi * trapezoidal_constant_fins)
                temp = self.calculateA(V, stall_angle * 180 / np.pi, fin_delta_angle * 180 / np.pi, J, omega / tempo, omega, DynamicPressure)
                LA.append(temp[0])
                LAmin.append(temp[1])
                LSpeed.append(temp[2])
                V += passo
                

            plt.plot(LSpeed, LA, label = "A")
            plt.plot(LSpeed, LAmin, label = "Amin")
            plt.title('A vs Amin')
            plt.xlabel('Speed')
            plt.ylabel('A/Amin')
            plt.legend()
            plt.show()

        return [A, Amin, speed]
    def torque(self, speed, XCgFins, stall_angle, rho = 1.06):
        # Condition to find the index at the panda dataset
        Airfoil1 = np.array(self.canard1.CN0)
        stall_angle = stall_angle * math.pi / 180
        speed *= 343

        data0Canard1 = [ [float(Airfoil1[i][0]), float(Airfoil1[i][1])] for i in range(len(Airfoil1)) ]

        # Generates the CN0 as a Function object
        cn0dataCanard1 = Function(data0Canard1, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural') # Cnalfa1 for each canard set

        # Map the cnalfa1 for the 3d fin, taking as base the CN0 of the airfoil and the fin parameters
        dataCanard1 = [ [Airfoil1[i][0], self.calculate_cnalfa1(float(Airfoil1[i][1]), self.canard1)] for i in range(len(Airfoil1)) ]

        # Converts lift coefficient data to a Function object
        cndataCanard1 = Function(dataCanard1, 'Alpha (rad)', 'Cn', interpolation='linear', extrapolation = 'natural') # Cnalfa1 for each canard set

        # Calculate the canards lift coefficient - formula 2.28
        MaxClfCanards = self.canard1.N * self.canard1.YtFins * cndataCanard1(stall_angle) / (2 * self.canard1.radius)

        # Damping force was disconsidered since it reduces the applied torque
        MaxNormalForce = MaxClfCanards * (1/2) * rho * speed**2 * self.canard1.Aref

        XFins = 

        MaxTorque = MaxNormalForce * (XFins - XCgFins)

        print("Applied torque = {:.3f} Nm").format(MaxTorque)

# Note the the data must have a Step of 0.5 on the attack angle to use the plot function
df = pd.read_csv(r'Lift coeff completo.csv')
df2 = pd.read_csv(r'NACA0012 curva Completa.csv')

df.dropna(inplace = True)


# Panda
J = 0.007
r = 80/2000

# Fin set
N = 3
root = 85 / 1000
tip = 33.5 / 1000
span = 76.5 / 1000
delta = 2 # Angulo de ataque para as aletas

# Canards set 
n = 2
Cr = 40 / 1000
Ct = 20 / 1000 #O tip é do tamanho do root para aproveitar que quanto mais longe do foguete, maior é o braço do momento
s = 40 / 1000
arm = 5/1000 # Braço entre a aleta e a fuselagem
alfa = 7 # angulo de ataque máximo para as canards

# Calculating Dynamic Pressure
speed = 0.6 # speed of the rocket in Mach number
rho = 1.06 # air density
DynamicPressure = (343 * speed)**2 * rho/2

# Desired Parameters
timeMax = 3
thetaDotMax = 12 #given in radians per second
thetaDotDotMax = (0 - thetaDotMax)/timeMax

Cana = Canards(n, Cr, Ct, s, r, arm)
Cana.setCN0(df2)
Fin = Canards(N, root, tip, span, r)
Fin.setCN0(df2)

Comp = Compare(Cana, Fin)

#Comp.plot_coeff_curves(mode='forcing', speed=0.7)
#Comp.calculateA(speed = speed, stall_angle= alfa, fin_delta_angle= delta, J=J, thetaDotDotMax = thetaDotDotMax, thetaDot0 =thetaDotMax, DynamicPressure=DynamicPressure, compare = True, speedMin = 0.1, speedMax = 0.4, tempo = 3)
Comp.torque(speed = 0.4, XCgFins = 21.425/1000, stall_angle = alfa, rho = 1.06)


    

