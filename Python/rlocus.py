import numpy as np
from numpy import genfromtxt
from rocketpy import Function
from data import *
from control.matlab import *
plt.style.use('seaborn')

class Canard:
    def __init__(self, n, span, rootChord, tipChord, radius, airfoil):
        self.n = n
        self.span = span
        self.rootChord = rootChord
        self.tipChord = tipChord
        self.radius = radius
        self.airfoil = airfoil
        
        self.area = np.pi * radius**2
        self.addFins()

    def addFins(self):
        # Retrieve parameters for calculations
        Af = (self.rootChord + self.tipChord) * self.span / 2  # fin area
        AR = 2 * (self.span ** 2) / Af  # Aspeself.tipChord ratio
        gamac = np.arctan((self.rootChord - self.tipChord) / (2 * self.span))  # mid chord angle

        # Import the lift curve as a funself.tipChordion of lift values by attack angle
        read = genfromtxt(self.airfoil, delimiter=",")
        cnalfa0 = Function(read, extrapolation="natural").differentiate(0, 1e-01)

        # Calculate clalpha
        FD = 2 * np.pi * AR / (cnalfa0 * np.cos(gamac))
        clalpha = (
            cnalfa0
            * FD
            * (Af / self.area)
            * np.cos(gamac)
            / (2 + FD * (1 + (4 / FD ** 2)) ** 0.5)
        )

        # Aplies number of fins to lift coefficient
        clalpha *= self.n / 2

        # Fin–body interference correself.tipChordion
        clalpha *= 1 + self.radius / (self.span + self.radius)

        # self.rootChordeate a funself.tipChordion of lift values by attack angle
        cldata = Function(
            lambda x: clalpha * x, "Alpha (rad)", "Lift coeficient (Cl)", interpolation="linear"
        )

        # Save cldata
        self.cldata = cldata

        # Calculate roll forcing properties
        Ymac = self.radius + self.span / 3 * (self.rootChord + 2 * self.tipChord) / (self.rootChord + self.tipChord)
        clf_delta = clalpha * Ymac / (2 * self.radius)

        clfdata = Function(
            lambda delta: clf_delta * delta, "Delta (rad)", "Roll forcing coeficient (Clf)", interpolation="linear"
        )

        # Save clfdata
        self.clfdata = clfdata

        # Calculate roll damping properties
        b1 = (Ymac /  2) * (self.radius**2) * self.span
        b2 = ((self.rootChord + 2 * self.tipChord) / 3) * self.radius * (self.span**2)
        b3 = ((self.rootChord + 3 * self.tipChord) / 12) * (self.span**3)
        trapezoidal_constant_fins = b1 + b2 + b3

        cld_wv = self.n * cnalfa0 * trapezoidal_constant_fins / (self.area * 2 * self.radius)
        self.cld_data = lambda w, v: cld_wv * w / v
        
'''Dados do Mini'''
# Dados das aletas
n = 3
span = 0.077
rootChord = 0.058
tipChord = 0.018
airfoil = 'NACA0012 curva Completa.txt'
angulo_maximo_de_abertura = 3 * np.pi / 180

# Dados das canards
n_canard = 2
span_canard = 0.06
rootChord_canard = 0.035
tipChord_canard = 0.035
airfoil_canard = 'NACA0012 curva Completa.txt'
angulo_maximo_de_abertura_canard = 7 * np.pi / 180

# Dados do foguete
radius = 80.9/2000
J = 0.007
Ar = np.pi * radius**2 # Área de referencia
Lr = 2 * radius # Comprimento de referencia

# Dados do ambiente ao redor
rho = 1.06 # air density
velocidade = 0.3 * 343 # Mach 0.5
DynamicPressure = velocidade**2 * rho / 2

# Criando objeto correspondente à aleta
aleta = Canard(n, span, rootChord, tipChord, radius, airfoil)
canard = Canard(n_canard, span_canard, rootChord_canard, tipChord_canard, radius, airfoil)

# Parametros das aletas
forcing_aletas_coef = aleta.clfdata.differentiate(0)
damping_aletas_coef = aleta.cld_data(1, velocidade) - aleta.cld_data(0, velocidade)

# Parametros das canards
forcing_canard_coef = canard.clfdata.differentiate(0)
damping_canard_coef = canard.cld_data(1, velocidade) - canard.cld_data(0, velocidade)

# Parametros intermediarios
P = DynamicPressure * Ar * Lr
C = (damping_aletas_coef + damping_canard_coef)

s = tf([1, 0], 1)

# Planta
Gp = 1 / (J * s + P * C)

# Servo
tau_s = 0.07 / (np.pi/3)
Gs = 1 / (tau_s * s + 1)

# Controlador
Kp = 0.015
I = 7e-3 * 100 * 3
D = 0

print('Kp =', Kp)
print('I =', I)
print('D =', D)

# =========================================================================== Proporcional
Gcp = (1 + I / s + D * s) # Root Locus do ganho proporcional

# Sensoreamento
H = 1

# Função de transferência em malha aberta
Grp = Gcp * Gs * Gp * forcing_canard_coef

r, k = rlocus(Grp)
plt.show()

# =========================================================================== Integrativo
# Gci = (Kp + I / s + D * s)

# # Função de transferência em malha aberta
# Gri = Gci * Gs * Gp * forcing_canard_coef

# r, k = rlocus(Gri, xlim=(-20, 2), ylim=(-10, 10))
# plt.show()

print(Grp)