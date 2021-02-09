import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from Function import *


data = np.array(pd.read_csv(r'NACA0012 curva Completa.csv'))
data2 = np.array(pd.read_csv(r'NACA0012 curva Completa Re 360000.csv'))

# alfa = [data[i][0] * 180 / np.pi for i in range(len(data))]
# cnalfa0 = [data[i][1] for i in range(len(data))]
temp = [[data[i][0] * 180 / np.pi, data[i][1]] for i in range(len(data))]

Airfoil = Function(temp, 'Alfa (Ângulo de ataque em graus)', 'Cl (Coeficiente de Sustentação)', interpolation='spline', extrapolation = 'natural')
#Airfoil2 = Function(data2, 'Alfa (Ângulo de ataque em graus)', 'Cl (Coeficiente de Sustentação)', interpolation='spline', extrapolation = 'natural')
#Airfoil.plot(lower = 0, upper = 180)
#Airfoil.plot(lower = 0, upper = 9)

#Function.comparePlots([(Airfoil, 'Re = 160000'), (Airfoil2, 'Re = 360000')], lower = 0, upper = 180, title= "Curva interpolada do coeficiente de sustentação do aerofólio NACA0012", xlabel='Alfa (Ângulo de ataque em graus)', ylabel='Cl (Coeficiente de Sustentação)')

df = pd.read_csv(r'Lift coeff completo.csv')
df.dropna(inplace = True)


data3 = np.array(df["M=" + str(0.1) + str(0)])
alfa3 = [[i/2, data3[i]] for i in range(len(data3))]
print(alfa3)
Airfoil3 = Function(alfa3, 'Alfa (Ângulo de ataque em graus)', 'Cl (Coeficiente de Sustentação)', interpolation='spline', extrapolation = 'natural')

Function.comparePlots([(Airfoil, 'Literatura'), (Airfoil3, 'Software XFLR5')], lower = 0, upper = 14, title= "Comparativo entre dados da literatura e simulados", xlabel='Alfa (Ângulo de ataque em graus)', ylabel='Cl (Coeficiente de Sustentação)')
