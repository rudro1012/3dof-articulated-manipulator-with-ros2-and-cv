from sympy import *
import numpy as np
import math

theta1=symbols('theta1')
theta2=symbols('theta2')
theta3=symbols('theta3')
theta4=symbols('theta4')

dh_table=[
    {},
    {'alpha0': 0,     'a0': 0,    'd1': 9.2, 'theta1':theta1},
    {'alpha1': -pi/2, 'a1': 1.1,  'd2': 0,   'theta2':theta2},
    {'alpha2': pi,    'a2': 10.4, 'd3': 0,   'theta3':theta1},
    {'alpha3': 0,     'a3': 13.4, 'd4': 0,   'theta4':theta1},
]

x=dh_table[3]['alpha2']
print(sec(x))
