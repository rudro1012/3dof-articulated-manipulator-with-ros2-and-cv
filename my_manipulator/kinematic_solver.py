from sympy import *
import numpy as np
import math

init_printing()

theta1=symbols('theta1')
theta2=symbols('theta2')
theta3=symbols('theta3')


dh_table=[
    {},
    {'alpha': 0,     'a': 0,    'd': 9.2, 'theta':theta1},
    {'alpha': -pi/2, 'a': 1.1,  'd': 0,   'theta':theta2},
    {'alpha': pi,    'a': 10.4, 'd': 0,   'theta':theta3},
    {'alpha': 0,     'a': 13.4, 'd': 0,   'theta':0},
]


def matrix(i) :
    transformation_matrix=1
    return transformation_matrix 


# x=dh_table[3]['alpha2']
# print(sec(x))
