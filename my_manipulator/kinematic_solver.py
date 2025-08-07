from sympy import *
import numpy as np
import math

init_printing()

theta1=symbols('theta1')
theta2=symbols('theta2')
theta3=symbols('theta3')


# dh parameter measured form the physical system
dh_table=[
    {},
    {'alpha': 0,     'a': 0,    'd': 9.2, 'theta':theta1},
    {'alpha': -pi/2, 'a': 1.1,  'd': 0,   'theta':theta2},
    {'alpha': pi,    'a': 10.4, 'd': 0,   'theta':theta3},
    {'alpha': 0,     'a': 13.4, 'd': 0,   'theta':0},
]

# defining general transformation matrix for each frame 
def matrix(i) :
    
    alpha=dh_table[i]['alpha']
    a=dh_table[i]['a']
    d=dh_table[i]['d']
    theta=dh_table[i]['theta']

    # from john j craig, intro to robotics, 3rd edition, chapter 3, section 3.5
    # t(i-1)i(transformation matrix of frame i relative to frame i-1)=Rx[alpha(i-1]*Dx[a(i-1)]*Rz(thetai)*Dz(di)

    transformation_matrix=Matrix([[ cos(theta)            ,      -sin(theta)      ,      0      ,         a    ],
                                  [ sin(theta)*cos(alpha) , cos(theta)*cos(alpha) , -sin(alpha) , -sin(alpha)*d],
                                  [ sin(theta)*sin(alpha) , cos(theta)*sin(alpha) ,  cos(alpha) ,  cos(alpha)*d],
                                  [           0           ,           0           ,       0     ,         1    ]
                                 ])


    return transformation_matrix 

# tmn= transformation matrix of n relative to m

t01=matrix(1)
t12=matrix(2)
t23=matrix(3)
t34=matrix(4)


t04=t01*t12*t23*t34

print(t04)
# x=dh_table[1]['d']
# y=x*3
# print(y)



# x=dh_table[3]['alpha2']
# print(sec(x))
