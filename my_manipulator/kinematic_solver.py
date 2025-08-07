from sympy import *
# import numpy as np
import math

init_printing()

θ1=symbols('θ1')
θ2=symbols('θ2')
θ3=symbols('θ3')


# dh parameter measured form the physical system
dh_table=[
    {},
    {'alpha': 0,     'a': 0,    'd': 9.2, 'θ':θ1},
    {'alpha': -pi/2, 'a': 1.1,  'd': 0,   'θ':θ2},
    {'alpha': pi,    'a': 10.4, 'd': 0,   'θ':θ3},
    {'alpha': 0,     'a': 13.4, 'd': 0,   'θ':0},
]

# defining general transformation matrix for each frame 
def matrix(i) :
    
    alpha=dh_table[i]['alpha']
    a=dh_table[i]['a']
    d=dh_table[i]['d']
    θ=dh_table[i]['θ']

    # from john j craig, intro to robotics, 3rd edition, chapter 3, section 3.5
    # t(i-1)i(transformation matrix of frame i relative to frame i-1)=Rx[alpha(i-1]*Dx[a(i-1)]*Rz(θi)*Dz(di)

    transformation_matrix=Matrix([[ cos(θ)            ,      -sin(θ)      ,      0      ,         a    ],
                                  [ sin(θ)*cos(alpha) , cos(θ)*cos(alpha) , -sin(alpha) , -sin(alpha)*d],
                                  [ sin(θ)*sin(alpha) , cos(θ)*sin(alpha) ,  cos(alpha) ,  cos(alpha)*d],
                                  [       0           ,       0           ,       0     ,         1    ]
                                 ])


    return transformation_matrix 

# tmn= transformation matrix of n relative to m

t01=matrix(1)
t12=matrix(2)
t23=matrix(3)
t34=matrix(4)


t04=t01*t12*t23*t34


tfik=Matrix([[1,0,0,13],
             [0,1,0,15],
             [0,0,1,8],
             [0,0,0,1]
             ])

eq1=Eq(t04[0,3],tfik[0,3])
eq2=Eq(t04[1,3],tfik[0,3])
eq3=Eq(t04[2,3],tfik[2,3])

solution=solve(eq1,eq2,eq3)
print(eq1)


# x=dh_table[1]['d']
# y=x*3
# print(y)



# x=dh_table[3]['alpha2']
# print(sec(x))
