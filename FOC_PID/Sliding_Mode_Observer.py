from math import sin, cos, pi , tan , atan
from control import *
from control.matlab import * # ss,  tf
#, step, control.feedback, obsv, ctrb,
#acker(pole placement),lqr,place,ss2tf(sys)
#
import pylab    # pylab.array
from numpy.linalg import matrix_rank


# matrices
R = 1.6
#Ra = 1.6
La = 400e-3
#Rb = Ra
Lb = 420e-3
#Rc = Rb
Lc = 440e-3

Lav = (La + Lb + Lc)/3

A = [[-R/Lav,0],[0,-R/Lav]]
B = [[1/Lav,0],[0,1/Lav]]
C = [[1,0],[0,1]]
D = [[0,0],[0,0]]

sys = ss(A,B,C,D)

O = obsv(A,C)   # observability matrix   full rank












