import sys
sys.path.append('.')
from numpy import eye, array, ones, zeros, pi, arange, concatenate, append, diag, linalg, linspace
# from numpy.linalg import inv, norm
from scipy.integrate import ode, odeint, solve_ivp
from scipy.spatial.transform import Rotation as R
from classes.RigidBody import RigidBody
from classes.Ground import Ground
from classes.RotationalJoint import RotationalJoint


#%% Setup MBD system

I_grav = array([[0,-9.81,0]]).T
ground = Ground()
link1 = RigidBody(m_B=4, B_I_B=diag([0.0042,0.8354,0.8354]), I_grav=I_grav)
link2 = RigidBody(m_B=4, B_I_B=diag([0.0042,0.8354,0.8354]), I_grav=I_grav)
link3 = RigidBody(m_B=4, B_I_B=diag([0.0042,0.8354,0.8354]), I_grav=I_grav)
joint1 = RotationalJoint(ground,link1, A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[0,0,0]]).T,   S_r_SDs= array([[-0.5,0,0]]).T)
joint2 = RotationalJoint(link1, link2, A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[0.5,0,0]]).T, S_r_SDs= array([[-0.5,0,0]]).T)
joint3 = RotationalJoint(link1, link3, A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[0.5,0,0]]).T, S_r_SDs= array([[-0.5,0,0]]).T)


# set generalized coordinate indices
joint1.qIndex = 0
joint2.qIndex = 1
joint3.qIndex = 2
nq=3
ground.setup(nq=nq)

# set initial conditions
ground.recursive_setall_q( q = array([30,10,-20]) * pi/180 )


#%% Simulate

def odefun(t,y):
    q, qDot = y[0:nq], y[nq:]

    # set generalized coordinates to correspondent joints
    ground.recursive_setall_q( q=q, qDot=qDot )

    # calculate mass matrix M, coriollis and cetripedal forces f and gravitational force g
    M,f,g = ground.getODE()

    # calculate controller input
    tau = 0

    # return acceleration
    qDDot = ( linalg.inv(M) @ (f + g + tau) ).squeeze()
    return concatenate((qDot,qDDot))


# initial conditions
q0, dq0, ddq0  = ground.recursive_getall_q()

# simulate
odesol = solve_ivp( odefun, t_span=[0,20], t_eval=arange(0,10,1/60), y0=concatenate((q0,dq0)).squeeze(), method='RK45', dense_output=True, events=None )

from matplotlib import pyplot as plt
plt.figure()
plt.plot(odesol.t, odesol.y[0,:]*180/pi, label='joint1.q')
plt.plot(odesol.t, odesol.y[1,:]*180/pi, label='joint2.q')
plt.plot(odesol.t, odesol.y[2,:]*180/pi, label='joint3.q')
plt.legend()
plt.grid(True)
plt.show()


#%% animate

from vpython import *

class vellipsoid:
    import numpy as np

    def __init__(self, pos, color, size):
        self.ell = ellipsoid(pos=pos, color=color, size=size)
        self.A_IB = eye(3)

    @property
    def pos(self):
        return self.ell.pos 

    @pos.setter
    def pos(self, newpos:np.ndarray):
        self.ell.pos = vector( *(newpos) )
    
    @property
    def orientation(self):
        return self.A_IB

    @orientation.setter
    def orientation(self, A_IB:np.ndarray):
        '''
        rotate vpython object using rotation matrix instead of vector-angle
        '''
        rot = R.from_matrix( A_IB @ self.A_IB.T ).as_rotvec()
        self.ell.rotate( angle=linalg.norm(rot), axis=vector(*rot) )
        self.A_IB = A_IB


canvas(width=1200, height=800, range=1, background=color.white, title='A double pendulum')

obj_link1 = vellipsoid(pos=vector(0,0,0), color=color.orange, size=vector(*link1.ellsize))
obj_link2 = vellipsoid(pos=vector(0,0,0), color=color.orange, size=vector(*link2.ellsize))
obj_link3 = vellipsoid(pos=vector(0,0,0), color=color.orange, size=vector(*link3.ellsize))

for t,y in zip(odesol.t,odesol.y.T):

    ground.recursive_setall_q( q=y[0:nq], qDot=y[nq:] )
    ground.updateKinTree()

    # update positions
    obj_link1.pos = link1.A_IB @ link1.B_r_IB
    obj_link2.pos = link2.A_IB @ link2.B_r_IB
    obj_link3.pos = link3.A_IB @ link3.B_r_IB

    # update rotations
    obj_link1.orientation = A_IB=link1.A_IB @ link1.A_BP
    obj_link2.orientation = A_IB=link2.A_IB @ link2.A_BP
    obj_link3.orientation = A_IB=link3.A_IB @ link3.A_BP

    rate(60)

print('Animation finished!')