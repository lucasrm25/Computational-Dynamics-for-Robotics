import sys
sys.path.append('.')
from numpy import eye, array, ones, zeros, pi, arange, concatenate, append, diag, linspace, block, sum
from numpy.linalg import inv, norm, solve, pinv
from scipy.integrate import ode, odeint, solve_ivp
from scipy.spatial.transform import Rotation as R
from classes.RigidBody import RigidBody, Ground
from classes.MultiRigidBody import MultiRigidBody
from classes.RotationalJoint import RotationalJoint
from classes.SpringDamper import SpringDamper
from classes.PositionBilateralConstraint import PositionBilateralConstraint


#%% Setup MBD system

I_grav = array([[0,-9.81,0]]).T
ground = Ground()
link1 = RigidBody(m_B=4, B_I_B=diag([0.0042,0.8354,0.8354]), I_grav=I_grav)
link2 = RigidBody(m_B=4, B_I_B=diag([0.0042,0.8354,0.8354]), I_grav=I_grav)
link3 = RigidBody(m_B=4, B_I_B=diag([0.0042,0.8354,0.8354]), I_grav=I_grav)
link4 = RigidBody(m_B=4, B_I_B=diag([0.0042,0.8354,0.8354]), I_grav=I_grav)

joint1 = RotationalJoint(ground,link1, A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[0,0,0]]).T,   S_r_SDs= array([[-0.5,0,0]]).T)
joint2 = RotationalJoint(link1, link2, A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[0.5,0,0]]).T, S_r_SDs= array([[-0.5,0,0]]).T)
joint3 = RotationalJoint(link2, link3, A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[0.5,0,0]]).T, S_r_SDs= array([[-0.5,0,0]]).T)
joint4 = RotationalJoint(link1, link4, A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[0.5,0,0]]).T, S_r_SDs= array([[-0.5,0,0]]).T)

springDamper1 = SpringDamper(ground, link4, P_r_PDp=array([[1,1,0]]).T, S_r_SDs=array([[0.5,0,0]]).T, K=50, D=5, d0=0)

constraint = PositionBilateralConstraint(link3,ground, P_r_PDp=array([[0.5,0,0]]).T, S_r_SDs=array([[2,0,0]]).T)

# create multi-rigid-body object
pendulum = MultiRigidBody(ground=ground, springDampers=[springDamper1], bilateralConstraints=[constraint])

# set generalized coordinate indices
joint1.qIndex = 0
joint2.qIndex = 1
joint3.qIndex = 2
joint4.qIndex = 3
nq = 4
pendulum.setup(nq=nq)

# set initial conditions
pendulum.recursive_setall_q( q = array([60,-60,-60,-60-90]) * pi/180 )
pendulum.updateKinTree()


#%% Simulate

def odefun(t,y):
    q, qDot = y[0:nq], y[nq:]
    qDDot = pendulum.getODE ( q=q, qDot=qDot )
    return concatenate((qDot,qDDot))


# initial conditions
q0, dq0, ddq0  = pendulum.recursive_getall_q()

# simulate
tf = 20
fps = 60
odesol = solve_ivp( odefun, t_span=[0,tf], t_eval=arange(0,tf,1/fps), y0=concatenate((q0,dq0)).squeeze(), method='RK45', dense_output=True, events=None )
print(odesol.message)

from matplotlib import pyplot as plt
plt.figure()
plt.plot(odesol.t, odesol.y[0,:]*180/pi, label='joint1.q')
plt.plot(odesol.t, odesol.y[1,:]*180/pi, label='joint2.q')
plt.plot(odesol.t, odesol.y[2,:]*180/pi, label='joint3.q')
plt.legend()
plt.grid(True)
plt.show()


#%% Animate

pendulum.initGraphics(width=1200, height=800, range=1.5, title='A double pendulum', updaterate=fps)

while True:
    for t,y in zip(odesol.t,odesol.y.T):
        pendulum.recursive_setall_q( q=y[0:nq], qDot=y[nq:] )
        pendulum.updateKinTree()
        pendulum.updateGraphics()

print('Animation finished!')
