import sys
sys.path.append('.')
from numpy import eye, array, ones, zeros, pi, arange, concatenate, append, diag, linspace, block, sum
from numpy.linalg import inv, norm, solve, pinv
from scipy.integrate import ode, odeint, solve_ivp
from scipy.spatial.transform import Rotation as R
from classes.RigidBody import RigidBody, Ground, Rod, Ellipsoid
from classes.MultiRigidBody import MultiRigidBody
from classes.RotationalJoint import RotationalJoint
from classes.SpringDamper import SpringDamper
from classes.PositionBilateralConstraint import PositionBilateralConstraint
from classes.robotics_helpfuns import rotZ


#%% Setup MBD system

I_grav  = array([[0,-9.81,0]]).T
chassis  = Ground()
lowerA  = Rod(length=0.3, radius_o=0.01, radius_i=0, I_grav=I_grav)
upperA  = Rod(length=0.3, radius_o=0.01, radius_i=0, I_grav=I_grav)
pushbar = Rod(length=0.45, radius_o=0.01, radius_i=0, I_grav=I_grav)
upright = Ellipsoid(rx=0.2, ry=0.01,  rz=0.2,  density=2000, I_grav=I_grav)
rocker  = Ellipsoid(rx=0.05, ry=0.05, rz=0.02, density=8000, I_grav=I_grav)

j_chassis_lowerA = RotationalJoint(chassis,lowerA,  A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[0,0,0]]).T, S_r_SDs= array([[-lowerA.length/2,0,0]]).T)
j_lowerA_upright = RotationalJoint(lowerA,upright,  A_PDp=rotZ(pi/2), A_SDs=eye(3), P_r_PDp=array([[lowerA.length/2,0,0]]).T, S_r_SDs= array([[-0.1,0,0]]).T)
j_upright_upperA = RotationalJoint(upright, upperA, A_PDp=rotZ(pi/2), A_SDs=eye(3), P_r_PDp=array([[0.1,0,0]]).T, S_r_SDs= array([[-upperA.length/2,0,0]]).T)
c_upperA_chassis = PositionBilateralConstraint(upperA, chassis, P_r_PDp=array([[upperA.length/2,0,0]]).T, S_r_SDs=array([[0,0.2,0]]).T)

j_chassis_rocker = RotationalJoint(chassis, rocker, A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[-0.1,0.3,0]]).T, S_r_SDs= array([[0,-0.05,0]]).T)
j_rocker_pushbar = RotationalJoint(rocker, pushbar, A_PDp=eye(3), A_SDs=eye(3), P_r_PDp=array([[0.05,0,0]]).T, S_r_SDs= array([[-pushbar.length/2,0,0]]).T)
sd_chassis_rocker = SpringDamper(chassis, rocker, P_r_PDp=array([[-0.25,0.35,0]]).T, S_r_SDs=array([[-0.05,0,0]]).T, K=500, D=200, d0=0, radius=0.02, coils=30)

c_pushbar_lowerA = PositionBilateralConstraint(pushbar,lowerA, P_r_PDp=array([[pushbar.length/2,0,0]]).T, S_r_SDs=array([[1/2.5*lowerA.length,0,0]]).T)



# create multi-rigid-body object
# suspension = MultiRigidBody(ground=chassis, springDampers=[sd_chassis_rocker], bilateralConstraints=[c_pushbar_lowerA])
suspension = MultiRigidBody(ground=chassis, springDampers=[sd_chassis_rocker], bilateralConstraints=[c_upperA_chassis,c_pushbar_lowerA])


# set generalized coordinate indices
j_chassis_lowerA.qIndex = 0
j_lowerA_upright.qIndex = 1
j_upright_upperA.qIndex = 2
j_chassis_rocker.qIndex = 3
j_rocker_pushbar.qIndex = 4
nq = 5
suspension.setup(nq=nq)

# set initial conditions
suspension.recursive_setall_q( q = array([0, 0, 0, 0, -50]) * pi/180 )
suspension.updateKinTree()


#%% Simulate

def odefun(t,y):
    q, qDot = y[0:nq], y[nq:]
    qDDot = suspension.getODE ( q=q, qDot=qDot )
    return concatenate((qDot,qDDot))


# initial conditions
q0, dq0, ddq0  = suspension.recursive_getall_q()

# simulate
tf = 1
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

suspension.initGraphics(width=1200, height=800, range=1.5, title='Suspension', updaterate=fps/6)

while True:
    for t,y in zip(odesol.t,odesol.y.T):
        suspension.recursive_setall_q( q=y[0:nq], qDot=y[nq:] )
        suspension.updateKinTree()
        suspension.updateGraphics()

print('Animation finished!')
