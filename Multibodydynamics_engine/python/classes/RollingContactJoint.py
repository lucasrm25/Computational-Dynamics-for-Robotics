from numpy import array, ones, zeros, eye, cos, sin, asscalar
from .GenericJoint import GenericJoint
from .robotics_helpfuns import skew

class RotationalJoint(GenericJoint):

    def __init__(self, predBody, sucBody,
                       A_PDp = eye(3), A_SDs = eye(3),
                       P_r_PDp = zeros([3,1]), S_r_SDs = zeros([3,1]), r=0):

        super().__init__(predBody, sucBody, A_PDp, A_SDs, P_r_PDp, S_r_SDs)
        # rolling contact radius
        self.r = r
        # init generalized coordinates -> joint angle
        self.q = self.qDot = self.qDDot = array([0])
        self.dof = 1

    def JointFunction(self, q): # -> [Dp_r_DpDs, A_DpDs]
        gamma = asscalar(q)
        Dp_r_DpDs = array([[-self.r*gamma,0,0]]).T
        A_DpDs    = array([[cos(gamma), -sin(gamma), 0],
                           [sin(gamma), +cos(gamma), 0],
                           [0         , +0         , 1]])
        return [Dp_r_DpDs, A_DpDs]
        
    def JointVelocity(self, q, qDot): # -> [Dp_rDot_DpDs, Dp_omega_DpDs]
        # Overwrite generic JointVelocity:
        Dp_rDot_DpDs  = array([[-self.r*qDot,0,0]]).T
        Dp_omega_DpDs = array([[0,0,qDot]]).T
        return [Dp_rDot_DpDs, Dp_omega_DpDs]
    
    def JointAcceleration(self, q, qDot, qDDot): # -> [Dp_rDDot_DpDs, Dp_omegaDot_DpDs]
        # Overwrite generic JointAcceleration:
        Dp_rDDot_DpDs    = array([[-self.r*qDDot,0,0]]).T
        Dp_omegaDot_DpDs = array([[0,0,qDDot]]).T
        return [Dp_rDDot_DpDs, Dp_omegaDot_DpDs]
    
    def JointJacobian(self, q, qIndex, nq): # -> [S, R]
        # Overwrite generic JointJacobian:
        S = zeros([3,nq])
        R = zeros([3,nq])
        S[:,qIndex] = [-self.r,0,0]
        R[:,qIndex] = [0,0,1]
        return [S, R]