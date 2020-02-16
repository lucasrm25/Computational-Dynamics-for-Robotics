from numpy import array, ones, zeros, eye, cos, sin, asscalar
from .GenericJoint import GenericJoint
from .robotics_helpfuns import skew

class RigidJoint(GenericJoint):

    def __init__(self, predBody, sucBody,
                       A_PDp = eye(3), A_SDs = eye(3),
                       P_r_PDp = zeros([3,1]), S_r_SDs = zeros([3,1])):

        super().__init__(predBody, sucBody, A_PDp, A_SDs, P_r_PDp, S_r_SDs)
        # init generalized coordinates -> joint angle
        self.q = self.qDot = self.qDDot = array([0])
        self.dof = 0

    def JointFunction(self, q): # -> [Dp_r_DpDs, A_DpDs]
        Dp_r_DpDs = zeros([3,1])
        A_DpDs    = eye(3)
        return [Dp_r_DpDs, A_DpDs]
        
    def JointVelocity(self, q, qDot): # -> [Dp_rDot_DpDs, Dp_omega_DpDs]
        # Overwrite generic JointVelocity:
        Dp_rDot_DpDs  = zeros([3,1])
        Dp_omega_DpDs = zeros([3,1])
        return [Dp_rDot_DpDs, Dp_omega_DpDs]
    
    def JointAcceleration(self, q, qDot, qDDot): # -> [Dp_rDDot_DpDs, Dp_omegaDot_DpDs]
        # Overwrite generic JointAcceleration:
        Dp_rDDot_DpDs    = zeros([3,1])
        Dp_omegaDot_DpDs = zeros([3,1])
        return [Dp_rDDot_DpDs, Dp_omegaDot_DpDs]
    
    def JointJacobian(self, q, qIndex, nq): # -> [S, R]
        # Overwrite generic JointJacobian:
        S = zeros([3,nq])
        R = zeros([3,nq])
        return [S, R]