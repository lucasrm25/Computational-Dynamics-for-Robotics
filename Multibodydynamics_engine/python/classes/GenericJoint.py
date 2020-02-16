'''
    Abstract class that defines a generic Joint

    Defines a generic joint (without any specific motion) in a kinematic tree that is implemented as a set of linked selfects.  
    This class allows the computation of Positions/Orientations, Velocities, Accelerations, and Jacobians.

    Created by: Lucas Rath (lucasrm25@gmail.com)
'''

from numpy import append, array, size, ones, zeros, eye
from numpy.linalg import matrix_power
from abc import ABC, abstractmethod
from .robotics_helpfuns import skew
from .RigidBody import RigidBody


class GenericJoint(ABC):
    '''
    Abstract Generic Joint class
    '''

    def __init__(self, predBody:RigidBody, sucBody:RigidBody, 
                       A_PDp = eye(3), A_SDs = eye(3),
                       P_r_PDp = zeros([3,1]), S_r_SDs = zeros([3,1])):
        # link pred and suc bodies to current joint
        self.predBody = predBody
        self.sucBody  = sucBody
        sucBody.parentJoint = self
        predBody.childJoints = append(predBody.childJoints, self)
        
        # init abstract properties
        self.A_PDp   = A_PDp
        self.A_SDs   = A_SDs
        self.P_r_PDp = P_r_PDp
        self.S_r_SDs = S_r_SDs
        self.dof = 0
        self.q = array([])
        self.qDot = array([])
        self.qDDot = array([])

        # generalized coordinates indices
        self.qIndex = []

    def _recursive_reset_qDDot(self):
        self.qDDot[:] = 0
        for child in self.sucBody.childJoints:
            child._recursive_reset_qDDot()

    @abstractmethod
    def JointJacobian(self, q, qIndex, nq): # -> [S, R]
        pass

    @abstractmethod
    def JointFunction(self, q): # -> [Dp_r_DpDs, A_DpDs]
        pass

    @abstractmethod
    def JointVelocity(self, q, qDot): # -> [Dp_rDot_DpDs, Dp_omega_DpDs]
        pass

    @abstractmethod
    def JointAcceleration(self, q, qDot, qDDot): # -> [Dp_rDDot_DpDs, Dp_omegaDot_DpDs]
        pass

    def _recursiveForwardKinematics(self, P_r_IP, A_IP, P_omega_P, P_v_P, P_omega_dot_P, P_a_P, P_J_S, P_J_R):
        '''
            Given predecessor (P) body kinematics and dynamics:
                - P_r_IP, A_IP, P_omega_P, P_v_P, P_omega_dot_P, P_a_P, P_J_S, P_J_R
            Calculate kinematics and dynamics for the succesor body:
                - S_r_IS, A_IS, S_omega_S, S_v_S, S_omegaDot_S, S_a_S, S_J_S, S_J_R
            based on current joint configuration
        '''

        # Rotation and displacement about the joint:
        Dp_r_DpDs, A_DpDs = self.JointFunction(self.q)
        # Angular rate and translational velocity accross the joint:
        Dp_rDot_DpDs, Dp_omega_DpDs = self.JointVelocity(self.q, self.qDot)
        # Angular andtranslational acceleration accross the joint:
        Dp_rDDot_DpDs, Dp_omegaDot_DpDs = self.JointAcceleration(self.q, self.qDot, self.qDDot)
        # Compute the matrices that define the velocity accross the joint as a linear function of q_dot:
        nq = size(P_J_S,1)
        S, R = self.JointJacobian( self.q, self.qIndex, nq)

        # Compute the position, velocity, and acceleration of each successing coordinate system:
        A_IDp           = A_IP @ self.A_PDp
        Dp_r_IDp        = self.A_PDp.T @ (P_r_IP + self.P_r_PDp)
        Dp_omega_Dp     = self.A_PDp.T @ (P_omega_P + 0)
        Dp_v_Dp         = self.A_PDp.T @ (P_v_P + 0 + skew(P_omega_P) @ self.P_r_PDp)
        Dp_omegaDot_Dp  = self.A_PDp.T @ (P_omega_dot_P + 0 + 0)
        Dp_a_Dp         = self.A_PDp.T @ (P_a_P + 0 + 0 + (skew(P_omega_dot_P) + matrix_power(skew(P_omega_P),2) ) @ self.P_r_PDp)

        A_IDs           = A_IDp @ A_DpDs
        Ds_r_IDs        = A_DpDs.T @ (Dp_r_IDp + Dp_r_DpDs)

        Ds_omega_Ds     = A_DpDs.T @ (Dp_omega_Dp + Dp_omega_DpDs)
        Ds_v_Ds         = A_DpDs.T @ (Dp_v_Dp + Dp_rDot_DpDs + skew(Dp_omega_Dp) @ Dp_r_DpDs)
        Ds_omegaDot_Ds  = A_DpDs.T @ (Dp_omegaDot_Dp + Dp_omegaDot_DpDs + skew(Dp_omega_Dp) @ Dp_omega_DpDs)
        Ds_a_Ds         = A_DpDs.T @ (Dp_a_Dp + Dp_rDDot_DpDs + 2 * skew(Dp_omega_Dp) @ Dp_rDot_DpDs + (skew(Dp_omegaDot_Dp) + matrix_power(skew(Dp_omega_Dp),2) ) @ Dp_r_DpDs)

        A_IS            = A_IDs @ self.A_SDs.T
        S_r_IS          = self.A_SDs @ Ds_r_IDs - self.S_r_SDs
        S_omega_S       = self.A_SDs @ (Ds_omega_Ds + 0)
        S_v_S           = self.A_SDs @ (Ds_v_Ds + 0) - skew(S_omega_S) @ self.S_r_SDs
        S_omegaDot_S    = self.A_SDs @ (Ds_omegaDot_Ds + 0 + 0)
        S_a_S           = self.A_SDs @ (Ds_a_Ds + 0 + 0) - (skew(S_omegaDot_S) + matrix_power(skew(S_omega_S),2) ) @ self.S_r_SDs

        # Compute the displacement and orientation of the successor body:
        # Compute the overall rotation first:
        A_PS = self.A_PDp @ A_DpDs @ self.A_SDs.T
        # Compute the rotational Jacobian of the successor body:
        S_J_R = A_PS.T @ (P_J_R + self.A_PDp @ R)
        # Compute the translational Jacobian of the successor body:
        S_J_S = A_PS.T @ (P_J_S + self.A_PDp @ S + skew(self.P_r_PDp + self.A_PDp @ Dp_r_DpDs).T @ P_J_R) - skew(self.S_r_SDs).T @ S_J_R

        # Pass this information on to the successor body:
        self.sucBody._recursiveForwardKinematics(S_r_IS, A_IS, S_omega_S, S_v_S, S_omegaDot_S, S_a_S, S_J_S, S_J_R)

    
    def _recursive_setall_q(self, q, qDot, qDDot):
        if size(q) > 0:
            self.q = q[self.qIndex]
        if size(qDot) > 0:  
            self.qDot = qDot[self.qIndex]
        if size(qDDot) > 0:
            self.qDDot = qDDot[self.qIndex]

        for childJoint in self.sucBody.childJoints:
            childJoint._recursive_setall_q(q, qDot, qDDot)

    def _recursive_getall_q(self, q, qDot, qDDot):
        q[self.qIndex]     = self.q
        qDot[self.qIndex]  = self.qDot
        qDDot[self.qIndex] = self.qDDot

        for childJoint in self.sucBody.childJoints:
            childJoint._recursive_getall_q(q, qDot, qDDot)
        return [q, qDot, qDDot] 