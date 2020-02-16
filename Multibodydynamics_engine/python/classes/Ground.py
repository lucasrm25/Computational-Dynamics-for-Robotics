from numpy import array, ones, zeros, eye, size
from numpy.linalg import inv
from scipy.linalg import expm
from .robotics_helpfuns import skew
from .RigidBody import RigidBody

class Ground():
    
    def __init__( self ):
        # link body to joints
        self.childJoints = array([])
        self._nq = 0 # number of generalized coordinates in the whole multi-body system
    
    def setup(self, nq):
        self._nq = nq
        # TODO: set joint indices
        #       for now, indices are set manually

    @property
    def nq(self):
        assert self._nq > 0, 'Error: Please call setup() from Ground'
        return self._nq

    @property
    def nChildren(self):
        return self.childJoints.size
    
    def updateKinTree( self ):
        # ground, by definition, is at the origin and is static
        A_IB                     = eye(3)
        B_omega_B = B_omegaDot_B = zeros([3,1])
        B_r_IB = B_v_B = B_a_B   = zeros([3,1])
        B_J_S = B_J_R            = zeros([3,self.nq])

        for child in self.childJoints:
            child._recursiveForwardKinematics(B_r_IB, A_IB, B_omega_B, B_v_B, B_omegaDot_B, B_a_B,  B_J_S, B_J_R)

    def getODE( self ): # -> [M, f, g]
        M = zeros([self.nq,self.nq]) 
        f = zeros([self.nq,1])
        g = zeros([self.nq,1])

        # Set all joint accelerations to zero, so the subsequent call to _recursiveForwardKinematics 
        # will produce bias accelerations, not real accelerations
        for childJoint in self.childJoints:
            childJoint._recursive_reset_qDDot()
        self.updateKinTree()

        for child in self.childJoints:
            M_part, f_part, g_part = child.sucBody._recursiveComputationOfMfg()
            M += M_part
            f += f_part
            g += g_part
        return M, f, g

    def recursive_setall_q(self, q=[], qDot=[], qDDot=[]):
        for childJoint in self.childJoints:
            childJoint._recursive_setall_q(q, qDot, qDDot)

    def recursive_getall_q(self):
        q    = zeros(self.nq)
        qDot = zeros(self.nq)
        qDDot = zeros(self.nq)
        for childJoint in self.childJoints:
            q, qDot, qDDot = childJoint._recursive_getall_q(q, qDot, qDDot)
        return [q, qDot, qDDot]