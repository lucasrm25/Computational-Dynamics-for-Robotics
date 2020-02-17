from numpy import array, ones, zeros, eye, size
from numpy.linalg import inv
from scipy.linalg import expm
from .robotics_helpfuns import skew
from .RigidBody import RigidBody, Ground
from .SpringDamper import SpringDamper
from typing import List, Dict, Tuple

from vpython import canvas, vector, color, rate

class MultiRigidBody():
    
    def __init__( self, ground:Ground , springDampers:List=[], bilateralConstraints:List=[]):
        self._nq = 0 # dimensions of the q-vector
        self.ground = ground
        self.springDampers = springDampers
        self.bilateralConstraints = bilateralConstraints
    
    def setup(self, nq:int):
        self._nq = nq
        # TODO: set joint indices
        #       for now, indices are set manually

    @property
    def nq(self):
        assert self._nq > 0, 'Error: Please call setup() from Ground'
        return self._nq
    
    def updateKinTree( self ):
        self.ground._recursiveForwardKinematics(nq=self.nq)

    def getODE( self, q, qDot ): # -> [M, f, g, tau]
        # set q and qDot to correspondent joints
        self.recursive_setall_q( q=q, qDot=qDot )

        # Set all joint accelerations to zero, so the subsequent call to _recursiveForwardKinematics 
        # will produce bias accelerations, not real accelerations
        for childJoint in self.ground.childJoints:
            childJoint._recursive_reset_qDDot()
        self.updateKinTree()
        [M, f, g] = self.ground._recursiveComputationOfMfg()
        
        # calculate generalized forces tau, due to springs and dampers
        tau = 0
        for springdamper in self.springDampers:
            tau += springdamper.computationOfTau()

        return M, f, g, tau


    def recursive_setall_q(self, q=[], qDot=[], qDDot=[]):
        for childJoint in self.ground.childJoints:
            childJoint._recursive_setall_q(q, qDot, qDDot)


    def recursive_getall_q(self):
        q    = zeros(self.nq)
        qDot = zeros(self.nq)
        qDDot = zeros(self.nq)
        for childJoint in self.ground.childJoints:
            q, qDot, qDDot = childJoint._recursive_getall_q(q, qDot, qDDot)
        return [q, qDot, qDDot]


    def initGraphics(self, width:int=1200, height:int=800, range:int=1.5, background=color.white, title:str='Vpython animation', updaterate:int=60):
        canvas(width=width, height=height, range=range, background=background, title=title)
        self.ground._recursiveInitGraphicsVPython()
        for sd in self.springDampers:
            sd.initGraphics()
        self.updaterate = updaterate
    
    def updateGraphics(self):
        self.ground._recursiveUpdateGraphicsVPython()
        for sd in self.springDampers:
            sd.updateGraphics()
        rate(self.updaterate)