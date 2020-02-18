'''
    Bilateral Constraint making 2 points in different bodies coincide (one might be ground)

    Created by: Lucas Rath (lucasrm25@gmail.com)
'''

from numpy import append, array, size, ones, zeros, eye, ndarray, absolute
from numpy.linalg import matrix_power, norm
from .robotics_helpfuns import skew
from .RigidBody import RigidBody

from vpython import canvas, vector, color, rate, helix, arrow
from classes.vpython_ext import vellipsoid


class PositionBilateralConstraint():
    '''
    Abstract Generic Joint class
    '''

    def __init__(self, predBody:RigidBody, sucBody:RigidBody, 
                       A_PDp = eye(3), A_SDs = eye(3),
                       P_r_PDp = zeros([3,1]), S_r_SDs = zeros([3,1]),
                       wn=10, ksi=1):
        # link pred and suc bodies to current joint
        self.predBody = predBody
        self.sucBody  = sucBody
        
        # store static properties
        self.A_PDp   = A_PDp
        self.A_SDs   = A_SDs
        self.P_r_PDp = P_r_PDp
        self.S_r_SDs = S_r_SDs
        self.K       = wn**2       # Baumgarte proportional stabilization constant
        self.D       = 2*ksi*wn    # Baumgarte derivative stabilization constant


    def getConstraintTerms(self):
        '''
            Compute constraint terms
        '''

        # calculate displacement and velocity constraint violation
        I_c = self.predBody.I_r_IQ(B_r_BQ=self.P_r_PDp) - self.sucBody.I_r_IQ (B_r_BQ = self.S_r_SDs )
        I_cDot = self.predBody.I_v_Q(B_r_BQ=self.P_r_PDp) - self.sucBody.I_v_Q (B_r_BQ = self.S_r_SDs )
        
        # calculate jacobian J_lambda, such that: cDot = J_lambda * qDot, which is the ratio of the constraint
        # displacement to the generalized coordinates
        J_lambda = self.predBody.A_IB @ (self.predBody.B_J_S - skew(self.P_r_PDp) @ self.predBody.B_J_R ) -\
                   self.sucBody.A_IB  @ (self.sucBody.B_J_S  - skew(self.S_r_SDs) @ self.sucBody.B_J_R )

        sigma_lambda = self.predBody.I_a_Q( B_r_BQ=self.P_r_PDp ) - self.sucBody.I_a_Q( B_r_BQ=self.S_r_SDs )

        # add Baumgarte stabilization
        nc = sigma_lambda.size
        sigma_lambda += eye(nc)*self.D @ I_cDot + eye(nc)*self.K @ I_c

        return [J_lambda, sigma_lambda]


    ''' -------------------- GRAPHICS ------------------- '''


    def initGraphics(self, shaftwidth=0.01):
        self.arrow = arrow(pos=vector(0,0,0), axis=vector(1,0,0), shaftwidth=shaftwidth)
    
    def updateGraphics(self):
        origin = self.predBody.A_IB @ (self.predBody.B_r_IB + self.P_r_PDp )
        target = self.sucBody.A_IB @ (self.sucBody.B_r_IB + self.S_r_SDs )
        self.arrow.pos = vector( *(origin) )
        self.arrow.axis = vector( *(target-origin) )

