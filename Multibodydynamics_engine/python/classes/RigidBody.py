from numpy import array, ones, zeros, eye, size, sqrt
from numpy.linalg import inv, eig, matrix_power
from scipy.linalg import expm
from .robotics_helpfuns import skew

class RigidBody():
    
    def __init__( self, m_B=1, B_I_B=eye(3), I_grav=array([[0,0,9.81]]).T ):
        # link body to joints
        self.childJoints = array([])
        self.parentJoint = array([])
        
        # body properties
        self.m_B    = array(m_B)
        self.B_I_B  = array(B_I_B)

        # Inertia ellipse and principal axes
        self.ellsize, self.A_BP = self.getInertiaEllipsoid()

        # gravity vector
        self.I_grav = array(I_grav)
        
        # body state
        self.A_IB          = eye(3)          # The rotational orientation of the body B with respect to the inertial frame
        self.B_omega_B     = zeros([3,1])    # The absolute angular velocity [rad/s]
        self.B_omegaDot_B  = zeros([3,1])    # The absolute angular acceleration  [rad/s^2]
        self.B_r_IB        = zeros([3,1])    # The displacement of the body's COG [m]
        self.B_v_B         = zeros([3,1])    # The absolute velocity of the body's COG [m/s]
        self.B_a_B         = zeros([3,1])    # The absolute acceleration of the body's COG [m/s^2]

    @property
    def nChildren(self):
        return self.childJoints.size
    
    @property
    def isLeaf(self):
        return self.childJoints.size == 0

    def getInertiaEllipsoid(self): # -> []
        '''
            returns:
                - A_BP: rotation matrix from principal axes to body coordinates
                - ellsize: ellipse size corresponding to the Inertia in principal axes (P)
        '''
        # Compute the inertia axis:
        D, V = eig(self.B_I_B)

        A_BP = V

        I1, I2, I3 = D
        # Define the main axis of the ellipsoid:
        a = sqrt(2.5/self.m_B*(- I1 + I2 + I3))
        b = sqrt(2.5/self.m_B*(+ I1 - I2 + I3))
        c = sqrt(2.5/self.m_B*(+ I1 + I2 - I3))
        ellsize = array([a,b,c])

        return [ellsize, A_BP]


    def integrationStep( self, delta_t=0.001 ):
        # Using the M = skew(w) function which is defined below, we compute the 
        # skew symmetric matrices of omega_B in I and in B-coordinates: 
        B_omega_IB = skew(self.B_omega_B)
        I_omega_IB = skew(self.A_IB @ self.B_omega_B)
        
        # Doing one-step Euler forward integration for linear motion
        # while taking into account that we do so in a moving coordinate system:  
        self.B_r_IB = self.B_r_IB + delta_t * (self.B_v_B - B_omega_IB @ self.B_r_IB)
        self.B_v_B  = self.B_v_B  + delta_t * (self.B_a_B - B_omega_IB @ self.B_v_B)
        # Using the matrix-exponential to compute A_IB exactly over the course of one integration time-step.
        self.A_IB   = expm(delta_t*I_omega_IB) @ self.A_IB
        # Doing one-step Euler forward integration for angular velocity:
        self.B_omega_B  = self.B_omega_B + delta_t * (self.B_omegaDot_B - 0)


    def positionOfPoint( self, B_r_BQ ): # -> I_r_IQ
        I_r_IQ = self.A_IB @ (self.B_r_IB + B_r_BQ)
        return I_r_IQ
    
    def velocityOfPoint( self, B_r_BQ ): # -> I_v_Q
        B_omega_IB = skew(self.B_omega_B)
        I_v_Q = self.A_IB @ (self.B_v_B + B_omega_IB @ B_r_BQ)
        return I_v_Q
    
    def accelerationOfPoint( self, B_r_BQ ): # -> I_a_Q
        B_omega_IB = skew(self.B_omega_B)
        B_omegaDot_IB = skew(self.B_omegaDot_B)
        I_a_Q = self.A_IB @ (self.B_a_B + (B_omegaDot_IB + matrix_power(B_omega_IB,2) ) @ B_r_BQ)
        return I_a_Q

    def computeNaturalDynamics( self ):
        # Since no external forces or moments are acting, the change of
        # angular momentum and linear moment is zero:
        B_pDot   = zeros([3,1])
        B_LDot_B = zeros([3,1])
        # Compute the current angular momentum and the skew symmetric
        # matrix of B_omega_B
        B_L_B = self.B_I_B @ self.B_omega_B
        B_omega_IB = skew(self.B_omega_B)
        # Compute accelerations from the equations of motion of a rigid
        # body.  Note that instead of using inv(B_I_B), we're using the
        # matrix 'devision' '\' that Matlab implements ("...X = A\B is
        # the solution to the equation A*X = B..."):   
        self.B_a_B         = B_pDot / self.m_B
        self.B_omegaDot_B  = inv(self.B_I_B) @ (B_LDot_B - B_omega_IB @ B_L_B)
    

    def _recursiveForwardKinematics( self, B_r_IB, A_IB, B_omega_B, B_v_B, B_omegaDot_B, B_a_B, B_J_S, B_J_R ):
        # Position and orientation, as well as velocities and accelerations are given by the parent 
        # joint and passed in its call of 'recursiveForwardKinematics' 
        self.A_IB          = A_IB
        self.B_omega_B     = B_omega_B
        self.B_omegaDot_B  = B_omegaDot_B
        self.B_r_IB        = B_r_IB
        self.B_v_B         = B_v_B
        self.B_a_B         = B_a_B
        self.B_J_S         = B_J_S
        self.B_J_R         = B_J_R
        
        for childJoint in self.childJoints:
            childJoint._recursiveForwardKinematics(self.B_r_IB, self.A_IB, self.B_omega_B, self.B_v_B, self.B_omegaDot_B, self.B_a_B,  self.B_J_S, self.B_J_R)


    def _recursiveComputationOfMfg( self ): # -> [M, f, g]
        '''
            This method requires a model update with all generalized accelerations set to zero
            such that B_a_B and B_omegaDot_B represent bias accelerations and not real accelerations
        '''
        # Compute the components for this body:
        M =   self.B_J_S.T * self.m_B    @ self.B_J_S + \
              self.B_J_R.T @ self.B_I_B  @ self.B_J_R  
        f = - self.B_J_S.T * self.m_B    @ self.B_a_B - \
              self.B_J_R.T @ (self.B_I_B @ self.B_omegaDot_B + skew(self.B_omega_B) @ self.B_I_B @ self.B_omega_B)
        g =   self.B_J_S.T @ self.A_IB.T @ self.I_grav * self.m_B + \
              self.B_J_R.T @ self.A_IB.T @ zeros([3,1]) 

        for childJoint in self.childJoints:
            M_part, f_part, g_part = childJoint.sucBody._recursiveComputationOfMfg()
            M += M_part
            f += f_part
            g += g_part
        return [M, f, g]