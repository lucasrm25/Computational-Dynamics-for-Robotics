from vpython import vector, ellipsoid
import numpy as np
from scipy.spatial.transform import Rotation as R

class vellipsoid:
    
    def __init__(self, pos, color, size):
        self.ell = ellipsoid(pos=pos, color=color, size=size)
        self.A_IB = np.eye(3)

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
        self.ell.rotate( angle=np.linalg.norm(rot), axis=vector(*rot) )
        self.A_IB = A_IB