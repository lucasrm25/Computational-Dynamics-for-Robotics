import numpy as np

def skew(vec):
    ''' 
    Generates a skew-symmetric matrix given a vector w
    '''
    S = np.zeros([3,3])

    S[0,1] = -vec[2]
    S[0,2] =  vec[1]
    S[1,2] = -vec[0]
    
    S[1,0] =  vec[2]
    S[2,0] = -vec[1]
    S[2,1] =  vec[0]
    
    return S