import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay


class Inertia:
    def __init__(self, inertia=np.eye(3)):
        self.B_I_B = inertia

    def plot(self, axes):


    def getEllipsoid(self, radius=np.array([1,1,1])):
        # https://stackoverflow.com/questions/7819498/plotting-ellipsoid-with-matplotlib

        # your ellispsoid and center in matrix form
        A = np.array([[1,0,0],[0,2,0],[0,0,2]])
        center = [0,0,0]

        # find the rotation matrix and radii of the axes
        U, s, rotation = linalg.svd(A)
        radii = 1.0/np.sqrt(s)

        # now carry on with EOL's answer
        u = np.linspace(0.0, 2.0 * np.pi, 100)
        v = np.linspace(0.0, np.pi, 100)
        
        x = radii[0] * np.outer(np.cos(u), np.sin(v))
        y = radii[1] * np.outer(np.sin(u), np.sin(v))
        z = radii[2] * np.outer(np.ones_like(u), np.cos(v))
        for i in range(len(x)):
            for j in range(len(x)):
                [x[i,j],y[i,j],z[i,j]] = np.dot([x[i,j],y[i,j],z[i,j]], rotation) + center


        fig = plt.figure(figsize=plt.figaspect(1))  # Square figure
        ax = fig.add_subplot(111, projection='3d')
        ax.plot_surface(x, y, z,  rstride=4, cstride=4, color='b')