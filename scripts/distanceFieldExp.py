"""
Plot the interpolated surface of the distance field from the rescaled D1 matrix
"""

import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
import scipy as sc

distFields = sc.io.loadmat('/home/shrini/thesis-workspace/slam-frameworks/scripts/distanceField.mat')
D1 = distFields["D1"]
D1res = distFields["D2"]

x, y = D1.shape
xarr = np.arange(x)
yarr = np.arange(y)

interpDT = sc.interpolate.RectBivariateSpline(xarr, yarr, D1res)
Xq, Yq = np.meshgrid(np.linspace(0, x, 5000), np.linspace(0, y, 5000))
results = interpDT.__call__(Xq, Yq, grid=False)


fig = plt.figure()
ax = fig.gca(projection='3d')
# , rstride=1, cstride=1, cmap=cm.coolwarm, linewidth=0.1)
surf = ax.plot_surface(Xq, Yq, results, cmap=cm.coolwarm, linewidth=0.1)
# surf=ax.plot_surface(xg,yg,zSpline(xg,yg),rstride=1,cstride=1,cmap=cm.coolwarm,linewidth=0.1)
plt.show()
