from json import tool
import matplotlib.pyplot as plt
from os.path import exists
import read_data as rd
import numpy as np
import tools


def retVel():

    pass


filePath = "dataset/csail-dataset/csail.corrected.log"
# filePath = "dataset/csail-dataset/mit-csail-3rd-floor-2005-12-17-run4.log"
# filePath = "dataset/freiburg-dataset/fr-campus-20040714.carmen.log"
# filePath = "dataset/intel-dataset/intel.log"

savedFiles = ["laser.npy", "odom.npy"]

if exists(savedFiles[0]) and exists(savedFiles[1]):
    laser = np.load("laser.npy", allow_pickle=True)
    odom = np.load("odom.npy", allow_pickle=True)
    print("Saved file exists, loading.")
else:
    laser, odom = rd.read_data(filePath)
    np.save("laser.npy", laser)
    np.save("odom.npy", odom)
    print("Saved file does not exist. Saving.")

x1_odom = np.array([])
y1_odom = np.array([])
t1_odom = np.array([])

x2 = np.array([])
y2 = np.array([])
t2 = np.array([])

x3_robLas = np.array([])
y3_robLas = np.array([])
t3_robLas = np.array([])

x_vel = np.array([])
y_vel = np.array([])
t_vel = np.array([])

for odo in odom:
    x1_odom = np.append(x1_odom, odo.state[0])
    y1_odom = np.append(y1_odom, odo.state[1])
    t1_odom = np.append(t1_odom, odo.state[2])
    
    newState = tools.velMotionModel(odo.state, odo.ctrl)
    
    x_vel = np.append(x_vel, odo.state[0])
    y_vel = np.append(y_vel, odo.state[1])
    t_vel = np.append(t_vel, odo.state[2])
    
for rob in laser:
    # x2 = np.append(x2, rob.odomState[0])
    x3_robLas = np.append(x3_robLas, rob.pose[0])

    # y2 = np.append(y2, rob.odomState[1])
    y3_robLas = np.append(y3_robLas, rob.pose[1])

    # t2 = np.append(t2, rob.odomState[2])
    t3_robLas = np.append(t3_robLas, rob.pose[2])

z1 = np.arange(1, t1_odom.size+1)
z2 = np.arange(1, t2.size+1)
z3 = np.arange(1, t3_robLas.size+1)

plt.figure(1, figsize=(16,9))
plt.scatter(x1_odom, y1_odom, s=10) 
# plt.scatter(x2, y2, s=5, marker='x')
plt.scatter(x3_robLas, y3_robLas, s=5, marker='v')
# plt.scatter(x_vel, y_vel, s=7, marker='x')

# plt.figure(2, figsize=(16,9))
# plt.subplot(2, 1, 1)
# plt.scatter(z1, t1)

# plt.subplot(2, 1, 2)
# plt.scatter(z3, t3)
plt.show()

# plt.plot(t1)
# plt.plot(t2, '+')