import numpy as np


def t2v(A):
    v = np.zeros((3, 1))
    v[0:2, 0] = A[0:2, 2]
    v[2, 0] = np.arctan2(A[1, 0], A[0, 0])

    return v


def v2t(v):
    c = np.cos(v[2])
    s = np.sin(v[2])
    A = np.array([[c,  -s,  v[0]],
                  [s,   c,  v[1]],
                  [0,   0,  1]])

    return A


def str2double(S):
    if isinstance(S, list):
        X = np.array([], dtype=np.float64)
        for item in S:
            try:
                X = np.append(X, float(item))
            except Exception as e:
                X = np.append(X, np.nan)
    else:
        try:
            X = float(S)
        except:
            X = np.nan
    
    return X


def velMotionModel(state, ctrlCmd):
    # returns the next state based on the previous state and 
    # the translational and rotational velocity =: velocity motion model

    state = state.reshape((3, 1))
    deltaT = 0.1                    # time for which these velocities are executed
    v = ctrlCmd[0]                  # translation velocity
    w = ctrlCmd[1]                  # rotational velocity (omega)
    v_by_w = v/w                    # for reusability
    theta = state[2]
    
    update = np.array([(-v_by_w*np.sin(theta) + v_by_w*np.sin(theta + w*deltaT))[0],
                        (v_by_w*np.cos(theta) - v_by_w*np.cos(theta + w*deltaT))[0],
                        w*deltaT]).reshape((3, 1))
    
    state += update
    return state


class laserReading():
    # empty skeleton to hold data about each sensor reading
    def __init__(self) -> None:
        self.start_angle = 0
        self.angular_resolution = 0
        self.maximum_range = 0
        self.ranges = np.array([])
        self.pose = np.zeros((3,1))
        self.laser_offset = np.zeros((3,1))
        self.timestamp = 0


class odomReading():
    # empty skeleton to hold odometry readings
    def __init__(self) -> None:
        # self.x = 0
        # self.y = 0
        # self.theta = 0
        self.state = np.zeros((3, 1))
        # self.tv = 0
        # self.rv = 0
        self.ctrl = np.zeros((2, 1))
        self.accel = 0
        self.timestamp = 0


if __name__ == "__main__":
    pass