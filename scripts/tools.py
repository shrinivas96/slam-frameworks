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
    def __init__(self, start_angle, ang_res, max_range, ranges, pose, laser_offset, timestamp) -> None:
        self.start_angle = start_angle
        self.angular_resolution = ang_res
        self.maximum_range = max_range
        self.ranges = ranges
        self.pose = pose.reshape((3,1))
        self.laser_offset = laser_offset.reshape((3,1))
        self.timestamp = timestamp

    def getter(self):
        print("Values for laser reading:")
        print("Start angle: \t", self.start_angle)
        print("Ang Res: \t", self.angular_resolution)
        print("Max Range: \t", self.maximum_range)
        print("Ranges: \t", self.ranges[:4], "...", self.ranges[-4:])
        print("Pose: \t\t", self.pose)
        print("Timestamp: \t", self.timestamp)


class odomReading():
    # empty skeleton to hold odometry readings
    def __init__(self, state, ctrl, accel, timestamp) -> None:
        # self.x = 0
        # self.y = 0
        # self.theta = 0
        self.state = state.reshape((3, 1))
        # self.tv = 0
        # self.rv = 0
        self.ctrl = ctrl.reshape((2, 1))
        self.accel = accel
        self.timestamp = timestamp

    def getter(self):
        print("Values for laser reading:")
        print("State: \t\t", self.state.reshape((3,)))
        print("Control: \t", self.ctrl.reshape((2,)))
        print("Timestamp: \t", self.timestamp)


def roughPad():
    start_angle = 0.15
    ang_res = 0.0824
    max_range = 80
    ranges = np.arange(361)
    pose = np.array([[1], [2], [3]]).reshape((1, 3))
    laser_offset = np.zeros((3, 1))
    timestamp = 1234

    obj = laserReading(start_angle, ang_res, max_range, ranges, pose, laser_offset, timestamp)
    
    # obj.getter()
    
    state = np.array([1, 2, 3])
    ctrl = np.array([1, 2])
    accel = 0
    timestamp = 1234
    obj = odomReading(state, ctrl, accel, timestamp)
    
    obj.getter()


if __name__ == "__main__":
    pass