from tools import laserReading, odomReading
import numpy as np
import tools


def read_robotlaser(tokens):
    # order of each line of ROBOTLASER1:
    # ROBOTLASER1 laser_type start_angle field_of_view angular_resolution maximum_range accuracy remission_mode num_readings [range_readings] laser_pose_x laser_pose_y laser_pose_theta robot_pose_x robot_pose_y robot_pose_theta laser_tv laser_rv forward_safety_dist side_safty_dist
    # ipc_timestamp ipc_hostname logger_timestamp
    
    # all tokens converted into float values
    numTokens = tools.str2double(tokens)

    # to track the values present in each line
    col = 2 

    start_angle = numTokens[col]
    col += 2                        # 1: next, 1: skip FOV

    angular_resolution = numTokens[col]
    col += 1

    maximum_range = numTokens[col]
    col += 3                        # 1: next, 2: skip accuracy, remission_mode

    # the next numReadings values in the line are the laser ranges
    numReadings = int(numTokens[col])
    col += 1                        # 1: next

    # for reasons unknown, the original script from freiburg took 
    ranges = numTokens[col:col+numReadings]
    col += numReadings

    numRemissions = int(numTokens[col])
    col += 1 + numRemissions        # 1: next, and skip remissions

    laserPose = numTokens[col:col+3]
    col += 3                        # 3: laser pose

    pose = numTokens[col:col+3]
    col += 3                        # 3: robot pose

    # !! << this is not verified yet >> !!
    # translating the laser end points based on the current pose
    laserEndPtsTrans = np.matmul(tools.v2t(pose), tools.v2t(laserPose))
    invLaserOffset = np.linalg.inv(laserEndPtsTrans)
    laser_offset = tools.t2v(invLaserOffset)
    col += 5

    timestamp = numTokens[col]

    currentReading = laserReading(start_angle, angular_resolution, maximum_range, ranges, pose, laser_offset, timestamp)

    return currentReading


def read_odom(tokens):
    # order of each line of ODOM:
    # ODOM x y theta tv rv accel
    # ipc_timestamp ipc_hostname logger_timestamp
    
    # all tokens converted into float values
    numTokens = tools.str2double(tokens)
    
    col = 1

    state = numTokens[col:col+3]
    col += 3                            # x, y, theta

    ctrl = numTokens[col:col+2]
    col += 2                            # tv, rv

    accel = numTokens[col]
    col += 1                            # accel

    timestamp = numTokens[col]

    currentOdometry = odomReading(state, ctrl, accel, timestamp)

    return currentOdometry


def read_data(filePath):
    # empty array to place all readings
    laser = np.array([])
    odom = np.array([])

    # retrieve data from file
    with open(filePath) as f:
        lines = f.readlines()
    f.close()

    lineCount = 0

    # iterate over each line
    for line in lines:
        # divide line into tokens and skip if not a real line
        tokens = line.split()
        if tokens[0] == "ROBOTLASER1":
            currentReading = read_robotlaser(tokens)
            laser = np.append(laser, currentReading)
        elif tokens[0] == "ODOM":
            currentOdometry = read_odom(tokens)
            odom = np.append(odom, currentOdometry)
        lineCount +=1
    
    return laser, odom


if __name__ == "__main__":
    filePath = "csail-dataset/mit-csail-3rd-floor-2005-12-17-run4.log"
    laser, odom = read_data(filePath)