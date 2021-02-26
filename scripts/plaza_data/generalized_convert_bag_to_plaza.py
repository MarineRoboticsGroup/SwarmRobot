import rosbag
import rospy
from std_msgs.msg import Int32, String
from os import listdir, mkdir
from os.path import isfile, isdir, join
import matplotlib.pyplot as plt
import math
import numpy as np
import json
import re
from scipy.spatial.transform import Rotation as R


def get_trial_names(dir_path):
    """Gets all of the different trial names inside the given directory. The
    directory this should be used on is the root directory of all of the
    collected data. This assumes that every intended "trial" file is a rosbag

    Args:
        dir_path (str): the directory holding all of the trial data

    Returns:
        set: all found trialnames
    """

    def get_trial_from_filename(filename):
        end_index = filename.find(".bag")
        if end_index < 0:
            assert False, filename+" is not the specified format (.bag)"
        trial_name = filename[:end_index]
        return trial_name

    file_names = [f for f in listdir(dir_path) if isfile(join(dir_path, f))]
    trial_names = set([get_trial_from_filename(x) for x in file_names])
    return trial_names


def results_are_converted(data_dir, trial_name):
    """checks to see if the desired conversion files have all already been
    generated

    Args:
        data_dir (str): the directory holding all of the trial data
        trial_name (str): name of the current trial

    Returns:
        boolean: true if there exist the corresponding plaza format
    """
    results_dir = join(data_dir, trial_name)
    file_extensions = ["_GT.txt", "_DR.txt", "_DRp.txt"] #ground truth, odometry, dead reckoning
    file_base = join(results_dir, trial_name)
    for ext in file_extensions:
        file_path = file_base + ext
        if not isfile(file_path):
            print("Converting {} to Plaza format".format(trial_name))
            return False

    print("{} already converted to Plaza format".format(trial_name))
    return True


def odometry_from_wheel_velocity(right_vel, left_vel, delta_t):
    """Estimates the odometry of the robot based on the wheel geometry and velocities

    Args:
        right_vel (float): right wheel velocity
        left_vel (float): left wheel velocity

    Returns:
        (float, float): change in x and theta
    """
    r = 0.06/2.0  # radius of the wheels
    b = 0.305/2.0  # distance from wheel to center of vehicle
    J = np.array([[r/2, r/2], [0, 0], [r/(2*b), -r/(2*b)]])
    theta_dot = np.array([right_vel, left_vel])/60
    v = J.dot(theta_dot) * delta_t
    d_x = v[0]
    d_theta = v[2]
    return d_x, d_theta


def convert_results_to_plaza(data_dir, trial_name, robot_names):
    """Converts from rosbag to same format as the plaza dataset

    # % GT: Groundtruth path from GPS
    # %    Time (sec)    X_pose (m)    Y_pose (m)    Heading (rad)
    # % DR: Odometry Input (delta distance traveled and delta heading change)
    # %    Time (sec)    Delta Dist. Trav. (m)    Delta Heading (rad)
    # % DRp: Dead Reckoned Path from Odometry
    # %    Time (sec)    X_pose (m)    Y_pose (m)    Heading (rad)

    Args:
        data_dir (str): base directory where the data is held
        trial_name (str): name of the  trial

    """

    def write_drp_from_dr(dr_path, drp_path, start_pose, time_start):
        # %    Time (sec)    X_pose (m)    Y_pose (m)    Heading (rad)
        x_start_vicon, y_start_vicon, heading_start_vicon = start_pose
        current_x = x_start_vicon
        current_y = y_start_vicon
        current_heading = heading_start_vicon

        data_dr = open(dr_path, 'r')
        data_drp = open(drp_path, 'w')
        data_drp.write("timestamp(sec) x(m) y(m) theta(rad)\n")
        data_drp.write('{} {} {} {}\n'.format(
            time_start, x_start_vicon, y_start_vicon, heading_start_vicon))

        # skip over header line in file
        first_line = True
        for line in data_dr:
            if first_line:
                first_line = False
                continue
            time, d_x, d_theta = [float(val) for val in line.split()]
            current_x += d_x * np.cos(current_heading)
            current_y += d_x * np.sin(current_heading)
            current_heading += d_theta
            data_drp.write('{} {} {} {}\n'.format(
                time, current_x, current_y, current_heading))

        data_dr.close()

    # merge save / read input bag
    results_dir = join(data_dir, trial_name)

    merged_bag_path = join(data_dir, trial_name+".bag")
    merged_bag = rosbag.Bag(merged_bag_path)

    # make sure needed topics exist
    topic_info = merged_bag.get_type_and_topic_info()[1]
    topics = topic_info.keys()

    for name in robot_names:
        dynamixel_topic = "/"+name+"/dynamixel_state"
        vicon_topic = "/vicon/mrg_"+name+"/mrg_"+name
        assert dynamixel_topic in topics, "No dynamixel data found in rosbag"
        assert vicon_topic in topics, "No vicon groundtruth robot location data found in rosbag"

    for robot in robot_names:
        # separate ground truth, dead reckoning, and range measurments
        gt_path = join(results_dir, trial_name+"_"+robot+"_GT.txt")
        dr_path = join(results_dir, trial_name+"_"+robot+"_DR.txt")

        data_gt = open(gt_path, 'w')
        data_gt.write("timestamp(sec) x(m) y(m) theta(rad)\n")

        data_dr = open(dr_path, 'w')
        data_dr.write("timestamp(sec) d_x(m) d_theta(rad)\n")

        # previous timestamp
        odom_last_time = None
        # robot starting vals
        x_start_vicon = None
        y_start_vicon = None
        heading_start_vicon = None

        time_start = None
        for topic, msg, t in merged_bag.read_messages():
            if time_start is None:
                time_start = t.to_sec()

            if "/vicon/mrg_"+robot in topic:
                # write to GT (groundtruth robot location)
                # time, x(m), y(m), theta(rad)
                time = t.to_sec()
                robot_x = msg.transform.translation.x
                robot_y = msg.transform.translation.y

                # to make things simple we transform from quaternion into a rotation
                # vector to get the heading
                rotation = msg.transform.rotation
                a = rotation.x
                b = rotation.y
                c = rotation.z
                d = rotation.w
                r = R.from_quat([a, b, c, d])
                robot_heading = r.as_rotvec()[2]
                # print('{} {}'.format("Heading: ", robot_heading))
                if x_start_vicon is None:
                    x_start_vicon = robot_x
                    y_start_vicon = robot_y
                    heading_start_vicon = robot_heading

                data_gt.write('{} {} {} {}\n'.format(
                    time, robot_x, robot_y, robot_heading))

            elif robot+"/dynamixel_state" in topic:
                # write to DR (deadreckoned moves)
                # time, d_x(m), d_theta(rad)

                time = t.to_sec()
                right_vel = msg.dynamixel_state[0].present_velocity
                left_vel = msg.dynamixel_state[1].present_velocity

                if odom_last_time:
                    delta_t = time - odom_last_time
                    d_x, d_theta = odometry_from_wheel_velocity(
                        right_vel, left_vel, delta_t)

                    # reject large noise data
                    if abs(d_x) < 0.5 and abs(d_theta) < 0.5:
                        data_dr.write('{} {} {}\n'.format(time, d_x, d_theta))

                odom_last_time = time

        data_gt.close()
        data_dr.close()

        start_pose_vicon = (x_start_vicon, y_start_vicon, heading_start_vicon)
        drp_path = join(results_dir, trial_name+"_"+robot+"_DRp.txt")
        write_drp_from_dr(dr_path, drp_path, start_pose_vicon, time_start)


if __name__ == "__main__":
    """ Looks through directory specified and converts the rosbag to the
    same format as the Plaza dataset for only the vicon and dynamixel data (not uwb or orb slam).

    Resulting files are saved in a subdiractory with the original file's name

    To run just specify the directory in which bag files are held and make sure
    that there are rosbags for each data collection trial.
    """

    data_dir = "/home/thumman/Desktop/swarmbot-related/initial_experiment_rosbags"

    robot_names = ["archimedes", "mrg1", "pythagoras", "grace", "mrg2"] #susan not included in the current experiment

    trials = get_trial_names(data_dir)
    for trial_name in trials:

        results_dir = join(data_dir, trial_name)
        if not isdir(results_dir):
            mkdir(results_dir)

        if not results_are_converted(data_dir, trial_name):
            convert_results_to_plaza(data_dir, trial_name, robot_names)