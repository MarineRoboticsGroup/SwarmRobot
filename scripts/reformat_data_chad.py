import rosbag
import rospy
from std_msgs.msg import Int32, String
from os import listdir, mkdir
from os.path import isfile, isdir, join
import matplotlib.pyplot as plt
import math
import Queue
from numpy import median
import numpy as np
import json
import re
from scipy.spatial.transform import Rotation as R

eps = 0.01


def dist_between_poses(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    dz = p1.z - p2.z
    dist_sq = dx**2 + dy**2 + dz**2
    dist = math.sqrt(dist_sq)
    return dist


def get_vicon_start_and_end(data_dir, trial_name, trial_span, near_start=0, normalize_time=True):
    """This function finds when the robot started moving according to the vicon data

    Args:
        trial_name (str): the name of the trial
        trial_span (float): the length over which the robot is expected to be moving
        near_start (int, optional): This is to skip false early movements. We will only look for the start of this movement after this many seconds have elapsed. Defaults to 0.
        normalize_time (bool, optional): Whether or not to time shift so the first data begins at t=0. Defaults to True.

    Returns:
        (float, float): the start and end times for the trial in this rosbag
    """
    robot_bag = join(data_dir, trial_name+"_vicon.bag")
    bag = rosbag.Bag(robot_bag)
    x_vel = []
    time = []
    shifted_time = []
    start_time = None
    end_time = None

    last_pose = None
    last_time = None
    cur_pose = None
    cur_time = None
    first_time = 0
    cnt = 0
    vel_queue = []
    queue_len = 20
    for topic, msg, t in bag.read_messages(topics=["/vicon/mrg_robot_tag/mrg_robot_tag"]):
        cur_pose = msg.transform.translation
        cur_time = t.to_sec()
        if first_time == 0:
            first_time = cur_time

        time_elapsed = cur_time - first_time

        if cnt > 0:
            d_dist = dist_between_poses(last_pose, cur_pose)
            d_time = cur_time - last_time
            vel = d_dist / d_time
            vel_queue.append(vel)
            if len(vel_queue) > queue_len:
                vel_queue.pop(0)
            if cnt > 10:
                avg_vel = sum(vel_queue) / len(vel_queue)
                avg_vel = median(vel_queue)
                x_vel.append(avg_vel)
                time.append(cur_time)
                shifted_time.append(time_elapsed)

                if time_elapsed > near_start and avg_vel > eps and start_time is None:
                    start_time = cur_time

        last_pose = cur_pose
        last_time = cur_time
        first_msg = False
        cnt += 1

    end_time = start_time+trial_span
    print("time", start_time, end_time)
    if normalize_time:
        plt.plot(shifted_time, x_vel)
        plt.vlines([start_time-first_time, end_time-first_time],
                   0, .5, linestyles='dashed', colors='green')
    else:
        plt.plot(time, x_vel)
        plt.vlines([start_time, end_time], 0, .5,
                   linestyles='dashed', colors='green')
    plt.show(block=True)

    return start_time, end_time


def get_robot_start_and_end(data_dir, trial_name, normalize_time=True):
    robot_bag = join(data_dir, trial_name+"_robot.bag")
    bag = rosbag.Bag(robot_bag)
    x_vel = []
    time = []
    shifted_time = []
    start_time = None
    end_time = None
    first_time = 0
    for topic, msg, t in bag.read_messages(topics=["/dynamixel_workbench/cmd_vel"]):
        vel = msg.linear.x
        cur_time = t.to_sec()
        if first_time == 0 and normalize_time:
            first_time = cur_time

        time_elapsed = cur_time - first_time

        if not vel == 0 and start_time is None:
            start_time = cur_time

        x_vel.append(vel)
        time.append(cur_time)
        shifted_time.append(time_elapsed)
        end_time = cur_time

    print("time", start_time, end_time)
    if normalize_time:
        plt.plot(shifted_time, x_vel)
        plt.vlines([start_time-first_time, end_time-first_time],
                   0, 1, linestyles='dashed', colors='green')
    else:
        plt.plot(time, x_vel)
        plt.vlines([start_time, end_time], 0, 1,
                   linestyles='dashed', colors='green')
    plt.show(block=True)
    return start_time, end_time, end_time-start_time


def merge_rosbags(data_dir, trial_name, robot_start, vicon_start, trial_span):
    results_dir = join(data_dir, trial_name)
    merged_bag_path = join(results_dir, trial_name+"_merged.bag")
    merged_bag = rosbag.Bag(merged_bag_path, mode='w')

    # time shift so that each bag is synchronized. Arbitrarily chose to
    # synchronize to vicon time
    time_shift = rospy.Duration.from_sec(vicon_start - robot_start)
    trial_duration = rospy.Duration.from_sec(trial_span)

    robot_start_time = rospy.Time.from_sec(robot_start)
    robot_bag_path = join(data_dir, trial_name+"_robot.bag")
    robot_bag = rosbag.Bag(robot_bag_path)
    for topic, msg, t in robot_bag.read_messages(start_time=robot_start_time, end_time=robot_start_time+trial_duration):
        merged_bag.write(topic, msg, t+time_shift)

    vicon_start_time = rospy.Time.from_sec(vicon_start)
    vicon_bag_path = join(data_dir, trial_name+"_vicon.bag")
    vicon_bag = rosbag.Bag(vicon_bag_path)
    for topic, msg, t in vicon_bag.read_messages(start_time=vicon_start_time, end_time=vicon_start_time + trial_duration):
        merged_bag.write(topic, msg, t)

    merged_bag.close()


def get_trial_names(dir_path):

    def get_trial_from_filename(filename):
        lower_name = filename.lower()
        end_index = max(lower_name.find("_robot"), lower_name.find(
            "_vicon"), lower_name.find("_merged"))
        trial_name = lower_name[:end_index]
        return trial_name

    filenames = [f for f in listdir(dir_path) if isfile(join(dir_path, f))]
    trialnames = set([get_trial_from_filename(x) for x in filenames])
    return trialnames


def merged_bag_exists(data_dir, trial_name):
    results_dir = join(data_dir, trial_name)
    merged_bag_path = join(results_dir, trial_name+"_merged.bag")
    return isfile(merged_bag_path)


def odometry_from_wheel_velocity(right_vel, left_vel, delta_t):
    """Estimates the odometry of the robot based on the wheel geometry and velocities

    Args:
        right_vel ([type]): [description]
        left_vel ([type]): [description]

    Raises:
        NotImplementedError: [description]

    Returns:
        [type]: [description]
    """
    r = 0.06/2.0  # radius of the wheels
    b = 0.305/2.0  # distance from wheel to center of vehicle
    J = np.array([[r/2, r/2], [0, 0], [r/(2*b), -r/(2*b)]])
    theta_dot = np.array([right_vel, left_vel])
    v = J.dot(theta_dot) * delta_t
    d_x = v[0]
    d_theta = v[2]
    return d_x, d_theta


def convert_results_to_plaza(data_dir, trial_name):
    """Converts from rosbag to same format as the plaza dataset

    # % GT: Groundtruth path from GPS
    # %    Time (sec)    X_pose (m)    Y_pose (m)    Heading (rad)
    # % DR: Odometry Input (delta distance traveled and delta heading change)
    # %    Time (sec)    Delta Dist. Trav. (m)    Delta Heading (rad)
    # % DRp: Dead Reckoned Path from Odometry
    # %    Time (sec)    X_pose (m)    Y_pose (m)    Heading (rad)
    # % TL: Surveyed Node Locations
    # %    Time (sec)    X_pose (m)    Y_pose (m)
    # % TD
    # %    Time (sec)    Sender / Antenna ID    Receiver Node ID    Range (m)

    Args:
        data_dir (str): base directory where the data is held
        trial_name (str): name of the specific trial

    """

    def write_drp_from_dr(dr_path, drp_path, start_pose, time_start):
        # %    Time (sec)    X_pose (m)    Y_pose (m)    Heading (rad)
        x_start, y_start, heading_start = start_pose
        current_x = x_start
        current_y = y_start
        current_heading = heading_start

        data_dr = open(dr_path, 'r')
        data_drp = open(drp_path, 'w')
        data_drp.write("timestamp(sec) x(m) y(m) theta(rad)\n")
        data_drp.write('{} {} {} {}\n'.format(
            time_start, x_start, y_start, heading_start))

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

    def get_uwb_addr_from_vicon_topic(topic):
        """Takes a topic published from vicon and finds the uwb address inside the topic name

        Args:
            topic (str): the topic name
        """
        search_str = '(?<=mrg_dw)....'
        addr = re.search(search_str, topic).group(0)
        return addr

    def get_anchor_addrs_map(bag_obj):
        """takes a bag object and builds an ID map for the UWB anchor addresses inside it by looking at the vicon topics

        Args:
            bag_obj (rosbag.Bag): rosbag object from a specific trial

        Returns:
            dict: [description]
        """
        topics = merged_bag.get_type_and_topic_info()[1].keys()
        topics = [topic for topic in topics if 'vicon/mrg_dw' in topic]
        anchor_addrs = [get_uwb_addr_from_vicon_topic(
            topic) for topic in topics]
        anchor_addrs.sort()
        id_map = {}
        for i, addr in enumerate(anchor_addrs):
            id_map[addr] = i
        return id_map

    results_dir = join(data_dir, trial_name)

    merged_bag_path = join(results_dir, trial_name+"_merged.bag")
    merged_bag = rosbag.Bag(merged_bag_path)
    topics = merged_bag.get_type_and_topic_info()[1]

    # print(topics['/dynamixel_workbench/dynamixel_state'])
    # if "vicon/mrg_dw" # gt_pose
    # elif "vicon/mrg_robot" # gt_pose
    # elif "dynamixel_state" # dr_pose
    # elif "uwb_msg" # range measurements

    gt_path = join(results_dir, trial_name+"_GT.txt")
    dr_path = join(results_dir, trial_name+"_DR.txt")
    td_path = join(results_dir, trial_name+"_TD.txt")

    data_gt = open(gt_path, 'w')
    data_gt.write("timestamp(sec) x(m) y(m) theta(rad)\n")

    data_dr = open(dr_path, 'w')
    data_dr.write("timestamp(sec) d_x(m) d_theta(rad)\n")

    data_td = open(td_path, 'w')
    data_td.write("timestamp(sec) robot_id lmk_id range(m)\n")

    anchor_id_map = get_anchor_addrs_map(merged_bag)
    anchor_ids = anchor_id_map.keys()
    anchor_x_vicon_locs = [[] for anchor_id in anchor_ids]
    anchor_y_vicon_locs = [[] for anchor_id in anchor_ids]

    odom_last_time = None
    x_start = None
    y_start = None
    heading_start = None
    time_start = None
    for topic, msg, t in merged_bag.read_messages():
        if time_start is None:
            time_start = t.to_sec()

        if "vicon/mrg_dw" in topic:
            # write to TL (tag location)
            anchor_addr = get_uwb_addr_from_vicon_topic(topic)
            anchor_id = anchor_id_map[anchor_addr]
            anchor_x = msg.transform.translation.x
            anchor_y = msg.transform.translation.y
            anchor_x_vicon_locs[anchor_id].append(anchor_x)
            anchor_y_vicon_locs[anchor_id].append(anchor_y)
        elif "vicon/mrg_robot" in topic:
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
            if x_start is None:
                x_start = robot_x
                y_start = robot_y
                heading_start = robot_heading

            data_gt.write('{} {} {} {}\n'.format(
                time, robot_x, robot_y, robot_heading))
        elif "dynamixel_state" in topic:
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
                if d_x < 0.5 and d_theta < 0.5:
                    data_dr.write('{} {} {}\n'.format(time, d_x, d_theta))

            odom_last_time = time
        elif "uwb_msg" in topic:
            # write to TD (tag distance measurements)
            # timestamp(sec) robot_id lmk_id range(m)
            time = t.to_sec()
            data = msg.uwb
            dist = float(data.dist)
            addr = data.addr
            anchor_id = anchor_id_map[addr]
            data_td.write('{} {} {} {}\n'.format(
                time, len(anchor_ids), anchor_id, dist))

    data_gt.close()
    data_dr.close()
    data_td.close()

    # write vicon anchor locations
    tl_path = join(results_dir, trial_name+"_TL.txt")
    data_tl = open(tl_path, 'w')
    data_tl.write("lmd_id x(m) y(m)\n")
    for i in range(len(anchor_ids)):
        x = median(anchor_x_vicon_locs[i])
        y = median(anchor_y_vicon_locs[i])
        data_tl.write('{} {} {}\n'.format(i, x, y))
    data_tl.close()

    start_pose = (x_start, y_start, heading_start)
    drp_path = join(results_dir, trial_name+"_DRp.txt")
    write_drp_from_dr(dr_path, drp_path, start_pose, time_start)


# TODO actually write this function
def results_are_converted(data_dir, trial_name):
    return False


if __name__ == "__main__":
    # TODO check to make sure that robot z is relatively constant

    data_dir = "/home/alan/swarmbot_vicon_data/"
    with open('near_trial_start.json') as json_file:
        begin_check_start = json.load(json_file)

    trials = get_trial_names(data_dir)
    for trial_name in trials:
        assert trial_name in begin_check_start, "Do not have start time for this trial. Please correct in the .json file"

        results_dir = join(data_dir, trial_name)
        if not isdir(results_dir):
            mkdir(results_dir)

        print(trial_name)
        if not merged_bag_exists(data_dir, trial_name):
            robot_start, robot_end, trial_span = get_robot_start_and_end(
                data_dir, trial_name)
            vicon_start, vicon_end = get_vicon_start_and_end(
                data_dir, trial_name, trial_span, begin_check_start[trial_name])
            merge_rosbags(data_dir, trial_name, robot_start,
                          vicon_start, trial_span)

        if not results_are_converted(data_dir, trial_name):
            convert_results_to_plaza(data_dir, trial_name)

        print
