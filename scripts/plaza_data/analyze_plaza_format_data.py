from os import listdir, mkdir
from os.path import isfile, isdir, join
from math import sqrt, pi
import matplotlib.pyplot as plt
from convert_bag_to_plaza import get_trial_names
import pandas as pd

def get_groundtruth_odometry_data(data_dir, trial_name):
    """Parses through groundtruth position txt file written in plaza format and returns the change in location and heading at each timestep. This is for comparison to our odometry estimates.

    Args:
        data_dir (str): the root directory that holds all of the recorded data
        trial_name (str): the name of the trial in format specified in the function get_trial_names in convert_bag_to_plaza.py

    Returns:
        [type]: [description]
    """
    results_dir = join(data_dir, trial_name)
    gt_filename = trial_name +"_GT.txt"
    gt_path = join(results_dir, gt_filename)

    first_line = True
    data_lines = []
    times = []
    column_names = ['time(s)', 'delta_loc(m)', 'delta_heading(rad)']
    x_last = None
    y_last = None
    heading_last = None
    data_gt = open(gt_path, 'r')
    for line in data_gt:
        if first_line:
            first_line = False
            continue
        time, x, y, heading = [float(val) for val in line.split()]
        if x_last and y_last and heading_last:
            delta_dist = sqrt((x_last-x)**2 + (y_last-y)**2)
            delta_heading = heading-heading_last

            # correct due to sometimes jumping by 2pi
            if delta_heading < -5:
                delta_heading += 2*pi
            elif delta_heading > 5:
                delta_heading -= 2*pi
            line_data = [time, delta_dist, delta_heading]
            data_lines.append(line_data)

        x_last = x
        y_last = y
        heading_last = heading
    gt_dataframe = pd.DataFrame(data_lines, columns=column_names)
    return gt_dataframe

def get_deadreckoned_odometry_data(data_dir, trial_name):
    """Parses through the deadreckoned odometry txt file written in plaza format and returns the change in location and heading at each timestep. This is for comparison of our odometry estimates to groundtruth data.

    Args:
        data_dir (str): the root directory that holds all of the recorded data
        trial_name (str): the name of the trial in format specified in the function get_trial_names in convert_bag_to_plaza.py

    Returns:
        [type]: [description]
    """
    results_dir = join(data_dir, trial_name)
    dr_filename = trial_name +"_DR.txt"
    dr_path = join(results_dir, dr_filename)

    column_names = ['time(s)', 'delta_loc(m)', 'delta_heading(rad)']
    first_line = True
    data_lines = []
    data_dr = open(dr_path, 'r')
    for line in data_dr:
        if first_line:
            first_line = False
            continue
        line_data = [float(val) for val in line.split()]
        data_lines.append(line_data)
    dr_dataframe = pd.DataFrame(data_lines, columns=column_names)
    return dr_dataframe

def compare_deadreckon_to_groundtruth(data_dir):
    trial_names = get_trial_names(data_dir)
    for t_name in trial_names:

        fig, axes = plt.subplots(nrows=2, ncols=1)
        fig.suptitle(t_name, fontsize=16)
        gt_dataframe = get_groundtruth_odometry_data(data_dir, t_name)
        dr_dataframe = get_deadreckoned_odometry_data(data_dir, t_name)
        dr_dataframe['delta_loc(m)']
        dr_dataframe['delta_heading(rad)']

        gt_dataframe.plot.line(x='time(s)', y='delta_loc(m)', ax=axes[0])
        dr_dataframe.plot.line(x='time(s)', y='delta_loc(m)', ax=axes[0])
        axes[0].legend(["Groundtruth", "Deadreckoned"])
        axes[0].set_title("Delta Location")

        gt_dataframe.plot.line(x='time(s)', y='delta_heading(rad)', ax=axes[1])
        dr_dataframe.plot.line(x='time(s)', y='delta_heading(rad)', ax=axes[1])
        axes[1].legend(["Groundtruth", "Deadreckoned"])
        axes[1].set_title("Delta Heading")
        plt.show(block=True)



if __name__ == "__main__":
    """General functions to plot and visualize the results of the data obtained
    """

    data_dir = "/home/alan/swarmbot_vicon_data/"
    compare_deadreckon_to_groundtruth(data_dir)
