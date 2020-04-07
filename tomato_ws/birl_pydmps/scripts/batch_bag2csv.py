#!/usr/bin/env python
import numpy as np
import pandas as pd
import sys,os
import copy
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from mpl_toolkits.mplot3d import Axes3D
from scipy.ndimage.filters import gaussian_filter1d
import ipdb
import glob
import subprocess
import ConfigParser


# read conf file
file_path = os.path.dirname(__file__)

# load the raw data set
bag_path = "../data/"
datasets_raw_path = os.path.join(file_path, bag_path, 'raw')

def main():
    # run sh script for each rosbag file in datasets path
    task_path_list = glob.glob(os.path.join(datasets_raw_path, "*"))
    for task_path in task_path_list:
        demo_path_list = glob.glob(os.path.join(task_path, "*.bag"))
        for demo_path in demo_path_list:
            subprocess.Popen([os.path.join(file_path, 'bag_to_csv.sh') + ' ' +
                              demo_path + ' ' +
                              os.path.join(task_path, 'csv')], shell=True)  # non block function
                              # os.path.join(task_path, 'csv')], shell = True).wait()   # the block function
if __name__ == '__main__':
    main()




