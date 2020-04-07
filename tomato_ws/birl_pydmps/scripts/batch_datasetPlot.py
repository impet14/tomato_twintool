import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import ipdb
import os
import dill


# the current file path
file_path = os.path.dirname(__file__)

# set models params
datasets_path  = "../data/"

# the pkl data
datasets_pkl_path = os.path.join(datasets_path, 'pkl')
datasets_norm_path = os.path.join(datasets_pkl_path, 'datasets_norm')
task_name_path = os.path.join(datasets_pkl_path, 'task_name_list')

def main():
    # load the data from pkl
    # ipdb.set_trace()
    with open(datasets_norm_path, 'rb') as in_strm:
        datasets_norm = dill.load(in_strm)
    with open(task_name_path, 'rb') as in_strm:
        task_name_list = dill.load(in_strm)

    for task_idx, task_norm in enumerate(datasets_norm):
        # fig = plt.figure(task_idx,figsize=(10,8))
        # ax = fig.gca(projection='3d')
        for demo_idx, demo in enumerate(task_norm):
            fig = plt.figure(demo_idx,figsize=(10,8))
            ax = fig.gca(projection='3d')
            ax.plot(demo[:, 0], demo[:, 1], demo[:, 2], linewidth=1, linestyle='-')
    plt.show()



if __name__ == '__main__':
    main()
