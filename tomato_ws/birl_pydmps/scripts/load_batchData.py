#!/usr/bin/python
import numpy as np
import pandas as pd
from scipy.interpolate import griddata
import dill
import glob
import os
import ConfigParser
from sklearn import preprocessing
from scipy.ndimage.filters import gaussian_filter1d
import ipdb

# the current file path
file_path = os.path.dirname(__file__)

# set models params
datasets_path  = "../data/"
len_norm = 101


def main():
    # datasets-related info
    datasets_raw_path = os.path.join(file_path, datasets_path, 'raw')
    task_path_list = glob.glob(os.path.join(datasets_raw_path, "*"))
    task_name_list = [task_path.split('/')[-1] for task_path in task_path_list]
    datasets_raw = []
    datasets_filtered = []
    datasets_norm = []
    for task_path in task_path_list:
        demo_path_list = glob.glob(os.path.join(task_path,"csv","*"))
        raw_data_list = []
        filtered_data_list = []
        norm_data_list = []
        for demo_path in demo_path_list:
            data_csv = pd.read_csv(os.path.join(demo_path, 'hand_marker.csv'))
            raw_data = data_csv.values[:, 24:27].astype(float)
            filtered_data = gaussian_filter1d(raw_data.T, sigma=5).T

            time_stamp = np.linspace(0,1,len(filtered_data))
            grid = np.linspace(0, 1, len_norm)
            data_norm = griddata(time_stamp, filtered_data, grid, method='linear')
            raw_data_list.append(raw_data)
            filtered_data_list.append(filtered_data)
            norm_data_list.append(data_norm)
            
        datasets_raw.append(raw_data_list)
        datasets_filtered.append(filtered_data_list)
        datasets_norm.append(norm_data_list)
    # ipdb.set_trace()
    # save all the datasets
    print('Saving the datasets as pkl ...')
    dill.dump(task_name_list, open(os.path.join(datasets_path, 'pkl/task_name_list'),'w'))
    dill.dump(datasets_raw, open(os.path.join(datasets_path, 'pkl/datasets_raw'),'w'))
    dill.dump(datasets_filtered, open(os.path.join(datasets_path, 'pkl/datasets_filtered'),'w'))
    dill.dump(datasets_norm, open(os.path.join(datasets_path, 'pkl/datasets_norm'),'w'))


    # the finished reminder
    print('Loaded, filtered, normalized, preprocessed and saved the datasets successfully!!!')


if __name__ == '__main__':
    main()
