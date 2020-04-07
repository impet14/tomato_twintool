#!/usr/bin/python
import numpy as np
import pandas as pd
from scipy.interpolate import griddata
from sklearn.externals import joblib
import dill
import glob
import os
import ConfigParser
from sklearn import preprocessing
from scipy.ndimage.filters import gaussian_filter1d
import ipdb
from birl_pydmps import DMPs_rhythmic
from birl_pydmps import DMPs_discrete

# the current file path
file_path = os.path.dirname(__file__)

# set models params
datasets_path  = "../data/"
len_norm = 101
n_dmps = 3
n_bfs = 200

# the pkl data
datasets_pkl_path = os.path.join(datasets_path, 'pkl')
datasets_norm_path = os.path.join(datasets_pkl_path, 'datasets_norm.pkl')
task_name_path = os.path.join(datasets_pkl_path, 'task_name_list.pkl')

def main():
    # load the data from pkl
    datasets_norm = joblib.load(datasets_norm_path)
    task_name_list = joblib.load(task_name_path)
    for idx, task_norm in enumerate(datasets_norm):
        demo_mean = np.mean(task_norm, axis=0)
        # create DMPs instance
        dmp_discrete = DMPs_discrete(n_dmps=n_dmps, n_bfs=n_bfs)
        dmp_rhythmic = DMPs_rhythmic(n_dmps=n_dmps, n_bfs=n_bfs)
        dmp_discrete.imitate_path(y_des=demo_mean.T)
        dmp_rhythmic.imitate_path(y_des=demo_mean.T)
        # save the trained models
        
        dill.dump(dmp_discrete, open(os.path.join(datasets_pkl_path, 
                  str(task_name_list[idx])+'_discrete'), 'w'))

        dill.dump(dmp_rhythmic, open(os.path.join(datasets_pkl_path, 
                  str(task_name_list[idx])+'_rhythmic'), 'w'))        
        # ipdb.set_trace()          
        # joblib.dump(dmp_discrete, os.path.join(datasets_pkl_path, str(task_name_list[idx])+'discrete.pkl'))
        # joblib.dump(dmp_rhythmic, os.path.join(datasets_pkl_path, str(task_name_list[idx])+'rhythmic.pkl'))

    print('Trained the DMPs successfully!!!')


if __name__ == '__main__':
    main()
