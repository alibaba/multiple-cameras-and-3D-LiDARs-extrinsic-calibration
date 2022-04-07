import os
import re
import sys
import glob
import shutil
import argparse
import mv3dhelper
import zrpc
import time
import csv
import yaml
import numpy as np
from scipy.spatial.transform import Rotation

def help_information():
    print('''Usage: python run_imu_intrin_calib.py <workspace_folder> <dataset_folder> <output_folder> '''
        ''' <workspace_folder>: /code/kalibr_calibration. '''
        '''<dataset_folder>:  e.g.https://yuque.antfin-inc.com/aone857851/udonh7/bmazpr''')


def calibImuIntrin(calib_imu_intrin_exe, raw_imu_filepath, imu_name, max_time_min, max_cluster, res_folder):
    
    cmds = [calib_imu_intrin_exe, raw_imu_filepath, imu_name, max_time_min, max_cluster, res_folder]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    res_file = os.path.join(res_folder, 'backpack_imu.yaml')
    return res_file

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="A helper to caibrate imu intrinsic")

    parser.add_argument("ws_folder", type=str, default="",
                        help="the workspace folder")
    parser.add_argument('dataset_folder', type=str, default="",
                        help='the input dataset folder')
    parser.add_argument('output_folder', type=str, default='',
                        help='the folder of output files')
    parser.add_argument('--max_time_min', type=int,
                        default=120, help='data timestamp duration(min)')
    parser.add_argument('--max_cluster', type=int, default=1000,
                        help='the extension of image under dataset folder')

    args = parser.parse_args()
    ws_folder = args.ws_folder
    dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    max_time_min = args.max_time_min
    max_cluster = args.max_cluster

    mv3dhelper.create_folder_if_not_exists(output_folder)
    # exe path
    calib_imu_intrin_exe = os.path.join(ws_folder, 'build/test_imu_allan')

    time_start = time.time()
    timing_info = []
    
    ############################# calibrate IMU intrinsic ############################
    # raw data folder path
    raw_data_folder_list = glob.glob(os.path.join(dataset_folder, 'data*'))
    assert(len(raw_data_folder_list))
    # calibration results
    imu_intrin_list = []
    round_cnt = 0
    for folder in raw_data_folder_list:
        data_folder = os.path.join(folder, 'imu')
        res_folder = os.path.join(data_folder, 'results')
        raw_imu_filepath = os.path.join(data_folder, 'imu_0.txt')
        imu_name = 'xsens'
        if os.path.exists(raw_imu_filepath):
            imu_intrin_file = calibImuIntrin(calib_imu_intrin_exe, raw_imu_filepath, imu_name, max_time_min, max_cluster, res_folder)
            imu_intrin_list.append(imu_intrin_file)
        
            round_cnt += 1
            timing_info.append(('Round {} calibration of imu intrinsic'.format(round_cnt), time.time() - time_start))
            time_start = time.time()


    assert(len(imu_intrin_list))
    # copy to dataset root folder
    cmds = ['mv', imu_intrin_list[0], dataset_folder]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    print('------------------------------')
    for info in timing_info:
        print('| %s: %0.3f second(s)' % (info[0], info[1]))
    print('------------------------------')