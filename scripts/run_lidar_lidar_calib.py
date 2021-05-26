from genericpath import exists
import os
import sys
import glob
import shutil
import argparse
import mv3dhelper
import zrpc
import time
import numpy as np
from scipy.spatial.transform import Rotation

def help_information():
    print('''Usage: python run_lidar_lidar_calib.py <workspace_folder> <dataset_folder> <output_folder> '''
        ''' <workspace_folder>: /code/kalibr_calibration. '''
        '''<dataset_folder>:  e.g.https://yuque.antfin-inc.com/aone857851/udonh7/bmazpr''')

def calibLidarToLidarExt(calib_lidar_exe, data_folder, res_folder, init_T_filepath):
    # test_lidar2lidar_calibration [T_l0_l1_init.yaml] [input_dataset_folder] [output_folder]
    cmds = [calib_lidar_exe, init_T_filepath, data_folder, res_folder]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    T_res_filepath = os.path.join(res_folder, 'lidar0_to_lidar1.yml')
    return T_res_filepath


def evalExtrinsics(T_ext_list):
    
    assert len(T_ext_list)
    T_baseline = np.linalg.inv(T_ext_list[0])
    # TODO: calc the mean of the extrinsics list
    for T_ext in T_ext_list:
        delta_T = T_baseline.dot(T_ext)
        delta_R = delta_T[:3, :3]
        delta_t = delta_T[:3, 3]
        rot_vec = Rotation.from_dcm(delta_R).as_rotvec()
        rot_vec_norm = np.linalg.norm(rot_vec)
        t_norm = np.linalg.norm(delta_t)
        print('########################################################')
        print('delta rotation angle: {}, delta translation norm: {}'.format(np.rad2deg(rot_vec_norm), t_norm))

    return T_ext_list[0]

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="A helper to caibrate exterinsics between multi-lidar")

    parser.add_argument("ws_folder", type=str, default="",
                        help="the workspace folder")
    parser.add_argument('dataset_folder', type=str, default="",
                        help='the input dataset folder')
    parser.add_argument('output_folder', type=str, default='',
                        help='the folder of output files')
    parser.add_argument('init_T_yaml', type=str, default='',
                        help='the initial Transform between lidar0 and lidar1')
    parser.add_argument('--extension', type=str, default='.ply',
                        help='the extension of lidar point cloud under dataset folder')
    parser.add_argument('--lidar0_idx', type=int, default=0,
                        help='ply prefix under lidar0 folder')
    parser.add_argument('--lidar1_idx', type=int, default=1,
                        help='lidar prefix under lidar1 folder')

    args = parser.parse_args()
    ws_folder = args.ws_folder
    dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    # extension of lidar scan
    scan_extension = args.extension
    # image prefix under cam0 and cam1 folder
    lidar0_idx = args.lidar0_idx
    lidar1_idx = args.lidar1_idx
    init_T_filepath = args.init_T_yaml

    # exe path
    calib_lidar_exe = os.path.join(ws_folder, 'build/test_lidar2lidar_calib')
    mv3dhelper.create_folder_if_not_exists(output_folder)

    time_start = time.time()
    timing_info = []

    ############################# calibrate extrinsics between lidar0 and lidar1 ############################
    # raw data folder path
    raw_data_folder_list = glob.glob(os.path.join(dataset_folder, 'data*'))
    assert(len(raw_data_folder_list))
    # calibration results
    T_ext_list = []
    round_cnt = 0
    for folder in raw_data_folder_list:
        data_folder = os.path.join(folder, 'lidar_lidar')
        res_folder = data_folder
        T_ext_filepath = calibLidarToLidarExt(calib_lidar_exe, data_folder, res_folder, init_T_filepath)
        if not os.path.exists(T_ext_filepath):
            print('Fail to calibrate lidar0 to lidar1 in {}'.format(data_folder))
            exit(-1)
        T_ext = mv3dhelper.readExtFileOpencv(T_ext_filepath)
        T_ext_list.append(T_ext)

        round_cnt += 1
        timing_info.append(('Round {} calibration of lidar0-lidar1'.format(round_cnt), time.time() - time_start))
        time_start = time.time()

    T_avg = evalExtrinsics(T_ext_list)
    avg_res_filepath = os.path.join(output_folder, 'lidar0_to_lidar1.yml')
    mv3dhelper.saveExtFileOpencv(avg_res_filepath, T_avg)

    print('------------------------------')
    for info in timing_info:
        print('| %s: %0.3f second(s)' % (info[0], info[1]))
    print('------------------------------')
