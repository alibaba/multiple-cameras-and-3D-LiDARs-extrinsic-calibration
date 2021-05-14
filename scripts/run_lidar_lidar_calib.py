import os
import sys
import glob
import shutil
import argparse
import mv3dhelper
import zrpc
import time


def help_information():
    print('Usage: python run_lidar_lidar_calib.py <workspace_folder> <dataset_folder> <output_folder> ')


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="A helper to caibrate exterinsics between multi-lidar")

    parser.add_argument("ws_folder", type=str, default="",
                        help="the workspace folder")
    parser.add_argument('dataset_folder', type=str, default="",
                        help='the input dataset folder. e.g.https://yuque.antfin-inc.com/aone857851/udonh7/bmazpr')
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

    if not os.path.exists(init_T_filepath):
        print('File {} dowsnot exist'.format(init_T_filepath))
        exit(-1)

    # raw data folder path
    raw_data_folder_list = glob.glob(os.path.join(dataset_folder, 'data*'))
    assert(len(raw_data_folder_list))
    for data in raw_data_folder_list:
        raw_lidar_folder = os.path.join(data, 'lidar_lidar')
        lidar0_scan_folder = os.path.join(raw_lidar_folder, 'lidar0')
        lidar1_scan_folder = os.path.join(raw_lidar_folder, 'lidar1')
        if mv3dhelper.check_folder_empty(lidar0_scan_folder) or mv3dhelper.check_folder_empty(lidar1_scan_folder):
            print('lidar scan folder empty!')
            exit(-1)

    mv3dhelper.create_folder_if_not_exists(output_folder)

    # exe path
    calib_lidar_exe = os.path.join(ws_folder, 'build/test_lidar2lidar_calib')

    time_start = time.time()
    timing_info = []
    ############################# calibrate extrinsics between lidar0 and lidar1 ############################
    # test_lidar2lidar_calibration [T_l0_l1_init.yaml] [input_dataset_folder] [output_folder]
    cmds = [calib_lidar_exe, init_T_filepath, dataset_folder, output_folder]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    timing_info.append(('Calibrate multicams', time.time() - time_start))
    print('------------------------------')
    for info in timing_info:
        print('| %s: %0.3f second(s)' % (info[0], info[1]))
    print('------------------------------')
