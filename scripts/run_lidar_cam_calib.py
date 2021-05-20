import os
import sys
import glob
import shutil
import argparse
import mv3dhelper
import zrpc
import time


def help_information():
    print('Usage: python run_lidar_cam_calib.py <workspace_folder> <dataset_folder> <output_folder> ')


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="A helper to caibrate exterinsics between lidar and camera")

    parser.add_argument("ws_folder", type=str, default="",
                        help="the workspace folder")
    parser.add_argument('dataset_folder', type=str, default="",
                        help='the input dataset folder')
    parser.add_argument('output_folder', type=str, default='',
                        help='the folder of output files')
    parser.add_argument('init_T_yaml', type=str, default='',
                        help='the initial Transform between lidar and camera')
    parser.add_argument('cam_pose_file', type=str, default='',
                        help='the camera pose wrt the panoramic infrastruct')
    parser.add_argument('--extension', type=str, default='.ply',
                        help='the extension of lidar point cloud under dataset folder')
    parser.add_argument('--lidar_idx', type=int, default=1,
                        help='ply prefix under lidar folder')
    parser.add_argument('--cam_idx', type=int, default=0,
                        help='lidar prefix under lidar1 folder')

    args = parser.parse_args()
    ws_folder = args.ws_folder
    dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    # extension of lidar scan
    scan_extension = args.extension
    # image prefix under cam0 and cam1 folder
    lidar_idx = args.lidar_idx
    init_T_filepath = args.init_T_yaml
    cam_pose_filepath = args.cam_pose_file

    if not os.path.exists(init_T_filepath):
        print('File {} dowsnot exist'.format(init_T_filepath))
        exit(-1)
    if not os.path.exists(cam_pose_filepath):
        print('File {} dowsnot exist'.format(cam_pose_filepath))
        exit(-1)

    # dense model of panoramic infrastructure
    target_ply_filepath = os.path.join(ws_folder, 'config/cctag_20210120_EFC.ply')
    if not os.path.exists(cam_pose_filepath):
        print('File {} dowsnot exist'.format(cam_pose_filepath))
        exit(-1)

    mv3dhelper.create_folder_if_not_exists(output_folder)
    # exe path
    calib_lidar_exe = os.path.join(ws_folder, 'build/test_lidar2cam_calibration')

    time_start = time.time()
    timing_info = []
    ############################# calibrate extrinsics between lidar and camera ############################
    # test_lidar2cam_calibration [init_T_l1_cam0.yaml] [dataset_folder] [cam0_pose_file] [target_ply_file] [output_folder] [lidar_id]
    cmds = [calib_lidar_exe, init_T_filepath,
            dataset_folder, cam_pose_filepath, target_ply_filepath, output_folder, lidar_idx]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    timing_info.append(('Calibrate lidar2cam', time.time() - time_start))
    print('------------------------------')
    for info in timing_info:
        print('| %s: %0.3f second(s)' % (info[0], info[1]))
    print('------------------------------')
