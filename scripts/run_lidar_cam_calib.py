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
    print('''Usage: python run_lidar_cam_calib.py <workspace_folder> <dataset_folder> <output_folder> '''
        ''' <workspace_folder>: /code/kalibr_calibration. '''
        '''<dataset_folder>:  e.g.https://yuque.antfin-inc.com/aone857851/udonh7/bmazpr'''
        ''' This exe must be run after run_multicam_extrin_calib.py !!!!''')


def locateCamera(raw_cam0_folder, output_folder, img_undist_exe, cctag_detect_exe, cam_calib_exe, cam0_intrin_file):
    time_start = time.time()

    ################################## undistort image #################################
    undist_cam0_folder = os.path.join(output_folder, 'undist_cam0')
    if os.path.exists(undist_cam0_folder):
        shutil.rmtree(undist_cam0_folder)

    focal_scale = 1.0
    cmds = [img_undist_exe, raw_cam0_folder, cam0_intrin_file, undist_cam0_folder, focal_scale]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    timing_info.append(('Generate undistorted images', time.time() - time_start))
    time_start = time.time()

    ############################ run cctag detection #########################
    cctag_sample_exe = '/code/CCTag/build/src/applications/detection'
    # cctag_sample_exe = '/usr/local/bin/detection'
    # cctag_sample_exe = '/home/ziqianbai/Projects/vlab/CCTag/build/src/applications/detection'
    # test_detect_cctag <input_img_folder> <result_filepath> <detection_exe_path>
    detect_result_file = os.path.join(output_folder, 'cam0_cctag_result.yaml')
    cmds = [cctag_detect_exe, undist_cam0_folder,
            detect_result_file, cctag_sample_exe]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)
    
    timing_info.append(('Detect cctag on images', time.time() - time_start))
    time_start = time.time()

    ########################### locate camera0 w.r.t calibration pattern ######################
    time_start = time.time()
    # undistorted intrinsic file of each camera
    undist_cam0_intrin_file = glob.glob(os.path.join(undist_cam0_folder, '*.y*ml'))[0]

    # test_mono_calibration [global_map.yaml] [cctag_result.yaml][cam0_intrin_file] [output_folder]
    cmds = [cam_calib_exe, target_sparse_filepath, detect_result_file, undist_cam0_intrin_file, output_folder]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    timing_info.append(('Locate camera0 w.r.t panoramic infrastructure', time.time() - time_start))
    time_start = time.time()

    cam0_loc_result_filepath = os.path.join(output_folder, 'cam0_w2c.txt')
    if not os.path.exists(cam0_loc_result_filepath):
        print('Fail to locate camera0 !!!!')
        return None
    
    return cam0_loc_result_filepath

def calibLidarToCamExt(calib_lidar_exe, data_folder, res_folder, init_T_filepath, cam_pose_filepath, target_ply_filepath, lidar_idx):
    if not os.path.exists(cam_pose_filepath):
        print('File {} dowsnot exist'.format(cam_pose_filepath))
        exit(-1)

    # test_lidar2cam_calibration [init_T_l1_cam0.yaml] [dataset_folder] [cam0_pose_file] [target_ply_file] [output_folder] [lidar_id]
    cmds = [calib_lidar_exe, init_T_filepath,
            data_folder, cam_pose_filepath, target_ply_filepath, res_folder, lidar_idx]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    T_ext_filepath = os.path.join(res_folder, 'lidar1_to_camera0.yml')
    return T_ext_filepath


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
        description="A helper to caibrate exterinsics between lidar1 and camera0")

    parser.add_argument("ws_folder", type=str, default="",
                        help="the workspace folder")
    parser.add_argument('dataset_folder', type=str, default="",
                        help='the input dataset folder')
    parser.add_argument('output_folder', type=str, default='',
                        help='the folder of output files')
    parser.add_argument('init_T_yaml', type=str, default='',
                        help='the initial Transform between lidar1 and camera0')
    # parser.add_argument('cam_pose_file', type=str, default='',
    #                     help='the camera pose wrt the panoramic infrastruct')
    parser.add_argument('--extension', type=str, default='.ply',
                        help='the extension of lidar point cloud under dataset folder')
    parser.add_argument('--lidar_idx', type=int, default=1,
                        help='ply prefix under lidar folder')
    parser.add_argument('--cam_idx', type=int, default=0,
                        help='lidar prefix under cams folder')
    parser.add_argument('--cam0_intrin_file', type=str, default='/data/cam0.yml',
                        help='camera0 intrinsic file')

    args = parser.parse_args()
    ws_folder = args.ws_folder
    dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    # extension of lidar scan
    scan_extension = args.extension
    lidar_idx = args.lidar_idx
    cam_idx = args.cam_idx
    init_T_filepath = args.init_T_yaml
    # cam_pose_filepath = args.cam_pose_file
    cam0_intrin_file = args.cam0_intrin_file

    mv3dhelper.create_folder_if_not_exists(output_folder)

    # sparse model of panoramic infrastructure
    target_sparse_filepath = os.path.join(ws_folder, 'config/cctag_20210120_EFC.yaml')
    # dense model of panoramic infrastructure
    target_ply_filepath = os.path.join(ws_folder, 'config/cctag_20210120_EFC.ply')
    # exe path
    calib_lidar_exe = os.path.join(ws_folder, 'build/test_lidar2cam_calibration')
    img_undist_exe = os.path.join(ws_folder, 'build/test_undist_img')
    cctag_detect_exe = os.path.join(ws_folder, 'build/test_detect_cctag')
    cam_calib_exe = os.path.join(ws_folder, 'build/test_mono_calibration')

    time_start = time.time()
    timing_info = []

    ############################# calibrate extrinsics between lidar1 and camera0 ############################
    # raw data folder path
    raw_data_folder_list = glob.glob(os.path.join(dataset_folder, 'data*'))
    assert(len(raw_data_folder_list))
    # calibration results
    T_ext_list = []
    round_cnt = 0
    for folder in raw_data_folder_list:
        data_folder = os.path.join(folder, 'lidar_cam')
        res_folder = data_folder

        # locate camera0 w.r.t the calibration pattrern
        cam_folder = os.path.join(data_folder, 'cam0/cam0')
        cam_pose_filepath = locateCamera(cam_folder, res_folder, img_undist_exe, cctag_detect_exe, cam_calib_exe, cam0_intrin_file)
        # cam_pose_filepath = os.path.join(folder, 'cams/multicam/cam'+ str(cam_idx) +'_w2c.txt')
        T_ext_filepath = calibLidarToCamExt(calib_lidar_exe, data_folder, res_folder, init_T_filepath, cam_pose_filepath, target_ply_filepath, lidar_idx)
        if not os.path.exists(T_ext_filepath):
            print('Fail to calibrate lidar1 to camera0 in {}'.format(data_folder))
            exit(-1)
        T_ext = mv3dhelper.readExtFileOpencv(T_ext_filepath)
        T_ext_list.append(T_ext)

        round_cnt += 1
        timing_info.append(('Round {} calibration of lidar1-camera0'.format(round_cnt), time.time() - time_start))
        time_start = time.time()


    T_avg = evalExtrinsics(T_ext_list)
    avg_res_filepath = os.path.join(output_folder, 'lidar1_to_camera0.yml')
    mv3dhelper.saveExtFileOpencv(avg_res_filepath, T_avg)

    print('------------------------------')
    for info in timing_info:
        print('| %s: %0.3f second(s)' % (info[0], info[1]))
    print('------------------------------')
