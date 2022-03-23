import os
import sys
import glob
import shutil
import argparse
import mv3dhelper
import zrpc
import time


def help_information():
    print('Usage: python run_multicam_extrin_calib.py <workspace_folder> <dataset_folder> <cam_num> <output_folder> ')


def merge_undist_cam_folders(undist_cam_folder_list, merged_undist_cams_folder):
    if len(undist_cam_folder_list) == 0:
        print(' Empty undistorted camera folder!')
        return False

    mv3dhelper.create_folder_if_not_exists(merged_undist_cams_folder)

    # sort list by folder name
    undist_cam_folder_list.sort(key=lambda x: x[-1])
    for cam_idx in range(len(undist_cam_folder_list)):
        folder = undist_cam_folder_list[cam_idx]
        img_list = [img for img in os.listdir(folder) if img.endswith(".jpg")]
        # only copy 1 image
        src_img_path = os.path.join(folder, img_list[0])
        dst_img_path = os.path.join(
            merged_undist_cams_folder, 'spot0_'+str(cam_idx)+'.jpg')
        shutil.copy(src_img_path, dst_img_path)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="A helper to caibrate exterinsics between multi-camera")

    parser.add_argument("ws_folder", type=str, default="",
                        help="the workspace folder")
    parser.add_argument('dataset_folder', type=str, default="",
                        help='the input dataset folder')
    parser.add_argument('cam_num', type=str, default='mono',
                        help="the camera number: [1, 2, 4, 5]")
    parser.add_argument('output_folder', type=str, default='',
                        help='the folder of output files')
    parser.add_argument('--target_type', type=str,
                        default='cctag', help='target type')
    parser.add_argument('--extension', type=str, default='.jpg',
                        help='the extension of image under dataset folder')
    parser.add_argument('--show_result', type=bool, default=False,
                        help=' whether show calibration result or not')
    parser.add_argument('--cam0_idx', type=int, default=0,
                        help='image prefix under cam0 folder')
    parser.add_argument('--cam1_idx', type=int, default=1,
                        help='image prefix under cam1 folder if the database folder is aim to stereo calibration')
    parser.add_argument('--cam0_intrin_file', type=str, default='/data/cam0.yml',
                        help='camera0 intrinsic file')
    parser.add_argument('--cam1_intrin_file', type=str, default='/data/cam1.yml',
                        help='camera1 intrinsic file')
    parser.add_argument('--cam2_intrin_file', type=str, default='/data/cam2.yml',
                        help='camera2 intrinsic file')
    parser.add_argument('--cam3_intrin_file', type=str, default='/data/cam3.yml',
                        help='camera3 intrinsic file')
    parser.add_argument('--cam4_intrin_file', type=str, default='/data/cam4.yml',
                        help='camera4 intrinsic file')

    args = parser.parse_args()
    ws_folder = args.ws_folder
    dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    # extension of image
    img_extension = args.extension
    b_show_result = args.show_result
    cam_num = args.cam_num
    # image prefix under cam0 and cam1 folder
    cam0_idx = args.cam0_idx
    cam1_idx = args.cam1_idx
    cam_intrin_filelist = [args.cam0_intrin_file, args.cam1_intrin_file,
                           args.cam2_intrin_file, args.cam3_intrin_file, args.cam4_intrin_file]

    # target filepath
    if args.target_type == 'checkerboard':
        target_filepath = os.path.join(ws_folder, 'config/checkerboard_8x11_3x3cm.yaml')
    elif args.target_type == 'apriltag':
        target_filepath = os.path.join(ws_folder, 'config/april_6x6_80x80cm.yaml')
    elif args.target_type == 'cctag':
        target_filepath = os.path.join(ws_folder, 'config/cctag_20210120_EFC.yaml')
    else:
        print('Invalid target type {}'.format(args.target_type))
        exit(-1)

    # raw image folder path
    raw_cam_folder_list = [os.path.join(dataset_folder, 'cam0'), os.path.join(
        dataset_folder, 'cam1'), os.path.join(dataset_folder, 'cam2'), os.path.join(dataset_folder, 'cam3'), 
        os.path.join(dataset_folder, 'cam4')]

    mv3dhelper.create_folder_if_not_exists(output_folder)

    # exe path
    img_undist_exe = os.path.join(ws_folder, 'build/test_undist_img')
    cctag_detect_exe = os.path.join(ws_folder, 'build/test_detect_cctag')
    if args.cam_num == '1':
        cam_calib_exe = os.path.join(ws_folder, 'build/test_mono_calibration')
    elif args.cam_num == '2':
        cam_calib_exe = os.path.join(ws_folder, 'build/test_stereo_calibration')
    elif args.cam_num == '4':
        cam_calib_exe = os.path.join(ws_folder, 'build/test_teche_calibration')
    elif args.cam_num == '5':
        cam_calib_exe = os.path.join(ws_folder, 'build/test_ladybug_calibration')
    else:
        print('Invalid camera number {}'.format(args.cam_num))
        exit(-1)

    time_start = time.time()
    timing_info = []
    ################################## undistort image #################################
    undist_cam_folder_list = [os.path.join(output_folder, 'undist_cam0'), os.path.join(
        output_folder, 'undist_cam1'), os.path.join(output_folder, 'undist_cam2'), os.path.join(output_folder, 'undist_cam3'),
        os.path.join(output_folder, 'undist_cam4')]
    for f in undist_cam_folder_list:
        if os.path.exists(f):
            shutil.rmtree(f)

    focal_scale = 1.0
    for idx in range(0, int(cam_num)):
        raw_cam_folder = raw_cam_folder_list[idx]
        undist_cam_folder = undist_cam_folder_list[idx]
        cam_int_file = cam_intrin_filelist[idx]
        cmds = [img_undist_exe, raw_cam_folder,
                cam_int_file, undist_cam_folder, focal_scale]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)

    timing_info.append(('Generate undistorted images', time.time() - time_start))
    time_start = time.time()

    ############################ run cctag detection #########################
    # cctag_sample_exe = '/code/CCTag/build/src/applications/detection'
    # cctag_sample_exe = '/usr/local/bin/detection'
    cctag_sample_exe = '/home/ziqianbai/Projects/vlab/CCTag/build/src/applications/detection'
    # test_detect_cctag <input_img_folder> <result_filepath> <detection_exe_path>
    if cam_num == '1':
        detect_result_file = os.path.join(output_folder, 'cam'+str(cam0_idx)+'_cctag_result.yaml')
        cmds = [cctag_detect_exe, undist_cam_folder_list[cam0_idx],
                detect_result_file, cctag_sample_exe]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)
    elif cam_num == '2':
        cam0_result_file = os.path.join(output_folder, 'cam'+str(cam0_idx)+'_cctag_result.yaml')
        cmds = [cctag_detect_exe, undist_cam_folder_list[cam0_idx],
                cam0_result_file, cctag_sample_exe]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)
        cam1_result_file = os.path.join(output_folder, 'cam'+str(cam1_idx)+'_cctag_result.yaml')
        cmds = [cctag_detect_exe, undist_cam_folder_list[cam1_idx],
                cam1_result_file, cctag_sample_exe]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)
    elif cam_num == '4' or cam_num == '5':
        merged_cams_folder = os.path.join(output_folder, 'undist_cams')
        merge_undist_cam_folders(undist_cam_folder_list[:int(cam_num)], merged_cams_folder)

        detect_result_file = os.path.join(output_folder, 'cams_cctag_result.yaml')
        cmds = [cctag_detect_exe, merged_cams_folder,
                detect_result_file, cctag_sample_exe]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)
    
    timing_info.append(('Detect cctag on images', time.time() - time_start))
    time_start = time.time()

    ########################### calibrate extrinsics between multi-cameras ######################
    # undistorted intrinsic file of each camera
    undist_cam_intrin_filelist = []
    for idx in range(0, int(cam_num)):
        if os.path.exists(undist_cam_folder_list[idx]):
            undist_cam_intrin = glob.glob(os.path.join(undist_cam_folder_list[idx], '*.y*ml'))[0]
            undist_cam_intrin_filelist.append(undist_cam_intrin)

    if cam_num == '1':
        # test_mono_calibration [global_map.yaml] [cctag_result.yaml][cam0_intrin_file] [output_folder]
        cmds = [cam_calib_exe, target_filepath, detect_result_file, undist_cam_intrin_filelist[cam0_idx], output_folder]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)
    elif cam_num == '2':
        # test_stereo_calibration [global_map.yaml] [cam0_cctag_result.yaml] [cam1_cctag_result.yaml] [cam0_intrin_file] [cam1_intrin_file] [flag_calibrate_hik_teche] [output_folder]
        calib_hetercam_flag = 0
        cmds = [cam_calib_exe, target_filepath, cam0_result_file, cam1_result_file, undist_cam_intrin_filelist[cam0_idx], undist_cam_intrin_filelist[cam1_idx], calib_hetercam_flag, output_folder]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)
    elif cam_num == '4':
        # test_techecalibrator [global_map.yaml] [cctag_result.yaml][cam0_intrin_file] [cam1_intrin_file] [cam2_intrin_file] [cam3_intrin_file][output_folder]
        cmds = [cam_calib_exe, target_filepath, detect_result_file, undist_cam_intrin_filelist[0], 
        undist_cam_intrin_filelist[1], undist_cam_intrin_filelist[2], undist_cam_intrin_filelist[3], output_folder]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)
    elif cam_num == '5':
        # test_ladybugcalibrator [global_map.yaml] [cctag_result.yaml] [cam0_intrin_file] [cam1_intrin_file] [cam2_intrin_file] [cam3_intrin_file] [cam4_intrin_file] [output_folder]
        cmds = [cam_calib_exe, target_filepath, detect_result_file, undist_cam_intrin_filelist[0], 
        undist_cam_intrin_filelist[1], undist_cam_intrin_filelist[2], undist_cam_intrin_filelist[3], undist_cam_intrin_filelist[4], output_folder]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)

    timing_info.append(('Calibrate multicams', time.time() - time_start))
    print('------------------------------')
    for info in timing_info:
        print('| %s: %0.3f second(s)' % (info[0], info[1]))
    print('------------------------------')
