import os
import sys
import glob
import shutil
import argparse
import mv3dhelper
import zrpc
import time


def help_information():
    print('Usage: python run_multicam_intrin_calib.py <workspace_folder> <dataset_folder> <cam_num> <output_folder> ')

# calibrate mono camera using Kalibr
def calib_mono_camera_intrin(calib_exe, ws_folder, raw_data_folder, output_folder, target_type):
    if not os.path.exists(raw_data_folder):
        print(' Empty camera data folder!')
        return False

    cam_type = 'mono'
    cmds = ['python', calib_exe, ws_folder, raw_data_folder, cam_type, output_folder, target_type]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

# evaluate camera intrinsic result 
def eval_mono_camera_intrin(eval_exe, cam_res_list, cam_res_filepath, cam_analy_filepath):
    if len(cam_res_list) < 2:
        print('repeat camera data is not enough!')
        return False
    
    cam_type = 'mono'
    resfile_num = 3
    cmds = [eval_exe, cam_type, resfile_num, cam_res_list[0], cam_res_list[1], cam_res_list[2], cam_analy_filepath, cam_res_filepath]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="A helper to caibrate intrinsic between multi-camera")

    parser.add_argument("ws_folder", type=str, default="",
                        help="the workspace folder")
    parser.add_argument('dataset_folder', type=str, default="",
                        help='the dataset root folder, data specification: https://yuque.antfin-inc.com/aone857851/udonh7/bmazpr')
    parser.add_argument('cam_num', type=str, default='mono',
                        help="the camera number: 1 or 2 or 4")
    parser.add_argument('output_folder', type=str, default='',
                        help='the folder of output files')
    parser.add_argument('--target_type', type=str,
                        default='checkerboard', help='target type')
    parser.add_argument('--extension', type=str, default='.jpg',
                        help='the extension of image under dataset folder')
    parser.add_argument('--show_result', type=bool, default=False,
                        help=' whether show calibration result or not')
    parser.add_argument('--cam0_idx', type=int, default=0,
                        help='image prefix under cam0 folder')
    parser.add_argument('--cam1_idx', type=int, default=1,
                        help='image prefix under cam1 folder')
    parser.add_argument('--cam2_idx', type=int, default=2,
                        help='image prefix under cam2 folder')
    parser.add_argument('--cam3_idx', type=int, default=3,
                        help='image prefix under cam3 folder')

    args = parser.parse_args()
    ws_folder = args.ws_folder
    root_dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    target_type = args.target_type
    # extension of image
    img_extension = args.extension
    b_show_result = args.show_result
    cam_num = args.cam_num
    # image prefix under cam0 and cam1 folder
    cam0_idx = args.cam0_idx
    cam1_idx = args.cam1_idx
    cam2_idx = args.cam2_idx
    cam3_idx = args.cam3_idx

    # target filepath
    if target_type not in ['checkerboard', 'apriltag', 'cctag']:
        print('Invalid target type {}'.format(target_type))
        exit(-1)

    # raw image folder path
    data_folders_list = glob.glob(os.path.join(root_dataset_folder, 'data*'))
    if len(data_folders_list) == 0:
        print('{} doesnot contain any data* folder!'.format(root_dataset_folder))
        exit(-1)

    cam0_datafolder_list = []
    cam1_datafolder_list = []
    cam2_datafolder_list = []
    cam3_datafolder_list = []

    if cam_num == '1':
        for data_folder in data_folders_list:
            cam0_datafolder_list.append(os.path.join(data_folder, 'cam0'))
    elif cam_num == '2':
        for data_folder in data_folders_list:
            cam0_datafolder_list.append(os.path.join(data_folder, 'cam0'))
            cam1_datafolder_list.append(os.path.join(data_folder, 'cam1'))
    elif cam_num == '4':
        for data_folder in data_folders_list:
            cam0_datafolder_list.append(os.path.join(data_folder, 'cam0'))
            cam1_datafolder_list.append(os.path.join(data_folder, 'cam1'))
            cam2_datafolder_list.append(os.path.join(data_folder, 'cam2'))
            cam3_datafolder_list.append(os.path.join(data_folder, 'cam3'))

    mv3dhelper.create_folder_if_not_exists(output_folder)

    # exe path
    intrin_calb_exe = os.path.join(ws_folder, 'scripts/run_cam_intrin_calib.py')
    intrin_eval_exe = os.path.join(ws_folder, 'build/test_eval_kalibr')

    time_start = time.time()
    timing_info = []
    ################################## calibrate mono camera intrinsic #################################
    # camera intrinsic result : mono.yaml
    cam0_res_list = []
    cam1_res_list = []
    cam2_res_list = []
    cam3_res_list = []
    for data_path in cam0_datafolder_list:
        raw_datapath = data_path
        res_path = os.path.join(data_path, 'kalibr')
        calib_mono_camera_intrin(intrin_calb_exe, ws_folder,raw_datapath, res_path, target_type)
        res_filepath = os.path.join(res_path, 'result/mono.yaml')
        cam0_res_list.append(res_filepath)
    for data_path in cam1_datafolder_list:
        raw_datapath = data_path
        res_path = os.path.join(data_path, 'kalibr')
        calib_mono_camera_intrin(intrin_calb_exe, ws_folder,raw_datapath, res_path, target_type)
        res_filepath = os.path.join(res_path, 'result/mono.yaml')
        cam1_res_list.append(res_filepath)        
    for data_path in cam2_datafolder_list:
        raw_datapath = data_path
        res_path = os.path.join(data_path, 'kalibr')
        calib_mono_camera_intrin(intrin_calb_exe, ws_folder,raw_datapath, res_path, target_type)
        res_filepath = os.path.join(res_path, 'result/mono.yaml')
        cam2_res_list.append(res_filepath)
    for data_path in cam3_datafolder_list:
        raw_datapath = data_path
        res_path = os.path.join(data_path, 'kalibr')
        calib_mono_camera_intrin(intrin_calb_exe, ws_folder,raw_datapath, res_path, target_type)
        res_filepath = os.path.join(res_path, 'result/mono.yaml')
        cam3_res_list.append(res_filepath)

    timing_info.append(('Calibrate mono camera intrinsic', time.time() - time_start))
    time_start = time.time()

    ############################ run kalibr result evaluation ###################################
    # test_eval_kalibr <camera_model> <file_num> <kalibr_file1> <kalibr_file2> <kalibr_file3> <analysis_file_name> <avg_result_file_name>
    cam0_res_filepath = os.path.join(output_folder, 'cam0.yml')
    cam0_analy_filepath = os.path.join(output_folder, 'cam0_analysis.txt')
    eval_mono_camera_intrin(intrin_eval_exe, cam0_res_list, cam0_res_filepath, cam0_analy_filepath)

    cam1_res_filepath = os.path.join(output_folder, 'cam1.yml')
    cam1_analy_filepath = os.path.join(output_folder, 'cam1_analysis.txt')
    eval_mono_camera_intrin(intrin_eval_exe, cam1_res_list, cam1_res_filepath, cam1_analy_filepath)

    cam2_res_filepath = os.path.join(output_folder, 'cam2.yml')
    cam2_analy_filepath = os.path.join(output_folder, 'cam2_analysis.txt')
    eval_mono_camera_intrin(intrin_eval_exe, cam2_res_list, cam2_res_filepath, cam2_analy_filepath)

    cam3_res_filepath = os.path.join(output_folder, 'cam3.yml')
    cam3_analy_filepath = os.path.join(output_folder, 'cam3_analysis.txt')
    eval_mono_camera_intrin(intrin_eval_exe, cam3_res_list, cam3_res_filepath, cam3_analy_filepath)
    
    timing_info.append(('Evaluate intrinsic calirbation result', time.time() - time_start))
    time_start = time.time()

    print('------------------------------')
    for info in timing_info:
        print('| %s: %0.3f second(s)' % (info[0], info[1]))
    print('------------------------------')
