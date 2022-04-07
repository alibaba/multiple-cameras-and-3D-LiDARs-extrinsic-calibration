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
    print('''Usage: python run_cam_imu_calib.py <workspace_folder> <dataset_folder> <output_folder> <imu_yaml_file> <cam_chain_yaml_files>'''
        ''' <workspace_folder>: /code/kalibr_calibration. '''
        '''<dataset_folder>:  e.g.https://yuque.antfin-inc.com/aone857851/udonh7/bmazpr''')


def copyAndRenameCamData(raw_cam_folder, img_list, out_cam_folder):

    raw_cam_tms_filepath = glob.glob(os.path.join(raw_cam_folder, 'timestamp*.txt'))[0]
    if not os.path.exists(raw_cam_tms_filepath):
        print('{} doesnot exist!'.format(raw_cam_tms_filepath))
        exit(-1)

    # the raw image tms unit is s, then we cvt it into ns
    img_tms = mv3dhelper.readImageTms(raw_cam_tms_filepath)

    # copy images to out_cam_folder
    delim_pattern = r'\_|\.'
    for img_name in img_list:
        src_img_path = os.path.join(raw_cam_folder, img_name)
        img_idx = re.split(delim_pattern, img_name)[1]
        img_extension = re.split(delim_pattern, img_name)[2]
        dst_img_path = os.path.join(out_cam_folder, img_tms[img_idx] + '.' + img_extension)

        shutil.copyfile(src_img_path, dst_img_path)

def copyAndRenameImuData(raw_imu_file, out_imu_file):
    # the raw imu tms unit is us
    imu_data = mv3dhelper.readImuData(raw_imu_file)

    # reorder imu data
    new_imu_datas = []
    for data in imu_data:
        # timestamp: ms
        imu_timestamp = int(data[0])
        # if imu_timestamp >= img_start_timestamps:
        # the raw imu tms unit is us, then we cvt it into ns
        new_imu_tuple = (int(imu_timestamp*1000), data[1], data[2],
                         data[3], data[4], data[5], data[6])
        new_imu_datas.append(new_imu_tuple)

    # write imu.csv
    imu_headers = ["timestamp", "omega_x", "omega_y", "omega_z", "alpha_x", "alpha_y", "alpha_z"]
    with open(out_imu_file, 'w') as f:
        f_csv = csv.writer(f)
        f_csv.writerow(imu_headers)
        f_csv.writerows(new_imu_datas)

# calibrate extrinsic between camera and imu, return camera_to_imu.yaml
def calibCamImuExt(data_folder, output_folder, target_filepath, imu_yaml_file, cam_chain_yaml_file, img_extension, b_show_extract, cam_type, cam0_idx, cam1_idx):
    
    raw_imu_file = os.path.join(data_folder, 'imu_0.txt')
    raw_cam0_folder = os.path.join(data_folder, 'cam'+str(cam0_idx))
    if cam_type == 'stereo':
        raw_cam0_folder = os.path.join(data_folder, 'stereo/cam'+str(cam0_idx))
        raw_cam1_folder = os.path.join(data_folder, 'stereo/cam'+str(cam1_idx))

    if not os.path.exists(raw_imu_file):
        print('{} doesnot exist!'.format(raw_imu_file))
        exit(-1)
    if not os.path.exists(raw_cam0_folder):
        print('{} doesnot exist!'.format(raw_cam0_folder))
        exit(-1)

    mv3dhelper.create_folder_if_not_exists(output_folder)
    # kalibr exe
    kalibr_bag_exe = 'kalibr_bagcreater'
    kalibr_cam_imu_exe = 'kalibr_calibrate_imu_camera'
    # sort image
    cam0_img_prefix = str(cam0_idx)+'_'
    cam0_img_list = mv3dhelper.sort_rename_images(raw_cam0_folder, cam0_img_prefix, 'image_', img_extension)
    if cam_type == 'stereo':
        cam1_img_list = mv3dhelper.sort_rename_images(raw_cam1_folder, cam0_img_prefix, 'image_', img_extension)

    # copy and rename images according to timestamp
    out_cam0_folder = os.path.join(output_folder, 'data/cam0')
    out_cam1_folder = os.path.join(output_folder, 'data/cam1')
    out_imu_file = os.path.join(output_folder, 'data/imu0.csv')
    res_folder = os.path.join(output_folder, 'result')
    out_bag_filepath = os.path.join(res_folder, 'output.bag')

    if os.path.exists(out_cam0_folder):
        shutil.rmtree(out_cam0_folder)
    if os.path.exists(out_cam1_folder):
        shutil.rmtree(out_cam1_folder)
    if os.path.exists(out_imu_file):
        os.remove(out_imu_file)
    if os.path.exists(res_folder):
        shutil.rmtree(res_folder)
    os.makedirs(out_cam0_folder)
    if cam_type == 'stereo':
        os.makedirs(out_cam1_folder)

    copyAndRenameCamData(raw_cam0_folder, cam0_img_list, out_cam0_folder)
    if cam_type == 'stereo':
        copyAndRenameCamData(raw_cam1_folder, cam1_img_list, out_cam1_folder)
    copyAndRenameImuData(raw_imu_file, out_imu_file)

    # create rosbag
    mv3dhelper.create_folder_if_not_exists(res_folder)
    ros_data_folder = os.path.join(output_folder, 'data')
    # kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}
    cmds = [kalibr_bag_exe, '--folder', ros_data_folder, '--output-bag', out_bag_filepath]
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    # calibrate imu_camera
    os.chdir(res_folder)
    # kalibr_calibrate_imu_camera --bag ${ros_bag_file} --target ${target_file} --cams ${cam_chain_yaml} --imu ${imu_yaml} --dont-show-report --recompute-camera-chain-extrinsics --show-extraction
    cmds = [kalibr_cam_imu_exe, '--bag', out_bag_filepath, '--target',
            target_filepath, '--cams', cam_chain_yaml_file, '--imu', imu_yaml_file, '--dont-show-report']
    if b_show_extract:
        cmds.append('--show-extraction')
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    # rename result files
    res_report_filepath = glob.glob(os.path.join(res_folder, '*.pdf'))[0]
    dst_report_filepath = os.path.join(res_folder, 'camera' + str(cam0_idx) + '_to_imu.pdf')
    res_yaml_filepath = glob.glob(os.path.join(res_folder, '*.yaml'))[0]
    dst_yaml_filepath = os.path.join(res_folder, 'camera' + str(cam0_idx) + '_to_imu.yaml')
    res_txt_filepath = glob.glob(os.path.join(res_folder, '*.txt'))[0]
    dst_txt_filepath = os.path.join(res_folder, 'camera' + str(cam0_idx) + '_to_imu.txt')

    os.rename(res_report_filepath, dst_report_filepath)
    os.rename(res_yaml_filepath, dst_yaml_filepath)
    os.rename(res_txt_filepath, dst_txt_filepath)

    return dst_yaml_filepath

# read extrinsics from kalibr yaml file
def readCamImuExtFileKalir(filename):
    T_ext = np.eye(4)
    with open(filename, 'r') as fr:
        root_node = yaml.load(fr)
        ext_node = root_node['cam0']['T_cam_imu']
        T_ext[0, :4] = np.array(ext_node[0])
        T_ext[1, :4] = np.array(ext_node[1])
        T_ext[2, :4] = np.array(ext_node[2])
        T_ext[3, :4] = np.array(ext_node[3])
    return T_ext

def evalExtrinsics(T_cam_imu_list):
    
    assert len(T_cam_imu_list)
    T_baseline = np.linalg.inv(T_cam_imu_list[0])
    # TODO: calc the mean of the extrinsics list
    for T_ext in T_cam_imu_list:
        delta_T = T_baseline.dot(T_ext)
        delta_R = delta_T[:3, :3]
        delta_t = delta_T[:3, 3]
        rot_vec = Rotation.from_dcm(delta_R).as_rotvec()
        rot_vec_norm = np.linalg.norm(rot_vec)
        t_norm = np.linalg.norm(delta_t)
        print('########################################################')
        print('delta rotation angle: {}, delta translation norm: {}'.format(np.rad2deg(rot_vec_norm), t_norm))

    return T_cam_imu_list[0]


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="A helper to caibrate (mono)cam-imu in Kalibr")

    parser.add_argument("ws_folder", type=str, default="",
                        help="the workspace folder")
    parser.add_argument('dataset_folder', type=str, default="",
                        help='the input dataset folder')
    parser.add_argument('output_folder', type=str, default='',
                        help='the folder of output files')
    parser.add_argument('--cam_type', type=str, default='mono',
                        help="the camera type: mono or stereo")
    parser.add_argument('imu_yaml_file', type=str,
                        default='', help='the imu intrinsic file')
    parser.add_argument('cam_chain_yaml_file', type=str,
                        default='', help='the camera chain file')
    parser.add_argument('--target_type', type=str,
                        default='apriltag', help='target type')
    parser.add_argument('--extension', type=str, default='.jpg',
                        help='the extension of image under dataset folder')
    parser.add_argument('--show_extraction', type=bool, default=False,
                        help=' whether show feature extraction or not')
    parser.add_argument('--cam0_idx', type=int, default=0,
                        help='image prefix under cam0 folder')
    parser.add_argument('--cam1_idx', type=int, default=1,
                        help='image prefix under cam1 folder')

    args = parser.parse_args()
    ws_folder = args.ws_folder
    dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    imu_yaml_file = args.imu_yaml_file
    cam_chain_yaml_file = args.cam_chain_yaml_file
    if not os.path.exists(imu_yaml_file):
        print('{} doesnot exist!'.format(imu_yaml_file))
        exit(-1)
    if not os.path.exists(cam_chain_yaml_file):
        print('{} doesnot exist!'.format(cam_chain_yaml_file))
        exit(-1)

    if args.cam_type == 'mono':
        cam_type = 'mono'
    elif args.cam_type == 'stereo':
        cam_type = 'stereo'
    else:
        print('Invalid camera type {}'.format(args.cam_type))
        exit(-1)

    # extension of image
    img_extension = args.extension
    b_show_extract = args.show_extraction
    # image prefix under cam0 folder
    cam0_idx = args.cam0_idx
    cam1_idx = args.cam1_idx

    # target filepath
    if args.target_type == 'checkerboard':
        target_filepath = os.path.join(ws_folder, 'config/checkerboard_8x11_3x3cm.yaml')
    elif args.target_type == 'apriltag':
        target_filepath = os.path.join(ws_folder, 'config/april_6x6_80x80cm.yaml')
    else:
        print('Invalid target type {}'.format(args.target_type))
        exit(-1)

    time_start = time.time()
    timing_info = []
    
    ############################# calibrate extrinsics between camaera and imu ############################
    # raw data folder path
    raw_data_folder_list = glob.glob(os.path.join(dataset_folder, 'data*'))
    assert(len(raw_data_folder_list))
    # calibration results
    T_cam_imu_list = []
    round_cnt = 0
    for folder in raw_data_folder_list:
        data_folder = os.path.join(folder, 'cam_imu')
        res_folder = os.path.join(data_folder, 'kalibr')
        T_cam_imu_file = calibCamImuExt(data_folder, res_folder, target_filepath, imu_yaml_file, cam_chain_yaml_file, img_extension, b_show_extract, cam_type, cam0_idx, cam1_idx)
        T_ext = readCamImuExtFileKalir(T_cam_imu_file)
        T_cam_imu_list.append(T_ext)
        
        round_cnt += 1
        timing_info.append(('Round {} calibration of cam-imu'.format(round_cnt), time.time() - time_start))
        time_start = time.time()


    T_avg = evalExtrinsics(T_cam_imu_list)
    avg_res_filepath = os.path.join(output_folder, 'camera'+ str(cam0_idx)+'_to_imu.yaml')
    mv3dhelper.saveExtFileOpencv(avg_res_filepath, T_avg)

    print('------------------------------')
    for info in timing_info:
        print('| %s: %0.3f second(s)' % (info[0], info[1]))
    print('------------------------------')