import os
import sys
import glob
import shutil
import argparse
import mv3dhelper
import zrpc
import time


def help_information():
    print('Usage: python run_cam_intrin_calib.py <workspace_folder> <dataset_folder> <cam_type> <output_folder> ')


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="A helper to caibrate camera intrinsic in Kalibr")

    parser.add_argument("ws_folder", type=str, default="",
                        help="the workspace folder")
    parser.add_argument('dataset_folder', type=str, default="",
                        help='the input dataset folder')
    parser.add_argument('cam_type', type=str, default='mono',
                        help="the camera type: mono or stereo")
    parser.add_argument('output_folder', type=str, default='',
                        help='the folder of output files')
    parser.add_argument('--target_type', type=str,
                        default='checkerboard', help='target type')
    parser.add_argument('--extension', type=str, default='.jpg',
                        help='the extension of image under dataset folder')
    parser.add_argument('--dist_model', type=str, default='pinhole-equi',
                        help='the distortion model of lens: pinhole-radtan, pinhole-equi, omni-radtan')
    parser.add_argument('--show_extraction', type=bool, default=False,
                        help=' whether show feature extraction or not')
    parser.add_argument('--cam0_idx', type=int, default=0, help='image prefix under cam0 folder')
    parser.add_argument('--cam1_idx', type=int, default=1, help='image prefix under cam1 folder if the database folder is aim to stereo calibration')

    args = parser.parse_args()
    ws_folder = args.ws_folder
    dataset_folder = args.dataset_folder
    output_folder = args.output_folder
    # extension of image
    img_extension = args.extension
    b_show_extract = args.show_extraction
    # image prefix under cam0 and cam1 folder
    cam0_idx = args.cam0_idx
    cam1_idx = args.cam1_idx

    if args.cam_type == 'mono':
        cam_type = 'mono'
    elif args.cam_type == 'stereo':
        cam_type = 'stereo'
    else:
        print('Invalid camera type {}'.format(args.cam_type))
        exit(-1)

    # target filepath
    if args.target_type == 'checkerboard':
        target_filepath = os.path.join(ws_folder, 'config/checkerboard_8x11_3x3cm.yaml')
    elif args.target_type == 'apriltag':
        target_filepath = os.path.join(ws_folder, 'config/april_6x6_80x80cm.yaml')
    else:
        print('Invalid target type {}'.format(args.target_type))
        exit(-1)

    # camera distortion model
    cam_model = args.dist_model

    # raw image folder path
    raw_cam0_folder = os.path.join(dataset_folder, 'cam0')
    if not os.path.exists(raw_cam0_folder):
        print('{} doesnot exist!'.format(raw_cam0_folder))
        exit(-1)
    if cam_type == 'stereo':
        raw_cam1_folder = os.path.join(dataset_folder, 'cam1')
        if not os.path.exists(raw_cam1_folder):
            print('{} doesnot exist!'.format(raw_cam1_folder))
            exit(-1)

    mv3dhelper.create_folder_if_not_exists(output_folder)

    # exe path
    kalibr_bag_exe = 'kalibr_bagcreater'
    kalibr_calib_exe = 'kalibr_calibrate_cameras'

    # sort image
    cam0_img_prefix = str(cam0_idx)+'_'
    cam0_img_list = mv3dhelper.sort_rename_images(raw_cam0_folder, cam0_img_prefix, 'image_', img_extension)
    if cam_type == 'stereo':
        if os.path.exists(raw_cam1_folder):
            cam1_img_prefix = str(cam1_idx) + '_'
            cam1_img_list = mv3dhelper.sort_rename_images(raw_cam1_folder, cam1_img_prefix, 'image_', img_extension)

    # rename images according to timestamp
    out_cam0_folder = os.path.join(output_folder, 'data/cam0')
    out_cam1_folder = os.path.join(output_folder, 'data/cam1')
    res_folder = os.path.join(output_folder, 'result')
    # rosbag filepath
    out_bag_filepath = os.path.join(res_folder, 'output.bag')

    if os.path.exists(out_cam0_folder):
        shutil.rmtree(out_cam0_folder)
    os.makedirs(out_cam0_folder)
    if os.path.exists(out_cam1_folder):
        shutil.rmtree(out_cam1_folder)
    if cam_type == "stereo":
        os.makedirs(out_cam1_folder)

    for img_idx in range(0, len(cam0_img_list)):
        src_img0_path = os.path.join(raw_cam0_folder, cam0_img_list[img_idx])
        ts = int(time.time() * 1e9)
        dst_img0_path = os.path.join(out_cam0_folder, str(ts) + img_extension)
        shutil.copyfile(src_img0_path, dst_img0_path)
        print(dst_img0_path)

        if cam_type == "stereo":
            src_img1_path = os.path.join(raw_cam1_folder, cam1_img_list[img_idx])
            dst_img1_path = os.path.join(out_cam1_folder, str(ts) + img_extension)
            shutil.copyfile(src_img1_path, dst_img1_path)
        time.sleep(0.03)

    # calibrate camera intrinsic and extrinsic with kalibr toolkits
    # create rosbag file
    if not os.path.exists(out_bag_filepath):
        mv3dhelper.create_folder_if_not_exists(res_folder)
        ros_data_folder = os.path.join(output_folder, 'data')
        # kalibr_bagcreater --folder ${data_path} --output-bag ${bag_file}
        cmds = [kalibr_bag_exe, '--folder', ros_data_folder,
                '--output-bag', out_bag_filepath]
        print(cmds)
        if zrpc.map([cmds])[1] == 0:
            exit(-1)

    # run kalibr_calibrate_cameras
    os.chdir(res_folder)
    if cam_type == 'mono':
        # kalibr_calibrate_cameras --target ${target_file} --dont-show-report  --bag ${bag_file} --models ${cam_model} --topics /cam0/image_raw #--show-extraction
        cmds = [kalibr_calib_exe, '--target', target_filepath, '--dont-show-report', '--models', cam_model,
                '--bag', out_bag_filepath, '--topics', '/cam0/image_raw']
    if cam_type == 'stereo':
        # kalibr_calibrate_cameras --target ${target_file} --dont-show-report --bag ${bag_file} --models ${cam_model} ${cam_model} --topics /cam0/image_raw /cam1/image_raw
        cmds = [kalibr_calib_exe, '--target', target_filepath, '--dont-show-report', '--models', cam_model, cam_model,
                '--bag', out_bag_filepath, '--topics', '/cam0/image_raw', '/cam1/image_raw']
    if b_show_extract:
        cmds.append('--show-extraction')
    print(cmds)
    if zrpc.map([cmds])[1] == 0:
        exit(-1)

    # rename result files
    res_report_filepath = glob.glob(os.path.join(res_folder, '*.pdf'))[0]
    dst_report_filepath = os.path.join(res_folder, cam_type+'.pdf')
    res_yaml_filepath = glob.glob(os.path.join(res_folder, '*.yaml'))[0]
    dst_yaml_filepath = os.path.join(res_folder, cam_type+'.yaml')
    res_txt_filepath = glob.glob(os.path.join(res_folder, '*.txt'))[0]
    dst_txt_filepath = os.path.join(res_folder, cam_type+'.txt')

    os.rename(res_report_filepath, dst_report_filepath)
    os.rename(res_yaml_filepath, dst_yaml_filepath)
    os.rename(res_txt_filepath, dst_txt_filepath)
