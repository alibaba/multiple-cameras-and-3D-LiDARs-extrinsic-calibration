#!/usr/bin/env python
"""
The script to calib 2 cam
"""

from __future__ import print_function
import os
import logging
import argparse
import shutil
import time
import sys
import csv


imu_headers = ["timestamp", "omega_x", "omega_y",
               "omega_z", "alpha_x", "alpha_y", "alpha_z"]


def utc2local(utc_dtm):
    local_tm = datetime.fromtimestamp(0)
    utc_tm = datetime.utcfromtimestamp(0)
    offset = local_tm - utc_tm
    return utc_dtm + offset


def local2utc(local_dtm):
    return datetime.utcfromtimestamp(local_dtm.timestamp())


def main():
    """
    The main function
    """
    parser = argparse.ArgumentParser(
        description='A script to parse raw data')
    parser.add_argument('input_dir',
                        type=str,
                        default='',
                        help='the stereo dataset directory to be processed')
    parser.add_argument('output_dir',
                        type=str,
                        default='',
                        help='the stereo param result')
    parser.add_argument('imagelist_file',
                        type=str,
                        default='',
                        help='imagelist')
    parser.add_argument('imu_file',
                        type=str,
                        default='imu0_0.txt',
                        help='raw imu data path')
    parser.add_argument('--image_timestamp_file',
                        type=str,
                        default='image_timestamp.txt',
                        help='image timestamp file')
    parser.add_argument('--imu_timestamp_file',
                        type=str,
                        default='imu_tiemstamp.txt',
                        help='imu timestamp file')
    parser.add_argument('--l_folder',
                        type=str,
                        default='left',
                        help='left cam folder')
    parser.add_argument('--r_folder',
                        type=str,
                        default='right',
                        help='right cam folder')
    parser.add_argument('--target_data_path',
                        type=str,
                        default='data',
                        help='target ros data path')
    parser.add_argument('--cam_model',
                        type=str,
                        default='mono',
                        help='camera model: monocular or stereo or stereo_imu')

    args = parser.parse_args()
    input_dir = args.input_dir
    output_dir = args.output_dir
    imagelist_file = args.imagelist_file
    image_ts_file = args.image_timestamp_file
    imu_file = args.imu_file
    imu_ts_file = args.imu_timestamp_file
    l_folder = args.l_folder
    r_folder = args.r_folder
    dataset_path = args.target_data_path
    cam_model = args.cam_model

    l_folder = os.path.join(input_dir, l_folder)
    r_folder = os.path.join(input_dir, r_folder)
    image_ts_file = os.path.join(input_dir, image_ts_file)
    imu_ts_file = os.path.join(input_dir, imu_ts_file)

    # mkdir
    dataset_cam0_path = os.path.join(dataset_path, 'cam0')
    dataset_cam1_path = os.path.join(dataset_path, 'cam1')
    new_imu_file = os.path.join(dataset_path, 'imu0.csv')

    if os.path.exists(dataset_cam0_path):
        shutil.rmtree(dataset_cam0_path)
    os.makedirs(dataset_cam0_path)
    if os.path.exists(dataset_cam1_path):
        shutil.rmtree(dataset_cam1_path)
    os.makedirs(dataset_cam1_path)
    if(os.path.exists(new_imu_file)):
        os.remove(new_imu_file)

    img_timestamps = {}
    with open(image_ts_file, 'r') as img_ts_fd:
        for line in img_ts_fd:
            if(len(line) > 0):
                img_ts = line.split()
                img_timestamps[img_ts[2]] = str(int(float(img_ts[0])))+'000'

    img_start_timestamps = float(img_timestamps['0'])/1000000
    new_imu_datas = []

    with open(imu_file, 'r') as imu_fd:
        imu_datas = imu_fd.readlines()
        imu_datas = [d.strip() for d in imu_datas if len(d.split()) != 0]
        imu_datas.sort(key=lambda x: x.split()[0])

        # import imagelist
        with open(imagelist_file, 'r') as img_fd:
            img_names = img_fd.readlines()
            img_names = [x.strip() for x in img_names]

            for img_name in img_names:
                l_img_path = os.path.join(l_folder, img_name)
                r_img_path = os.path.join(r_folder, img_name)

                img_frame_num = img_name[6:-4]
                l_img_dst_path = os.path.join(
                    dataset_cam0_path, img_timestamps[img_frame_num] + '.' + img_name.split(".")[1])
                r_img_dst_path = os.path.join(
                    dataset_cam1_path, img_timestamps[img_frame_num] + '.' + img_name.split(".")[1])

                # print(l_img_dst_path)
                # print(r_img_dst_path)
                shutil.copyfile(l_img_path, l_img_dst_path)
                if cam_model == "mono_imu":
                    shutil.copyfile(r_img_path, r_img_dst_path)

        for imu_data in imu_datas:
            new_imu_data = imu_data.split()
            imu_timestamp = float(new_imu_data[0])
            if imu_timestamp >= img_start_timestamps:
                new_imu_tuple = (int(imu_timestamp*1000000), new_imu_data[4], new_imu_data[5],
                                new_imu_data[6], new_imu_data[1], new_imu_data[2], new_imu_data[3])
                new_imu_datas.append(new_imu_tuple)

    # write imu.csv
    with open(new_imu_file, 'w') as f:
        f_csv = csv.writer(f)
        f_csv.writerow(imu_headers)
        f_csv.writerows(new_imu_datas)

    cmds = []
    if cam_model == "stereo":
        cmds = ['./kalibr_stereo.sh', output_dir]
    elif cam_model == "mono":
        cmds = ['./kalibr_mono.sh', output_dir]
    elif cam_model == "stereo_imu":
        cmds = ['./kalibr_stereo_imu.sh', output_dir]
    elif cam_model == "mono_imu":
        cmds = ['./kalibr_mono_imu.sh', output_dir]
    else:
        sys.exit("Invalid camera model!")

    cmds = ' '.join(cmds)
    os.system(cmds)


if __name__ == '__main__':
    main()
