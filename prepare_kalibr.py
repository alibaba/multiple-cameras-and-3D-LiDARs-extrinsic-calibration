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
                        help='camera model: monocular or stereo')

    args = parser.parse_args()
    input_dir = args.input_dir
    output_dir = args.output_dir
    imagelist_file = args.imagelist_file
    l_folder = args.l_folder
    r_folder = args.r_folder
    dataset_path = args.target_data_path
    cam_model = args.cam_model

    if cam_model == 'mono':
        l_folder = os.path.join(input_dir, "data")
    elif cam_model == 'stereo':
        l_folder = os.path.join(input_dir, l_folder)
        r_folder = os.path.join(input_dir, r_folder)
    else:
        sys.exit("Invalid camera model!")

    # mkdir
    dataset_cam0_path = os.path.join(dataset_path, 'cam0')
    dataset_cam1_path = os.path.join(dataset_path, 'cam1')

    if os.path.exists(dataset_cam0_path):
        shutil.rmtree(dataset_cam0_path)
    os.makedirs(dataset_cam0_path)
    if os.path.exists(dataset_cam1_path):
        shutil.rmtree(dataset_cam1_path)
    if cam_model == "stereo":
        os.makedirs(dataset_cam1_path)

    # import imagelist
    with open(imagelist_file, 'r') as in_file:
        img_names = in_file.readlines()
        img_names = [x.strip() for x in img_names]

        for img_name in img_names:
            l_img_path = os.path.join(l_folder, img_name)
            ts = int(time.time() * 1e9)
            l_img_dst_path = os.path.join(
                dataset_cam0_path, str(ts) + '.' + img_name.split(".")[1])
            shutil.copyfile(l_img_path, l_img_dst_path)

            if cam_model == "stereo":
                r_img_path = os.path.join(r_folder, img_name)
                r_img_dst_path = os.path.join(
                    dataset_cam1_path, str(ts) + '.' + img_name.split(".")[1])
                shutil.copyfile(r_img_path, r_img_dst_path)
            time.sleep(0.03)
            # print(l_img_dst_path)
            # print(r_img_dst_path)

    cmds = []
    if cam_model == "stereo":
        cmds = ['./kalibr_stereo.sh', output_dir]
    elif cam_model == "mono":
        cmds = ['./kalibr_mono.sh', output_dir]
    elif cam_model == "stereo_imu":
        cmds = ['./kalibr_stereo_imu.sh', output_dir]
    else:
        sys.exit("Invalid camera model!")

    cmds = ' '.join(cmds)
    os.system(cmds)


if __name__ == '__main__':
    main()
