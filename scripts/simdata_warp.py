import os,sys
import shutil
import argparse
import cv2
import numpy as np
import mv3dhelper

def help_information():
    print('''Usage: python simdata_warp.py <raw_data_folder> <target_data_folder>'''
    '''' warp raw data of SIM ARGlass into the form of VCalib library.''')

def splitStereoImages(raw_stereo_folder, new_stereo_folder):
    new_left_cam_folder = os.path.join(new_stereo_folder, 'stereo', 'cam0')
    new_right_cam_folder = os.path.join(new_stereo_folder, 'stereo', 'cam1')
    if not os.path.exists(new_left_cam_folder):
        os.makedirs(new_left_cam_folder)
    if not os.path.exists(new_right_cam_folder):
        os.makedirs(new_right_cam_folder)

    left_img_tms_file = os.path.join(new_left_cam_folder, 'timestamp_0.txt')
    right_img_tms_file = os.path.join(new_right_cam_folder, 'timestamp_1.txt')

    l_tms_fd = open(left_img_tms_file, 'w')
    r_tms_fd = open(right_img_tms_file, 'w')

    raw_imgs = [img for img in os.listdir(
        raw_stereo_folder) if img.endswith('.pgm')]

    for img_name in raw_imgs:
        raw_img_path = os.path.join(raw_stereo_folder, img_name)
        print("raw img path: ", raw_img_path)
        img = cv2.imread(raw_img_path)

        img_height = int(img.shape[0])
        img_width = int(img.shape[1])
        print("image height: {}, image width: {}".format(img_height, img_width))
        # print('image : {}'.format(img[0,:]))

        left_cam = img[0:img_height, 0:img_width//2]
        right_cam = img[0:img_height, img_width//2:img_width]

        l_img_name = '0_' + img_name.split('.')[0] + '.jpg'
        left_img_path = os.path.join(new_left_cam_folder, l_img_name)
        r_img_name = '1_' + img_name.split('.')[0] + '.jpg'
        right_img_path = os.path.join(new_right_cam_folder, r_img_name)

        cv2.imwrite(left_img_path, left_cam)
        cv2.imwrite(right_img_path, right_cam)

        tms = img_name.split('.pgm')[0]
        l_img_name = '0 ' + tms
        l_tms_fd.write(tms+ ' ' + l_img_name + '\n')
        r_img_name = '1 ' + tms
        r_tms_fd.write(tms+ ' ' + r_img_name + '\n')

    l_tms_fd.close()
    r_tms_fd.close()


def warpSimRawData(input_folder, output_folder):
    # stereo images
    raw_img_list = [ img for img in os.listdir(input_folder) if img.endswith('.pgm')]
    raw_imu_file = os.path.join(input_folder,'IMU0.csv')
    if len(raw_img_list) == 0 or not os.path.exists(raw_imu_file):
        print('Empty input folder!!!')
        return False

    # split raw stereo images into cam0, cam1 folders
    splitStereoImages(input_folder, output_folder)

    # the raw imu tms unit is ns, we convert it into ns(same as BACKPACK imu_.txt)
    imu_data = mv3dhelper.readImuData(raw_imu_file, device='SIM')
    new_imu_file = os.path.join(output_folder, 'imu_0.txt')
    with open(new_imu_file, 'w') as new_fd:
        for data in imu_data:
            print(data)
            tms = int(data[0]) / 1000
            data[0] = str(tms)
            str_data = ' '.join(data)
            new_fd.write(str_data)
            new_fd.write('\n')
    return 

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="A helper to warp SIM ARGlass data.")

    parser.add_argument('dataset_folder', type=str, default="",
                        help='the input dataset folder')
    parser.add_argument('output_folder', type=str, default='',
                        help='the folder of output files')

    args = parser.parse_args()
    input_folder = args.dataset_folder
    output_folder = args.output_folder

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    warpSimRawData(input_folder, output_folder)