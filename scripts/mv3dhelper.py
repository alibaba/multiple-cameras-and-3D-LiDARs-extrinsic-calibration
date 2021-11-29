import os
import sys
import argparse
import shutil
import multiprocessing
import glob
import math
import platform
import json
import time
import yaml
import zipfile
import re
import cv2 as cv
import numpy as np


def write_pipeline_status(progress, success, error_step=str(), error_info=str()):
    status_config = dict()
    if success == True:
        status_config["success"] = "true"
    else:
        status_config["success"] = "false"
    status_config["progress"] = str(progress)
    status_config["errorMsg"] = str(error_step)
    status_config["errorCode"] = str(error_info)
    time_info = time.time()
    status_config["errorCode"] = str(error_info)
    status_config["time"] = str(time_info)
    status_json = json.dumps(status_config)
    print >> sys.stderr.write("VIRTUALBUY:" + str(status_json) + '\n')


def read_device_param_file(device_param_file):
    if os.path.exists(device_param_file):
        with open(device_param_file, 'r') as f:
            device_param = json.load(f)
            return device_param
    else:
        param = dict()
        return param


def auto_detect_binary(bin_str):
    if platform.system() == 'Windows':
        dir_name = os.path.dirname(sys.argv[0])
        bin_name = os.path.join(dir_name, bin_str+'.exe')
        if os.path.exists(bin_name):
            return bin_name
        else:
            return None
    else:
        dir_name = os.path.dirname(sys.argv[0])
        bin_name = os.path.join(dir_name, bin_str)
        if os.path.exists(bin_name):
            return bin_name
        else:
            if os.path.exists(bin_str):
                return bin_str
            else:
                return None


def safe_copy_file(src_path, dst_path):
    if os.path.exists(dst_path):
        os.remove(dst_path)
    if os.path.exists(src_path):
        shutil.copyfile(src_path, dst_path)
    else:
        print('{0} does not exist'.format(src_path))


def safe_copy_folder(src_path, dst_path):
    if os.path.isdir(dst_path):
        shutil.rmtree(dst_path)
    if os.path.isdir(src_path):
        shutil.copytree(src_path, dst_path)


def batch_copy_files(src_folder, dst_folder, cp_list):
    '''
            param cp_list is a list of tuple (src_name, dst_name, error_msg)
                      if error_msg is non-empty, the program will exit if src_file does not exist
    '''
    print('Copy files:')
    for i in range(0, len(cp_list)):
        cp_info = cp_list[i]
        error_msg = cp_info[2]
        src_path = os.path.join(src_folder, cp_info[0])
        dst_path = os.path.join(dst_folder, cp_info[1])
        if len(error_msg) > 0:
            check_file_exist(src_path, error_msg)
        print('{0} --> {1}'.format(src_path, dst_path))
        safe_copy_file(src_path, dst_path)


def check_file_exist(file_path, file_description):
    if not os.path.exists(file_path):
        print('Error: The {0} {1} does not exist.'.format(
            file_description, file_path))
        write_pipeline_status(0, False,
                              "File integrity check failed.",
                              'Error: The {0} {1} does not exist.'.format(file_description, file_path))
        exit(-1)


def check_folder_empty(folder_path):
    if len(os.listdir(folder_path)) == 0:
        return True
    else:
        return False


def check_folder_exist(folder_path, folder_description):
    if not os.path.isdir(folder_path):
        print('Error: The {0} {1} does not exist.'.format(
            folder_description, folder_path))
        write_pipeline_status(0, False,
                              "File integrity check failed.",
                              'Error: The {0} {1} does not exist.'.format(folder_description, folder_path))
        exit(-1)


def generate_file_list(file_list, folder_path, file_pattern):
    file_num = 0
    with open(file_list, "w") as file_stream:
        file_paths = glob.glob(os.path.join(folder_path, file_pattern))
        for file_path in sorted(file_paths):
            file_stream.write(file_path+'\n')
            file_num = file_num + 1

    return file_num


def write_file_list(file_list, file_path):
    with open(file_path, 'w') as fs:
        for file in sorted(file_list):
            fs.write(file + '\n')
    return len(file_list)


def create_folder_if_not_exists(folder_path):
    if not os.path.exists(folder_path):
        print('create folder: {0}'.format(folder_path))
        os.mkdir(folder_path)


def read_point_cloud_ranges(file_path):
    f = open(file_path)
    line_data = f.readline()
    z_min = float(line_data.split(':')[1])
    line_data = f.readline()
    z_max = float(line_data.split(':')[1])
    print(" z_min: {0}, z_max: {1}".format(z_min, z_max))
    return z_min, z_max


def update_config_file(config_file_path, subnode_key, subnode_value):
    with open(config_file_path, 'r') as fr:
        root_node = yaml.load(fr)
        root_node["config_file_name"][subnode_key] = subnode_value

    with open(config_file_path, 'w') as fw:
        yaml.dump(root_node, fw, default_flow_style=False)


def compress_file(zipfilename, dir_name):
    if os.path.isfile(dir_name):
        with zipfile.ZipFile(zipfilename, 'w', allowZip64=True) as z:
            z.write(dir_name)
    else:
        with zipfile.ZipFile(zipfilename, 'w', allowZip64=True) as z:
            parent_dir = os.path.dirname(dir_name)
            for root, dirs, files in os.walk(dir_name):
                for single_file in files:
                    if single_file != zipfilename:
                        src_path = os.path.join(root, single_file)
                        target_path = os.path.join(
                            root.replace(parent_dir, ""), single_file)
                        z.write(src_path, target_path)


def oss_upload(ossutil_exe, oss_url, local_datas):
    for src_file in local_datas:
        file_fields = [field for field in src_file.split(
            "/") if len(field) > 0]
        if os.path.isdir(src_file):
            src_folder_name = file_fields[-1]
            zip_src_file_name = src_folder_name + ".zip"
            zip_src_file_path = src_file + ".zip"
            if not os.path.exists(zip_src_file_path):
                compress_file(zip_src_file_path, src_file)

            target_url_path = os.path.join(oss_url, zip_src_file_name)
            cmd = ossutil_exe + " cp -f " + zip_src_file_path + " " + target_url_path
            os.system(cmd)
        if os.path.isfile(src_file):
            src_file_name = file_fields[-1]
            target_url_path = os.path.join(oss_url, src_file_name)
            cmd = ossutil_exe + " cp -f " + src_file + " " + target_url_path
            os.system(cmd)

# sort and rename images under input folder


def sort_rename_images(input_folder, raw_prefix, out_prefix, extension):
    """
    :param input_folder: Imu file path
    :param raw_prefix: e.g. 0_
    :param out_prefix: e.g. image_
    :param extension: e.g. .jpg or .png
    :return: the sorted image name list.
    """
    img_names = [img for img in os.listdir(
        input_folder) if img.endswith(extension)]

    def sort_by_suffix(img_name):
        delim_pattern = '\_|\.'
        spot_idx = re.split(delim_pattern, img_name)[1]
        # print('spot_idx: {}'.format(spot_idx))
        return int(spot_idx)

    sorted_img_names = sorted(img_names, key=sort_by_suffix)
    # for idx in range(0, len(sorted_img_names)):
    #     new_img_name = out_prefix + str(idx) + extension
    #     old_img_abs_path = os.path.join(
    #         input_folder, sorted_img_names[idx])
    #     new_img_abs_path = os.path.join(input_folder, new_img_name)
    #     os.rename(old_img_abs_path, new_img_abs_path)
    return sorted_img_names


def readImageTms(raw_cam_tms_filepath, device='BACKPACK'):
    """
    :param raw_cam_tms_filepath: image timestamp file
    :return: the image timestamp map{img_idx:timestamp}.
    """
    img_timestamps = {}
    with open(raw_cam_tms_filepath, 'r') as img_ts_fd:
        for line in img_ts_fd:
            if(len(line) > 0):
                if device == 'BACKPACK':
                    # data: timestamp cam_idx img_idx
                    img_ts = line.split()
                    # src time unit: s, new time unit: ns
                    img_timestamps[img_ts[2]] = str(
                        int(float(img_ts[0]) * 1000000))+'000'
                elif device == 'SIM' or device == 'XVISIO':
                    # data: timestamp cam_idx img_idx
                    img_ts = line.split()
                    # src time unit: ns, new time unit: ns
                    img_timestamps[img_ts[2]] = str(int(float(img_ts[0])))

    return img_timestamps


def readImuData(raw_imu_file, device='BACKPACK'):
    """
    :param raw_imu_file: imu file
    :param device: 'BACKPACK', 'KALEIDO', 'XVISIO', 'SIM'
    :return: the imu data list [['tms', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'],...]. tms unit is us.
    """
    with open(raw_imu_file, 'r') as imu_fd:
        imu_datas = imu_fd.readlines()
        if device == 'BACKPACK':
            imu_datas = [d.split() for d in imu_datas if len(d) != 0]
        elif device == 'XVISIO':
            imu_datas = [d.split() for d in imu_datas if len(d) != 0]
        elif device == 'SIM':
            raw_imu_datas = [d.strip().split(',') for d in imu_datas if len(d) != 0]
            # convert timestamp from ns to us
            imu_datas = [[d[0], d[4], d[5], d[6], d[1], d[2], d[3]] for d in raw_imu_datas] 
        imu_datas.sort(key=lambda x: float(x[0]))
        return imu_datas


class SpotMetaItem:
    def __init__(self):
        self.scene_name = ''
        self.spot_name = ''
        self.device_serial = ''
        self.pose = [float('nan'), float('nan'), float('nan'), float(
            'nan'), float('nan'), float('nan'), float('nan')]
        self.neighbors = []
        # additional rectify pose
        self.rectify_pose = [float('nan'), float('nan'), float(
            'nan'), float('nan'), float('nan'), float('nan'), float('nan')]

    @staticmethod
    def parse_spot_meta_file(spot_meta_file):
        spot_meta_item_list = []
        with open(spot_meta_file, 'r') as fs:
            lines = sorted(fs.readlines())
            for idx in range(len(lines)):
                line = lines[idx]
                elements = line.strip().split(' ')
                assert len(elements) > 1
                spot_meta = SpotMetaItem()
                spot_meta.spot_name = elements[0]
                spot_meta.device_serial = elements[1]

                # get angle-axis rotation
                if len(elements) > 5:
                    spot_meta.pose[0: 4] = [float(x) for x in elements[2: 6]]

                # get translation
                if len(elements) > 8:
                    spot_meta.pose[4:] = [float(x) for x in elements[6: 9]]

                # get neigbors
                if len(elements) > 9:
                    neighbors_cont = int(elements[9])
                    if neighbors_cont == -1:  # -1 means connected all spots
                        spot_meta.neighbors = [
                            x for x in range(len(lines)) if x != idx]
                    else:
                        # check spot neighbors valid
                        if len(elements) - 10 < neighbors_cont or neighbors_cont < 0:
                            print('Invalid spot meta file found: {0}'.format(
                                spot_meta_file))
                            write_pipeline_status(0, False,
                                                  "File integrity check failed.",
                                                  'Invalid spot meta file found: {0}'.format(spot_meta_file))
                            exit(-1)
                        for x in range(neighbors_cont):
                            spot_meta.neighbors.append(int(elements[10 + x]))

                    # get additional spot rectify pose
                    if len(elements) > 13 + max(neighbors_cont, 0):
                        spot_meta.rectify_pose[0: 4] = [float(
                            x) for x in elements[10 + max(neighbors_cont, 0): 14 + max(neighbors_cont, 0)]]
                    if len(elements) > 16 + max(neighbors_cont, 0):
                        spot_meta.rectify_pose[4:] = [float(
                            x) for x in elements[14 + max(neighbors_cont, 0): 17 + max(neighbors_cont, 0)]]

                spot_meta_item_list.append(spot_meta)
            print('Extract {0} spot meta in {1}'.format(
                len(spot_meta_item_list), spot_meta_file))
        return spot_meta_item_list

    def to_string(self, merge_neighbor):
        string_line = self.spot_name + ' ' + self.device_serial + \
            ' ' + ' '.join(str(x) for x in self.pose)
        if merge_neighbor:
            string_line += ' -1'
        else:
            string_line += ' ' + str(len(self.neighbors)) + \
                ' ' + ' '.join(str(x) for x in self.neighbors)
        # add additional spot rectify pose
        if not any(math.isnan(x) for x in self.rectify_pose):
            string_line += ' ' + ' '.join(str(x) for x in self.rectify_pose)
        return string_line

def saveExtFileOpencv(filename, T_ext):
    fs = cv.FileStorage(filename, cv.FileStorage_WRITE)
    R = T_ext[:3, :3]
    t = T_ext[:3, 3]
    fs.write('extrinsic_rotation', R)
    fs.write('extrinsic_translation', t)
    fs.release()
    return True

def readExtFileOpencv(extrin_filepath):
    fs = cv.FileStorage(extrin_filepath, cv.FileStorage_READ)
    # rotation axis to camera
    R_r_c = fs.getNode('extrinsic_rotation').mat()
    t_r_c = fs.getNode('extrinsic_translation').mat()
    T_r_c = np.identity(4, np.float)
    T_r_c[:3, :3] = R_r_c
    T_r_c[:3, 3] = t_r_c.T
    fs.release()
    return T_r_c
