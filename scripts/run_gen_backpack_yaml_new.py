
'''
Author: zhangqunkang.zqk zhangqunkang.zqk@alibaba-inc.com
Date: 2022-06-21 16:44:26
Description:
Support for new formats (2022-06-21) 
'''

import yaml
import numpy as np
import argparse
import os
import time
import cv2

localtime = time.asctime( time.localtime(time.time()) )

def np2string(nplist):
    line = str(nplist[0])
    for i in nplist[1:]:
        line += ", {}".format(i)
    return line

# read extrinsics from kalibr yaml file
def readExtFileKalir(filename):
    cv_file = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    T = np.eye(4)
    T[:3,:3] = cv_file.getNode('extrinsic_rotation').mat()
    T[0, 3] = cv_file.getNode('extrinsic_translation').mat()[0]
    T[1, 3] = cv_file.getNode('extrinsic_translation').mat()[1]
    T[2, 3] = cv_file.getNode('extrinsic_translation').mat()[2]
    return T

def readTimeOffsetFileKalir(filename):
    cv_file = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    return cv_file.getNode('timeshift_cam_imu').real()

class PanoramaCamera:
    def __init__(self):
        self.type = 'multi_cam'
        self.frequency = 1
        self.time_offset = 0
        self.image_height = 0
        self.image_width = 0
        self.time_offset = 0.0
        self.extrinsics = np.eye(4)
        self.cams = []
    
    def add_cam(self, cam, T_cam_to_lidar0 = None):
        if T_cam_to_lidar0 is not None:
            cam.T_cam_to_lidar0 = T_cam_to_lidar0
        self.cams.append(cam)
    
    def cam_num(self):
        return len(self.cams)

class Camera:
    def __init__(self, cam_intrin_yaml_path, cam_name, extrinsics = None):
        self.read_cam_intrin(cam_intrin_yaml_path)
        self.name = cam_name
        self.T_cam_to_imu = None
        self.T_cam_to_lidar0 = extrinsics

    def read_cam_intrin(self, path):
        cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        self.type = 'pinhole'
        self.distort_type = cv_file.getNode('camera_model').string()
        self.image_height = int(cv_file.getNode('image_height').real())
        self.image_width = int(cv_file.getNode('image_width').real())
        self.intrinsics = cv_file.getNode('camera_matrix').mat()
        self.distort_coefficient = cv_file.getNode('distortion_coefficients').mat()

class Lidar:
    def __init__(self, name, device_id):
        self.name = name
        self.device_id = device_id
        self.max_dist = 200
        self.min_dist = 0.8
        self.num_rays = 16
        self.frequency = 10
        self.time_offset = 0.0
        self.T_lidar_to_imu = None
        self.T_lidar_to_lidar0 = None

class Imu:
    def __init__(self, yaml_path, name, extrinsics = None):
        self.name = name
        self.extrinsics = extrinsics
        self.time_offset = 0 
        with open(yaml_path, 'r') as fr:
            root_node = yaml.load(fr, Loader=yaml.FullLoader)
            self.frequency = root_node['update_rate']
            self.gyro_meas_noise = root_node['gyroscope_noise_density']
            self.gyro_bias_noise = root_node['gyroscope_random_walk']
            self.acc_meas_noise = root_node['accelerometer_noise_density']
            self.acc_bias_noise = root_node['accelerometer_random_walk']
            self.acc_bias = np.array(root_node['acc_bias']).reshape(1,3)
            self.gyro_bias = np.array(root_node['gyro_bias']).reshape(1,3)

class CalibResult:
    def __init__(self, device_id, time):
        self.time = time
        self.device_id = device_id
        self.sensor_bucket = {
            'panorama_camera': 'multi_cam',
            'horizon_lidar': 'lidar',
            'vertical_lidar': 'lidar',
            'imu_config': 'imu'
        }
        self.panorama = None
        self.lidars = []
        self.imu = None

    def set_panoramacamera(self, cam):
        self.panorama = cam

    def add_lidar(self, lidar):
        self.lidars.append(lidar)
    
    def set_imu(self, imu):
        self.imu = imu

    def writeYAML(self, path):
        with open(path, 'w') as f:
            # Write Header information
            f.write("serial_num: {}\n".format(self.device_id))
            f.write("calibration_time: {}\n".format(self.time))
            f.write("sensor_bucket:\n")
            for key in self.sensor_bucket:
                f.write("  - [{}, {}]\n".format(key, self.sensor_bucket[key]))
            f.write("\n")
            
            # Write Panorama Camera
            for cam in self.panorama.cams:
                f.write("{}: &{}\n".format(cam.name, cam.name))
                f.write("  type: {}\n".format(cam.type))
                f.write("  distort_type: {}\n".format(cam.distort_type))
                f.write("  image_height: {}\n".format(cam.image_height))
                f.write("  image_width: {}\n".format(cam.image_width))
                f.write("  intrinsics: \n    cols: {}\n    rows: {}\n".format(cam.intrinsics.shape[0], cam.intrinsics.shape[1]))
                f.write("    data: [{}]\n".format(np2string(cam.intrinsics.reshape(-1))))
                f.write("  distort_coefficient: \n    cols: {}\n    rows: {}\n".format(cam.distort_coefficient.shape[0], cam.distort_coefficient.shape[1]))
                f.write("    data: [{}]\n".format(np2string(cam.distort_coefficient.reshape(-1))))
                extrinsics = cam.T_cam_to_lidar0
                f.write("  extrinsics: \n    cols: {}\n    rows: {}\n".format(extrinsics.shape[0], extrinsics.shape[1]))
                f.write("    data: [{}]\n".format(np2string(extrinsics.reshape(-1))))           
                f.write("\n")
            # Write Panorama Info
            f.write("panorama_camera:\n")
            f.write("  type: {}\n".format(self.panorama.type))
            f.write("  frequency: {}\n".format(self.panorama.frequency))
            f.write("  time_offset: {}\n".format(self.panorama.time_offset))
            f.write("  camera_num: {}\n".format(self.panorama.cam_num()))
            # f.write("  image_height: {}\n".format(self.panorama.image_height))
            # f.write("  image_width: {}\n".format(self.panorama.image_width))
            extrinsics = self.panorama.extrinsics
            f.write("  extrinsics: \n    cols: {}\n    rows: {}\n".format(extrinsics.shape[0], extrinsics.shape[1]))
            f.write("    data: [{}]\n".format(np2string(extrinsics.reshape(-1))))      
            for i, cam in enumerate(self.panorama.cams):
                f.write("  cam{}: *{}\n".format(i, cam.name))
            f.write("\n")

            # Write Lidars
            for lidar in self.lidars:
                f.write("{}: \n".format(lidar.name))
                f.write("  lidar_dev_id: {}\n".format(lidar.device_id))
                f.write("  max_dist: {}\n".format(lidar.max_dist))
                f.write("  min_dist: {}\n".format(lidar.min_dist))
                f.write("  num_rays: {}\n".format(lidar.num_rays))
                f.write("  frequency: {}\n".format(lidar.frequency))
                f.write("  time_offset: {}\n".format(lidar.time_offset))
                extrinsics = lidar.T_lidar_to_lidar0
                f.write("  extrinsics: \n    cols: {}\n    rows: {}\n".format(extrinsics.shape[0], extrinsics.shape[1]))
                f.write("    data: [{}]\n".format(np2string(extrinsics.reshape(-1)))) 
                f.write("\n")

            # Write IMU
            f.write("imu_config: \n")
            f.write("  name: \"{}\"\n".format(self.imu.name))
            f.write("  frequency: {}\n".format(self.imu.frequency))
            f.write("  time_offset: {}\n".format(self.imu.time_offset))
            extrinsics = self.imu.extrinsics
            f.write("  extrinsics: \n    cols: {}\n    rows: {}\n".format(extrinsics.shape[0], extrinsics.shape[1]))
            f.write("    data: [{}]\n".format(np2string(extrinsics.reshape(-1)))) 
            f.write("  gyro_meas_noise: {}\n".format(self.imu.gyro_meas_noise))
            f.write("  gyro_bias_noise: {}\n".format(self.imu.gyro_bias_noise))
            f.write("  acc_meas_noise: {}\n".format(self.imu.acc_meas_noise))
            f.write("  acc_bias_noise: {}\n".format(self.imu.acc_bias_noise))
            f.write("  gyro_bias: \n    cols: {}\n    rows: {}\n".format(self.imu.gyro_bias.shape[0], self.imu.gyro_bias.shape[1]))
            f.write("    data: [{}]\n".format(np2string(self.imu.gyro_bias.reshape(-1)))) 
            f.write("  acc_bias: \n    cols: {}\n    rows: {}\n".format(self.imu.acc_bias.shape[0], self.imu.acc_bias.shape[1]))
            f.write("    data: [{}]\n".format(np2string(self.imu.acc_bias.reshape(-1)))) 
            f.write("\n")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Usage : test_gen_backpack_yaml [calib_dataset_folder] [output_folder] [device_id] [cam_num]\n")
    parser.add_argument('input_folder', type=str, help='the input dataset folder')
    parser.add_argument('output_folder', type=str, help='the folder of output files')
    parser.add_argument('device_id', type=int, help='data timestamp duration(min)')

    # default cam num = 5

    args = parser.parse_args()
    backpack_device_name = "backpack"+ str(args.device_id)
    input_folder = args.input_folder
    output_folder = args.output_folder

    # extrinsic files
    lidar0_2_lidar1_filepath = os.path.join(input_folder, "lidar0_to_lidar1.yml");
    lidar1_2_cam0_filepath = os.path.join(input_folder, "lidar1_to_camera0.yml");
    cam0_2_cam1_filepath = os.path.join(input_folder, "camera0_to_camera1.yml");
    cam0_2_cam2_filepath = os.path.join(input_folder, "camera0_to_camera2.yml");
    cam0_2_cam3_filepath = os.path.join(input_folder, "camera0_to_camera3.yml");
    cam0_2_cam4_filepath = os.path.join(input_folder, "camera0_to_camera4.yml");
    cam0_2_imu_filepath = os.path.join(input_folder, "camera0_to_imu.yaml");
    # intrinsic files 
    cam0_intrin_filepath = os.path.join(input_folder, "cam0.yml");
    cam1_intrin_filepath = os.path.join(input_folder, "cam1.yml");
    cam2_intrin_filepath = os.path.join(input_folder, "cam2.yml");
    cam3_intrin_filepath = os.path.join(input_folder, "cam3.yml");
    cam4_intrin_filepath = os.path.join(input_folder, "cam4.yml");
    imu_intrin_filepath = os.path.join(input_folder, backpack_device_name+"_imu.yaml");

    # Set Output filepath
    output_filepath =  os.path.join(output_folder, "raw_" + backpack_device_name +".yaml");
    
    # Read extrinsics 
    T_cam0_to_lidar1 = readExtFileKalir(lidar1_2_cam0_filepath)
    T_lidar1_to_lidar0 = readExtFileKalir(lidar0_2_lidar1_filepath)
    T_imu_to_cam0 = readExtFileKalir(cam0_2_imu_filepath)
    timeoffset_cam0_to_imu = readTimeOffsetFileKalir(cam0_2_imu_filepath)
    T_cam1_to_cam0 = readExtFileKalir(cam0_2_cam1_filepath)
    T_cam2_to_cam0 = readExtFileKalir(cam0_2_cam2_filepath)
    T_cam3_to_cam0 = readExtFileKalir(cam0_2_cam3_filepath)
    T_cam4_to_cam0 = readExtFileKalir(cam0_2_cam4_filepath)

    # Trans extrinsics : sensor to lidar0
    T_cam0_to_imu = np.linalg.inv(T_imu_to_cam0)
    T_cam0_to_lidar0 = T_lidar1_to_lidar0 @ T_cam0_to_lidar1
    T_cam1_to_lidar0 = T_cam0_to_lidar0 @ T_cam1_to_cam0
    T_cam2_to_lidar0 = T_cam0_to_lidar0 @ T_cam2_to_cam0
    T_cam3_to_lidar0 = T_cam0_to_lidar0 @ T_cam3_to_cam0
    T_cam4_to_lidar0 = T_cam0_to_lidar0 @ T_cam4_to_cam0
    T_imu_to_lidar0 = T_cam0_to_lidar0 @ T_imu_to_cam0
    T_lidar0_to_lidar0 = np.eye(4)

    test_sensor_to_imu = True
    if test_sensor_to_imu:
        T_cam1_to_imu = T_cam0_to_imu @ T_cam1_to_cam0
        T_cam2_to_imu = T_cam0_to_imu @ T_cam2_to_cam0
        T_cam3_to_imu = T_cam0_to_imu @ T_cam3_to_cam0
        T_cam4_to_imu = T_cam0_to_imu @ T_cam4_to_cam0
        T_lidar0_to_imu = T_cam0_to_imu @ np.linalg.inv(T_cam0_to_lidar0)
        T_lidar1_to_imu = T_lidar0_to_imu @ T_lidar1_to_lidar0

        print("T_cam0_to_imu : \n {}".format(T_cam0_to_imu))
        print("T_cam1_to_imu : \n {}".format(T_cam1_to_imu))
        print("T_cam2_to_imu : \n {}".format(T_cam2_to_imu))
        print("T_cam3_to_imu : \n {}".format(T_cam3_to_imu))
        print("T_cam4_to_imu : \n {}".format(T_cam4_to_imu))
        print("T_lidar0_to_imu : \n {}".format(T_lidar0_to_imu))
        print("T_lidar1_to_imu : \n {}".format(T_lidar1_to_imu))

    test_sensor_to_lidar0 = True
    if test_sensor_to_lidar0:
        print("T_cam0_to_lidar0 : \n {}".format(T_cam0_to_lidar0))
        print("T_cam1_to_lidar0 : \n {}".format(T_cam1_to_lidar0))
        print("T_cam2_to_lidar0 : \n {}".format(T_cam2_to_lidar0))
        print("T_cam3_to_lidar0 : \n {}".format(T_cam3_to_lidar0))
        print("T_cam4_to_lidar0 : \n {}".format(T_cam4_to_lidar0))
        print("T_imu_to_lidar0 : \n {}".format(T_imu_to_lidar0))
        print("T_lidar1_to_lidar0 : \n {}".format(T_lidar1_to_lidar0))

    # Add Sensors
    imu = Imu(imu_intrin_filepath, "imu", extrinsics=T_imu_to_lidar0)
    lidar0 = Lidar("horizon_lidar", 0)
    lidar1 = Lidar("vertical_lidar", 1)
    lidar0.T_lidar_to_lidar0 = T_lidar0_to_lidar0
    lidar1.T_lidar_to_lidar0 = T_lidar1_to_lidar0
    lidar0.time_offset = 0.0 # 目前为手动设置
    lidar1.time_offset = 0.0 # 目前为手动设置
    Panoramacam = PanoramaCamera()
    Panoramacam.time_offset = timeoffset_cam0_to_imu
    Panoramacam.add_cam(Camera(cam0_intrin_filepath, "teche_0", extrinsics=T_cam0_to_lidar0))
    Panoramacam.add_cam(Camera(cam1_intrin_filepath, "teche_1", extrinsics=T_cam1_to_lidar0))
    Panoramacam.add_cam(Camera(cam2_intrin_filepath, "teche_2", extrinsics=T_cam2_to_lidar0))
    Panoramacam.add_cam(Camera(cam3_intrin_filepath, "teche_3", extrinsics=T_cam3_to_lidar0))
    Panoramacam.add_cam(Camera(cam4_intrin_filepath, "teche_4", extrinsics=T_cam4_to_lidar0))

    calibresult = CalibResult( args.device_id, localtime)
    calibresult.add_lidar(lidar0)
    calibresult.add_lidar(lidar1)
    calibresult.set_imu(imu)
    calibresult.set_panoramacamera(Panoramacam)
    calibresult.writeYAML(output_filepath)
    print("YAML File saved in : ", output_filepath)