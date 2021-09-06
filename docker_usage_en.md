	In order to standardize the multi-sensor calibration work, we integrate all the algorithms, scripts and tools used in the calibration procedure into a Docker image, to facilitate the deployment of the calibration work in the process of mass production.
**Support **:

Based on the  **Single-Shot Calibration**,  we implement several python scripts to support following  calirbation task:

* camera intrinsic calibration;
* Multi-cam intrinsic and extrinsic calibration;
* IMU intrinsic calibration, Camera-IMU calibration;
* LiDAR-LiDAR extrinsic calirbation;
* Camera-LiDAR extrinsic calibration;
* LiDAR-IMU extrinsic calibration; (TODO)

The Calibration workflow has strict requirements on calibration data catalog and file format, please refer to [test data](https://drive.google.com/file/d/1aaWk44UUGWs6tE-ATOtT1qy7E_eiUQUu/view?usp=sharing).
​

**The Calibration Workflow**

 **1. Intrinsic Calibration**
![image.png](https://intranetproxy.alipay.com/skylark/lark/0/2021/png/225917/1622634471472-2bd8a9eb-27ee-4113-8992-a97ac6a07d80.png#clientId=u12e67438-d0c7-4&from=paste&height=260&id=u844abdfa&margin=%5Bobject%20Object%5D&name=image.png&originHeight=520&originWidth=1294&originalType=binary&ratio=1&size=82703&status=done&style=none&taskId=u23bd9cd7-3c28-497f-bef7-46d881866d8&width=647)

**2.Extrinsic Calibration**
![image.png](https://intranetproxy.alipay.com/skylark/lark/0/2021/png/225917/1622634488953-fa0ef57b-d315-46b9-8782-65ce4c4a3c26.png#clientId=u12e67438-d0c7-4&from=paste&height=447&id=u063e993a&margin=%5Bobject%20Object%5D&name=image.png&originHeight=894&originWidth=910&originalType=binary&ratio=1&size=123854&status=done&style=none&taskId=u0232ba31-e7cf-4426-b3cb-f6f845b3ccf&width=455)






### 1. Pull docker image 
Before run the Docker image, we need to configure the local X server to map the physical display into container. 
**linux**:

```bash

# config xhost for calibration docker 
xhost +local:root
# mount the data path and code path
docker run -it -e "DISPLAY" \
               -e "QT_X11_NO_MITSHM=1" \
               -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
               -v /path/to/code:/code/calirbation \
               -v /media/ziqianbai/DATA:/data \
               single-shot-calib:latest /bin/bash
# compile source code
cd /code/calibration && mkdir build && cd build
cmake .. && make -j8
# cd scripts folder
cd /code/calirbation/scripts
```


**Mac**:
Mac Refer to [url](https://sourabhbajaj.com/blog/2017/02/07/gui-applications-docker-mac/) for XQuartz configuration.

```bash

# config xhost for kalibr docker 
open -a XQuartz
IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
xhost + $IP
# mount the data path and code path
docker run -it -e DISPLAY=$IP:0 -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /media/ziqianbai/DATA:/data -v /path/to/code:/code/calirbation \ single-shot-calib:latest /bin/bash 

# compile source code
cd /code/calibration && mkdir build && cd build
cmake .. && make -j8
# cd scripts folder
cd /code/calirbation/scripts
```



### 2、IMU intrinsic calibration
```bash
python run_imu_intrin_calib.py [ws_folder] [dataset_folder] [output_folder]
```
Output:

   backpack_imu.yaml.


### 3、camera intrinsic calirbation

1. **Mono-cam**:
```bash
python run_cam_intrin_calib.py ws_folder datsaset_folder cam_type output_folder
															[--target_type TARGET_TYPE]  #标定板类型: checkerboard, apriltag, cctag
                               [--extension EXTENSION]  #图像后缀名: jpg png
                               [--dist_model DIST_MODEL] #畸变类型: pinhole-equi, pinhole-radtan
                               [--show_extraction SHOW_EXTRACTION] #是否可视化特征提取过程
                               [--cam0_idx CAM0_IDX] #cam0 对应的相机idx
                               [--cam1_idx CAM1_IDX] #cam1 对应的相机idx

# 例：标定相机cam0
python run_cam_intrin_calib.py /code/kalibr_calibration \
															/data/data0/cams/cam0 \
                              mono \
                              /data/data0/cams/cam0/kalibr \
                              --target_type apriltag
```
Output :

​	  output_folder/result/mono.yaml;


2. **Stereo**:
```bash
python run_cam_intrin_calib.py [ws_folder] [datsaset_folder] stereo [output_folder]
# 例：标定相机stereo0
python run_cam_intrin_calib.py /code/kalibr_calibration \
															/data/data0/cams/stereo0 \
                              stereo \
                              /data/data0/cams/stereo0/kalibr \
                              --target_type apriltag
```
output file: output_folder/result/stereo.yaml;



3. **multi-cam**：
```bash
python  run_multicam_intrin_calib.py    ws_folder 
                                    dataset_folder 
                                    cam_num
                                    output_folder 
																		[--target_type TARGET_TYPE]
                                    [--extension EXTENSION]
                                    [--show_result SHOW_RESULT]
                                    [--cam0_idx CAM0_IDX]
                                    [--cam1_idx CAM1_IDX]
                                    [--cam2_idx CAM2_IDX]
                                    [--cam3_idx CAM3_IDX]


# 例：标定4个相机内参：cam0, cam1, cam2, cam3
python run_multicam_intrin_calib.py /code/kalibr_calibration \
																		/data/ \
                                    4 \
                                    /data/ \
                                    --target_type apriltag
```


### 4、Multi-cam Extrinsic calibration

```bash
python run_multicam_extrin_calib.py [-h] [--target_type TARGET_TYPE]  # 标定板类型: checkerboard, apriltag, cctag
                                    [--extension EXTENSION]           # 图像后缀: png, jpg
                                    [--show_result SHOW_RESULT]       # 是否可视化标定结果
                                    [--cam0_idx CAM0_IDX]             # 目标相机序列号, default:0
                                    [--cam1_idx CAM1_IDX]
                                    [--cam0_intrin_file CAM0_INTRIN_FILE] # 目标相机原始内参文件
                                    [--cam1_intrin_file CAM1_INTRIN_FILE]
                                    [--cam2_intrin_file CAM2_INTRIN_FILE]
                                    [--cam3_intrin_file CAM3_INTRIN_FILE]
                                    [ws_folder] [dataset_folder] [cam_num]
                                    [output_folder]
# 例:标定4目相机系统(只使用1组标定数据)
python run_multicam_extrin_calib.py /code/kalibr_calibration \
			                              /data/data0/cams/multicam \
                                    4 \
                                    /data/data0/cams/multicam \
                                    --target_type cctag
```
cam_num:  1: Mono-Cam; 

​					2: Stereo; 

​					4: Multi-Cam;
Output: 

   1. Mono-Cam: w2c_pose.txt contains the camera pose in w2c format;
   1. Stereo: cam0_cam1.yaml contains the relative pose from camera0 to camera1 frame;
   1. Multi-Cam: cam0_cam1.yaml, cam0_cam2.yaml, cam0_cam3.yaml；



### 5、Cam-IMU calibration
**MonoCam-IMU**

```bash
python run_cam_imu_calib.py ws_folder
														dataset_folder
                            output_folder
                            imu_yaml_file 
                             cam_chain_yaml_file
                            [--target_type TARGET_TYPE]
                            [--extension EXTENSION]
                            [--show_extraction SHOW_EXTRACTION]
                            [--cam0_idx CAM0_IDX]
                              
# 例:
python run_cam_imu_calib.py  /code/kalibr_calibration \
			                       /data/ \
                             /data/  \
                             --target_type apriltag \
                             /data/backpack_imu.yaml \ # imu 内参文件(kalibr输出)
                             /data/cam0_chain.yaml # cam0 内参文件(kalibr输出)
```
    --cam0_idx为单目相机的相机序列号0/1/2/3, 相机序列号为图像的前缀;



### 6、Multi-LiDAR Extrinsic Calibration
```bash
python run_lidar_lidar_calib.py ws_folder \
                                dataset_folder \
                                output_folder \
                                init_T_yaml \ # T_l0_l1的外参初值文件
#例:
python run_lidar_lidar_calib.py /code/kalibr_calibration \
                                /data/ \
                                /data/ \
                                /data/init_lidar0_to_lidar1.yml
```



### 7、LiDAR-Cam Extrinsic Calibration

```bash
python run_lidar_cam_calib.py ws_folder \
                                dataset_folder \
                                output_folder \
                                init_T_yaml \ # T_l1_cam0的外参初值文件
```
