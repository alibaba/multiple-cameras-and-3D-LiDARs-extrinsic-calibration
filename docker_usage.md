	为了进一步标准化背包标定工作, 我们将标定用到的所有算法、脚本、工具集成在一个docker镜像中, 方便在批量生产过程中标定工作的部署.
由于背包设备上的传感器种类和数量都比较多，所以背包标定工作比较繁琐, 具体由内参标定和外参标定工作两个部分组成. 内参标定包括有IMU内参标定、每个相机内参标定， 外参标定包括相机-相机外参标定、相机-IMU外参标定、LiDAR-相机外参标定、LiDAR-IMU外参标定(未开发)。


！！！标定工作流对标定数据有严格的目录、文件格式要求, 请见标定数据规范([语雀](https://yuque.antfin-inc.com/aone857851/udonh7/bmazpr)). 我们要求每种标定任务对应的标定数据至少3组，但是特殊情况（如imu内参标定需要2h的数据，这种情况下1组标定数据也可以）可以列外。
​

**标定工作整体流程**
整个标定工作的流程如下图所示。在数据生产环节，需要先进行数据标准化，即按照既定标定数据规范整理数据，然后数据上云，运行标定任务，先进行内参标定，在进行外参标定。
![image.png](https://intranetproxy.alipay.com/skylark/lark/0/2021/png/225917/1622628189328-65ced072-b8c8-42c0-a39d-203eb903ab20.png#clientId=u12e67438-d0c7-4&from=paste&height=208&id=uf20d8615&margin=%5Bobject%20Object%5D&name=image.png&originHeight=416&originWidth=1180&originalType=binary&ratio=1&size=91243&status=done&style=none&taskId=uea1e37f4-b2b9-4320-ad1a-e35267f6ed4&width=590)


 **1. 内参标定**
![image.png](https://intranetproxy.alipay.com/skylark/lark/0/2021/png/225917/1622634471472-2bd8a9eb-27ee-4113-8992-a97ac6a07d80.png#clientId=u12e67438-d0c7-4&from=paste&height=260&id=u844abdfa&margin=%5Bobject%20Object%5D&name=image.png&originHeight=520&originWidth=1294&originalType=binary&ratio=1&size=82703&status=done&style=none&taskId=u23bd9cd7-3c28-497f-bef7-46d881866d8&width=647)


**2.外参标定**
![image.png](https://intranetproxy.alipay.com/skylark/lark/0/2021/png/225917/1622634488953-fa0ef57b-d315-46b9-8782-65ce4c4a3c26.png#clientId=u12e67438-d0c7-4&from=paste&height=447&id=u063e993a&margin=%5Bobject%20Object%5D&name=image.png&originHeight=894&originWidth=910&originalType=binary&ratio=1&size=123854&status=done&style=none&taskId=u0232ba31-e7cf-4426-b3cb-f6f845b3ccf&width=455)




​

### 一、拉取、使用镜像
因为镜像中使用到的标定工具kalibr在生成pdf报告这个环节需要映射物理显示器, 所以我们在使用docker镜像前需要配置本机的X服务器
**linux平台下运行标定工具集docker镜像**:
```bash
docker pull reg.docker.alibaba-inc.com/backpack/kalibr:latest

# config xhost for kalibr docker 
xhost +local:root
# 将代码和待标定数据挂载到docker镜像中
docker run -it -e "DISPLAY" \
               -e "QT_X11_NO_MITSHM=1" \
               -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
               -v /media/ziqianbai/DATA:/data \
               reg.docker.alibaba-inc.com/backpack/kalibr:latest /bin/bash
# 
cd /code/kalibr_calibration/scripts
```
第一个 -e, 第二个-e和第一个-v都是为了开启docker图形化功能做的配置;
第二个-v 是把本地的 /media/ziqianbai/DATA 挂载到 docker container 的/data路径;
~~第三个 -v 是把本地的 /home/ziqianbai/Projects/vlab/VCalib 挂载到 docker container 的 /code/kalibr_calibration 路径;~~
**​**

**Mac平台下运行标定工具集docker镜像**:
Mac平台下运行本镜像需要提前安装XQuartz, 首次安装配置好之后需要重启电脑才能生效, 具体教程可参见[连接](https://sourabhbajaj.com/blog/2017/02/07/gui-applications-docker-mac/);
```bash
docker pull reg.docker.alibaba-inc.com/backpack/kalibr:latest

# config xhost for kalibr docker 
open -a XQuartz
IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
xhost + $IP
# 将代码和待标定数据挂载到docker镜像中
docker run -it -e DISPLAY=$IP:0 -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v /media/ziqianbai/DATA:/data -v /home/ziqianbai/Projects/vlab/VCalib:/code/kalibr_calibration reg.docker.alibaba-inc.com/backpack/kalibr:latest /bin/bash
cd /code/kalibr_calibration/scripts
```


**Windows平台下运行标定工具集docker镜像：**
Windows平台下运行本镜像需要提前安装好VcXsrv, 具体安装过程参考[Run GUI app in linux docker container on windows host](https://dev.to/darksmile92/run-gui-app-in-linux-docker-container-on-windows-host-4kde)。安装运行, 后打开Docker for windows:
```bash
docker pull reg.docker.alibaba-inc.com/backpack/kalibr:latest

# 将代码和待标定数据挂载到docker镜像中
docker run -it -e DISPLAY=host.docker.internal:0.0 -v /media/ziqianbai/DATA:/data -v /home/ziqianbai/Projects/vlab/VCalib:/code/kalibr_calibration reg.docker.alibaba-inc.com/backpack/kalibr:latest /bin/bash
cd /code/kalibr_calibration/scripts
```
​

### 二、标定IMU内参
标定IMU内参需要将IMU传感器静止2h，采集这段时间的静止数据来标定IMU器件噪声。IMU文件格式请参考[标准数据格式定义](https://yuque.antfin-inc.com/aone857851/pcsedu/ksqs4u). imu内参标定数据可以只采集一组即可。
imu内参标定命令:
```bash
python run_imu_intrin_calib.py [ws_folder] [dataset_folder] [output_folder]
```
在output_folder会生成imu内参文件 backpack_imu.yaml.


### 三、标定相机内参

1. **只标定一个单目相机的内参**:(这里只使用了1组标定数据)
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
生成单目相机内参文件output_folder/result/mono.yaml;


2. **只标定一个双目相机的内外参**:(这里只使用了1组标定数据)
```bash
python run_cam_intrin_calib.py [ws_folder] [datsaset_folder] stereo [output_folder]
# 例：标定相机stereo0
python run_cam_intrin_calib.py /code/kalibr_calibration \
															/data/data0/cams/stereo0 \
                              stereo \
                              /data/data0/cams/stereo0/kalibr \
                              --target_type apriltag
```
生成双目相机外参文件output_folder/result/stereo.yaml;
上述单目/双目标定的结果作为[cams_chain_yaml_file]输入到后续相机-IMU标定过程.


3. **标定多个相机的内参**：（这里要使用至少2组标定数据）
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


### 四、标定多相机外参
多相机外参依赖标定房间来完成外参标定，可以实现单目相机外参标定(单目相机定位)、双目相机外参标定、四目环视相机外参标定。
因为目前背包硬件的相机处于不稳定迭代阶段, 我们的相机外参标定功能处于标定1组数据，然后查看log信息，人工确定标定结果是否可用，确定当前标定结果可用才会重复操作，标定下1组数据。
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
cam_num为需要标定的相机数量, 1: 单目相机标定; 2: 双目相机外参标定; 4: 四目环视相机外参标定;
外参标定输出结果: 

   1. 单目相机标定结果: w2c_pose.txt 相机相对于标定房间的w2c位置;
   1. 双目相机标定结果: cam0_cam1.yaml cam0到cam1外参;
   1. 四目环视相机外参标定: cam0_cam1.yaml, cam0_cam2.yaml, cam0_cam3.yaml；



### 五、标定相机-IMU外参
**单目-IMU**的标定: 默认使用3组标定数据, 最后标定结果求均值然后输出到 output_folder/camerax_to_imu.yml
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
每一组标定数据的标定结果文件及pdf存放在 /dataset_folder/data0/cam-imu/kalibr/result 目录


### 六、标定多雷达外参
标定多雷达（目前只有LiDAR0 -LiDAR1）之间外参，默认使用至少3组数据，而且需要提供两个雷达之间的外参初值，初值从CAD图纸计算得到;
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
输出的结果是3次标定结果的均值, 存放在 output_folder/lidar0_to_lidar1.yml
​

### 七、标定雷达-相机外参
     因为背包设备的结构问题，LiDAR0 很难能够在标定房间找到一个合适的摆放位置，使LiDAR0的点云能够尽可能多的打到4面墙和一个地面，而LiDAR1可以很容易的打到这5面墙。因此我们选择去标定LiDAR1-camera0.
在标定LiDAR1_to_camera0之前，必须先完成多相机外参标定这一步(camera0相对于标定房间的外参, 存放在/data0/cams/multicam/cam0_w2c.txt, 其他2组数据同样是类似路径)！另外，本标定过程需要提供LiDAR1_to_camera0的外参初值.
```bash
python run_lidar_cam_calib.py ws_folder \
                                dataset_folder \
                                output_folder \
                                init_T_yaml \ # T_l1_cam0的外参初值文件
```
