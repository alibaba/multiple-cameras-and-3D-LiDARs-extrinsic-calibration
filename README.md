# Single-Shot is Enough: Panoramic Infrastructure Based Calibration of Multiple Cameras and 3D LiDARs



## Updates

* [2021/07/21] first commit; Given sparse map of panoramic infrastructure, we can easily calibrate the extrinsics among multi-camera and multi-LiDAR. In order to facilitate camera intrinsic calibration and imu calibration, [Kalibr](https://github.com/ethz-asl/kalibr) and [imu_utils](https://github.com/gaowenliang/imu_utils) are included in our code.



## Introduction

 This paper is accpeted in 2021 IROS, the preprint version is accessible in [arXiv](https://arxiv.org/abs/2103.12941).  In this paper, we propose a single-shot solution for calibrating extrinsic transformations among multiple cameras and 3D LiDARs. We establish a panoramic infrastructure, in which a camera or LiDAR can be robustly localized using data from single frame. Experiments are conducted on three devices with different camera-LiDAR configurations, showing that our approach achieved comparable calibration accuracy with the state-of-the-art approaches but with much greater efficiency.



## Quick Start

1. Refer to [docker_useage.md](./docker_usage.md)
2. 



## Citation

If you find this code is useful in your research, please cite:

```
@article{fang2021single,
  title={Single-Shot is Enough: Panoramic Infrastructure Based Calibration of Multiple Cameras and 3D LiDARs},
  author={Fang, Chuan and Ding, Shuai and Dong, Zilong and Li, Honghua and Zhu, Siyu and Tan, Ping},
  journal={arXiv preprint arXiv:2103.12941},
  year={2021}
}
```



## Acknowledgements

Thanks to gaowenliang for opening source of his excellent works  [imu_utils](https://github.com/gaowenliang/imu_utils). Thanks to the Kalibr maintenance team of Kalibr for the well-known open-source project [Kalibr](https://github.com/ethz-asl/kalibr).