import sys
import os
import csv


if __name__ == "__main__":
    imu_file = "/media/ziqianbai/DATA/3D_Capture/20191230-041944-029b322c4f/ali_data/mobile_align/20191230-041944-029b322c4f_spot_meta.txt"

    imu_fd = open(imu_file, 'r')
    imu_datas = imu_fd.readlines()

    imu_datas = [d.strip() for d in imu_datas if d.split() != '']

    print("before sort:")
    for i in imu_datas:
        print(i)

    imu_fd.close()

    imu_datas.sort(key=lambda x: x.split()[0], reverse=True)
    print("after sort:")
    for i in imu_datas:
        print(i)

    new_imu_file = "/media/ziqianbai/DATA/3D_Capture/20191230-041944-029b322c4f/ali_data/mobile_align/20191230-041944-029b322c4f_spot_meta_new.txt"
    with open(new_imu_file, 'w') as fd:
        for new_data in imu_datas:
            fd.write(new_data)
            fd.write('\n')
