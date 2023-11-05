import argparse
import csv
import json
import math
import time
from collections import namedtuple
import cv2
import numpy as np
import transforms3d as tfs
from H_R_t_xyzrxryrz_transform import \
    EulerAngle_to_R,\
    R_to_EulerAngle,\
    Rvec_to_R,\
    R_to_Rvec,\
    xyz_to_t,\
    t_to_xyz,\
    xyz_rxryrz_to_xyzrxryrz,\
    Rt_to_H,\
    H_to_Rt,\
    rotate_R


def handeye_calibration(end2base_xyzrxryrz,board2cam_xyzrxryrz):

    end2base_R , end2base_t = [], []
    board2cam_R , board2cam_t = [], []

    # end2base_xyzrxryrz转换为end2base_R和end2base_t
    for xyzrxryrz in end2base_xyzrxryrz: # xyzrxryrz为array类型
        R = EulerAngle_to_R(xyzrxryrz[3:6])
        t = xyz_to_t(xyzrxryrz[0:3])
        end2base_R.append(R)
        end2base_t.append(t)

    # board2cam_xyzrxryrz转换为board2cam_R和board2cam_t
    for xyzrxryrz in board2cam_xyzrxryrz:# xyzrxryrz为array类型
        R = Rvec_to_R(xyzrxryrz[3: 6])
        t = xyz_to_t(xyzrxryrz[0:3])
        board2cam_R.append(R)
        board2cam_t.append(t)

    # end2base转换为base2end
    base2end_R, base2end_t = [], []
    for R, t in zip(end2base_R, end2base_t):
        R_b2e = R.T
        t_b2e = -R_b2e @ t
        base2end_R.append(R_b2e)
        base2end_t.append(t_b2e)

    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_TSAI)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@TSAI@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_DANIILIDIS)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@DANIILIDIS@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_HORAUD)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@HORAUD@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_ANDREFF)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@ANDREFF@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    # 使用cv2.calibrateHandEye()函数计算cam2base_R和cam2base_t
    cam2base_R, cam2base_t = cv2.calibrateHandEye(base2end_R,
                                                  base2end_t,
                                                  board2cam_R,
                                                  board2cam_t,
                                                  method=cv2.CALIB_HAND_EYE_PARK)  # TSAI PARK DANIILIDIS HORAUD ANDREFF
    print(f'@@PARK@@\ncam2base_R:\n{cam2base_R}\ncam2base_t:\n{cam2base_t}')
    
    cam2base_H = tfs.affines.compose(np.squeeze(cam2base_t), cam2base_R, [1, 1, 1])

    return cam2base_H

# def get_camera_depth_scale(cam2base_H,distortion_coefficients):
#     """
#     说明:
#     获得深度图的比例因子,表示深度图像素值与实际距离之间的转换关系,通常以米/像素为单位。它是通过手眼标定得到的机械臂末端在相机坐标系下的变换矩阵和相机的畸变系数来计算的。

#     参数:
#     cam2base_H: cam到base的齐次变换矩阵
#     distortion_coefficients: 相机畸变矩阵

#     返回值:
#     camera_depth_scale: 深度图的比例因子

#     """
#     # 计算camera_depth_scale
#         # 从T_cam2base中取出前三行第四列的元素,即位于相机坐标系下机械臂末端的z坐标。然后对这个值求平方并求和,再开根号,得到相机与机械臂末端之间的距离(单位为米)。
#         # 从相机的畸变系数dist_coeffs中取出平均值。dist_coeffs是相机标定的结果,用于纠正畸变。这里取平均值是为了让这个比例因子更加准确地反映相机的畸变情况。
#         # 将上述两个值相除

#     camera_depth_scale = np.sqrt(np.sum(cam2base_H[:3, 3] ** 2)) / np.mean(distortion_coefficients)

#     return camera_depth_scale

def main():
    '''主函数'''
    parser = argparse.ArgumentParser(description='Initialize Params', epilog='HandEye-Calibration')
    parser.add_argument('--camera_params_path', type=str, default='../cfg/camera_params.json',help='the path of camera prams json')
    args = parser.parse_args()
    camera_params_path = args.camera_params_path

    # =====================创建一个camera params结构体实例,设置相机内参矩阵和畸变矩阵=====================
    print(f'===============读取camera params中.....==============')
    with open(camera_params_path, 'r') as f:
        params = json.load(f)
    camera_matrix = np.array(params['camera_matrix'])
    distortion_coefficients = np.array(params['distortion_coefficients'])
    Camera_Params = namedtuple('Camera_Params', ['camera_matrix', 'distortion_coefficients'])
    camera_params = Camera_Params(camera_matrix, distortion_coefficients)
    print(f'===============读取camera params成功==============')

    print("========读取end2base_xyzrxryrz和board2cam_xyzrxryrz测试手眼标定获得cam2base_H=========")
    with open('../output/end2base_xyzrxryrz.csv', newline='') as csvfile:
        data_reader = csv.reader(csvfile, delimiter=',')
        end2base_xyzrxryrz = [list(map(float, row)) for row in data_reader]
    end2base_xyzrxryrz = np.array(end2base_xyzrxryrz)

    with open('../output/board2cam_xyzrxryrz.csv', newline='') as csvfile:
        data_reader = csv.reader(csvfile, delimiter=',')
        board2cam_xyzrxryrz = [list(map(float, row)) for row in data_reader]
    board2cam_xyzrxryrz = np.array(board2cam_xyzrxryrz)

    # =====================计算相机到机械臂基地的变换矩阵cam2base_H=====================
    print(f'===============正在计算cam2base_H......===============')
    cam2base_H = handeye_calibration(end2base_xyzrxryrz, board2cam_xyzrxryrz)
    print(f'===============计算cam2base_H成功===============')

    # # =====================获得camera_depth_scale=====================
    # print(f'===============正在计算camera_depth_scale......===============')
    # camera_depth_scale = get_camera_depth_scale(cam2base_H,camera_params.distortion_coefficients)
    # print(f'===============计算camera_depth_scale成功===============')

    # ====================保存cam2base_H和camera_depth_scale================
    print(f'===============正在保存cam2base_H和camera_depth_scale......===============')
    with open('../output/cam2base_H.csv', 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(cam2base_H)
    # with open("./output/camera_depth_scale.txt", "w") as f:
    #     f.write(str(camera_depth_scale))
    #     f.close()
    print(f'===============保存成功============')

    # =====================最后结果输出=====================
    # print(f'@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 最后结果 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
    # print("camera_depth_scale:\n{}".format(camera_depth_scale))
    # print("cam2base_H:\n{}".format(cam2base_H))

   

if __name__ == '__main__':
    main()