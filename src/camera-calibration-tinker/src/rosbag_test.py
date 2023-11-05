#!/usr/bin/env python3
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import cv2
from cv_bridge import CvBridge
import os
from sensor_msgs.msg import Image
bridge = CvBridge()

for i in range(1,7):  #有6个bag文件
    bag_file_name = 'rosbag2_%d.bag' % i
    rgb_path = '/home/zzt/tk23_calibration/src/camera-calibration-tinker/process_bag/%d/' % i  # 已经建立好的存储rgb彩色图文件的目录
    # print(bag_file_name)
    with Reader(os.path.join('/home/zzt/tk23_calibration/src/camera-calibration-tinker/',bag_file_name)) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic=='/camera/color/image_raw':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                print(msg.header.frame_id)
                # 由于topic是被压缩的，所以采用compressed_imgmsg_to_cv2读取
                # 如果topic是无压缩的，可以采用bridge.imgmsg_to_cv2(msg,"bgr8")
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                # image_name = timestr + ".png"  # 图像命名：时间戳.png
                image_name = str(timestamp) + ".png"
                if not os.path.exists(rgb_path):
                    os.makedirs(rgb_path)
                cv2.imwrite(os.path.join(rgb_path,image_name), cv_image)  # 保存