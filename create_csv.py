"""Script to convert a single rosbag to the folder structure compatible with EuRoC format."""

import argparse
import csv
import os
from collections import defaultdict

import numpy as np
import rosbag
from cv_bridge import CvBridge, CvBridgeError

from PIL import Image
from ruamel.yaml import YAML


def initialize_directories(root_dir, camera_topics):
    # Create root directory
    if os.path.exists(root_dir) and not os.path.isdir(root_dir):
        raise RuntimeError(
            'Cannot create root directory. The specified directory "{}" already exists.'.format(root_dir))
    # Create subdirectories for each camera and IMU
    try:
        for cam_idx, cam_name in enumerate(camera_topics):
            os.makedirs(os.path.join(root_dir, 'cam{}/data'.format(cam_idx)))
        os.makedirs(os.path.join(root_dir, 'imu0'))
    except OSError:
        pass


def export_rosbag_to_filesystem(root_dir, bagfile, camera_topics, imu_topic):
    bag = rosbag.Bag(bagfile)
    bridge = CvBridge()

    image_timestamps = defaultdict(list)
    imu_timestamps = []

    for topic, msg, t in bag.read_messages():
        if topic in camera_topics:
            # Save image
            cam_idx = camera_topics.index(topic)
            img_save_path = os.path.join(
                root_dir, 'cam{}/data/{}.png'.format(cam_idx, t))
            cv_image = bridge.imgmsg_to_cv2(msg, 'bgra8')
            Image.fromarray(cv_image).save(img_save_path)
            # Include current timestamp in the CSV file
            image_timestamps[cam_idx].append(t)
        elif topic == imu_topic:
            imu_timestamps.append((msg, t))

    export_timestamps_to_csv(root_dir, image_timestamps, imu_timestamps)


def export_timestamps_to_csv(root_dir, image_timestamps, imu_timestamps):
    # Export image timestamps
    for cam_idx, timestamps in image_timestamps.iteritems():
        with open(os.path.join(root_dir, 'cam{}'.format(cam_idx), 'data.csv'), 'wb') as camfile:
            camwriter = csv.writer(camfile, delimiter=',')
            for t in timestamps:
                camwriter.writerow([t, '{}.png'.format(t)])
    # Export IMU timestamps
    with open(os.path.join(root_dir, 'imu0', 'data.csv'), 'wb') as imufile:
        imuwriter = csv.writer(imufile, delimiter=',')
        for msg, t in imu_timestamps:
            imuwriter.writerow([t, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Convert ROS bag to EuRoC format')
    parser.add_argument('bagfile')
    parser.add_argument('root_dir', help='root directory of output files')
    parser.add_argument('--camera_topics', nargs='*', default=[
                        '/zed/left/image_rect_color', '/zed/right/image_rect_color'], help='list of camera topics')
    parser.add_argument(
        '--imu_topic', default='/zed/imu/data_raw', help='IMU topic')
    parser.add_argument('--frame_rate', type=int, default=30,
                        help='frame rate of video stream')
    args = parser.parse_args()

    # Create root directory and subdirectories for each sensor
    initialize_directories(args.root_dir, args.camera_topics)
    # Export bag file to filesystem
    export_rosbag_to_filesystem(
        args.root_dir, args.bagfile, args.camera_topics, args.imu_topic)
