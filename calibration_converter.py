import argparse
import os
from pprint import pprint

import numpy as np

from ruamel.yaml import YAML
# import yaml


def export_cam_calibration_to_euroc_format(kalibr_cam_imu_file, root_dir):
    with open(kalibr_cam_imu_file, 'r') as infile:
        yaml = YAML(typ='safe')
        config = yaml.load(infile)

        n_cams = len(config.keys())
        for cam_idx in range(n_cams):
            with open(os.path.join(root_dir, 'cam{}/sensor.yaml'.format(cam_idx)), 'w') as outfile:
                matrix = np.array(config['cam{}'.format(cam_idx)]['T_cam_imu']).flatten().tolist()
                current_cam_config = {
                    'sensor_type': 'camera',
                    'comment': 'ZED Mini {}'.format(cam_idx),
                    'T_BS': {
                        'rows': 4,
                        'cols': 4,
                        'data': matrix
                    },
                    'rate_hz': 30,
                    'resolution': config['cam{}'.format(cam_idx)]['resolution'],
                    'camera_model': config['cam{}'.format(cam_idx)]['camera_model'],
                    'intrinsics': config['cam{}'.format(cam_idx)]['intrinsics'],
                    'distortion_model': config['cam{}'.format(cam_idx)]['distortion_model'],
                    'distortion_coeffs': config['cam{}'.format(cam_idx)]['distortion_coeffs']
                }
                print(current_cam_config)
                yaml.dump(current_cam_config, outfile)


def export_imu_calibration_to_euroc_format(imu_params_file, root_dir):
    output_file = os.path.join(root_dir, 'imu0/sensor.yaml')
    with open(imu_params_file, 'r') as imufile, open(output_file, 'w') as outfile:
        yaml = YAML(typ='safe')
        imu_params = yaml.load(imufile)

        del imu_params['rostopic']
        imu_params['rate_hz'] = imu_params['update_rate']
        del imu_params['update_rate']
        imu_params['sensor_type'] = 'imu'
        imu_params['comment'] = 'ZED Mini IMU'
        imu_params['T_BS'] = {
            'cols': 4,
            'rows': 4,
            'data': np.identity(4).flatten().tolist()
        }
        yaml.dump(imu_params, outfile)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('root_dir', help='root directory of output files')
    parser.add_argument('kalibr_cam_imu_file',
                        help='Output file from camera-IMU calibration with Kalibr')
    parser.add_argument(
        'imu_params_file', help='IMU parameters file (containing noise and random walk parameters)')
    args = parser.parse_args()

    export_cam_calibration_to_euroc_format(
        args.kalibr_cam_imu_file, args.root_dir)
    export_imu_calibration_to_euroc_format(args.imu_params_file, args.root_dir)
