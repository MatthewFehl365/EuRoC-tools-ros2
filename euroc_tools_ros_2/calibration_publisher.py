#! /use/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import numpy as np
import yaml as yml
import argparse as argp

# this script reads in a calibration file containing all the camera calibration parameters from a EuRoC dataset (in yaml format)
# and pubslished the parameters on the camera_info topic and the transforms on the tf_static topic

class Calibration(Node) :
    def __init__(self, calibration_file):
        super().__init__('calibration_publisher')
        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info_0', latching_qos)
        self.camera_info_pub_2 = self.create_publisher(CameraInfo, 'camera_info_1', latching_qos)
        self.tf_pub = self.create_publisher(TFMessage, 'tf_static', latching_qos)

        self.calibration_file = calibration_file
        self.read_calibration_file()
        self.publish_calibration()
        
    def read_calibration_file(self):
        # read in the calibration file
        with open(self.calibration_file, 'r') as file:
            self.calibration = yml.load(file, Loader=yml.FullLoader)
            
            # parse the calibration file
            self.camera_info = CameraInfo()
            self.camera_info_2 = CameraInfo()
            self.tf_message = TFMessage()
            
            # fill in the camera_info message
            self.camera_info.header.frame_id = 'cam0'
            self.camera_info.height = self.calibration['camera_0']['resolution'][1]
            self.camera_info.width = self.calibration['camera_0']['resolution'][0]
            self.camera_info.distortion_model = self.calibration['camera_0']['distortion_model']
            self.camera_info.d = self.calibration['camera_0']['distortion_coefficients']
            self.camera_info.k = np.array([self.calibration['camera_0']['intrinsics'][0], 0, self.calibration['camera_0']['intrinsics'][2],
                            0, self.calibration['camera_0']['intrinsics'][1], self.calibration['camera_0']['intrinsics'][3],
                            0, 0, 1])# is this right?
            self.camera_info.p = np.array([self.calibration['camera_0']['intrinsics'][0], 0, self.calibration['camera_0']['intrinsics'][2], 0,
                            0, self.calibration['camera_0']['intrinsics'][1], self.calibration['camera_0']['intrinsics'][3], 0,
                            0, 0, 1, 0])# is this right?
            
            self.camera_info_2.header.frame_id = 'cam1'
            self.camera_info_2.height = self.calibration['camera_1']['resolution'][1]
            self.camera_info_2.width = self.calibration['camera_1']['resolution'][0]
            self.camera_info_2.distortion_model = self.calibration['camera_1']['distortion_model']
            self.camera_info_2.d = self.calibration['camera_1']['distortion_coefficients']
            self.camera_info_2.k = np.array([self.calibration['camera_1']['intrinsics'][0], 0, self.calibration['camera_1']['intrinsics'][2],
                            0, self.calibration['camera_1']['intrinsics'][1], self.calibration['camera_1']['intrinsics'][3],
                            0, 0, 1]) # is this right?
            self.camera_info_2.p = np.array([self.calibration['camera_1']['intrinsics'][0], 0, self.calibration['camera_1']['intrinsics'][2], 0,
                            0, self.calibration['camera_1']['intrinsics'][1], self.calibration['camera_1']['intrinsics'][3], 0,
                            0, 0, 1, 0]) # is this right?
            
            # fill in the tf_message
            self.tf_message.transforms = []
            
            ## camera 0 || camera wrt body
            # read full transform into a 4x4 matrix
            T_BS = np.array(self.calibration['camera_0']['T_BS']['data']).reshape(4, 4)
            transform = TransformStamped()
            transform.header.frame_id = 'body'
            transform.child_frame_id = 'cam0'
            transform.transform.translation.x = T_BS[0, 3]
            transform.transform.translation.y = T_BS[1, 3]
            transform.transform.translation.z = T_BS[2, 3]
            # DCM to quaternion
            q = np.zeros(4)
            q[0] = np.sqrt(1 + T_BS[0, 0] + T_BS[1, 1] + T_BS[2, 2]) / 2
            q[1] = (T_BS[2, 1] - T_BS[1, 2]) / (4 * q[0])
            q[2] = (T_BS[0, 2] - T_BS[2, 0]) / (4 * q[0])
            q[3] = (T_BS[1, 0] - T_BS[0, 1]) / (4 * q[0])
            transform.transform.rotation.x = q[1]
            transform.transform.rotation.y = q[2]
            transform.transform.rotation.z = q[3]
            transform.transform.rotation.w = q[0]
            self.tf_message.transforms.append(transform)
            
            ## camera 1 || camera wrt body
            # read full transform into a 4x4 matrix
            T_BS = np.array(self.calibration['camera_1']['T_BS']['data']).reshape(4, 4)
            transform = TransformStamped()
            transform.header.frame_id = 'body'
            transform.child_frame_id = 'cam1'
            transform.transform.translation.x = T_BS[0, 3]
            transform.transform.translation.y = T_BS[1, 3]
            transform.transform.translation.z = T_BS[2, 3]
            # DCM to quaternion
            q = np.zeros(4)
            q[0] = np.sqrt(1 + T_BS[0, 0] + T_BS[1, 1] + T_BS[2, 2]) / 2
            q[1] = (T_BS[2, 1] - T_BS[1, 2]) / (4 * q[0])
            q[2] = (T_BS[0, 2] - T_BS[2, 0]) / (4 * q[0])
            q[3] = (T_BS[1, 0] - T_BS[0, 1]) / (4 * q[0])
            transform.transform.rotation.x = q[1]
            transform.transform.rotation.y = q[2]
            transform.transform.rotation.z = q[3]
            transform.transform.rotation.w = q[0]
            self.tf_message.transforms.append(transform)
            
            ## imu || imu wrt body
            # read full transform into a 4x4 matrix
            T_BS = np.array(self.calibration['imu']['T_BS']['data']).reshape(4, 4)
            transform = TransformStamped()
            transform.header.frame_id = 'body'
            transform.child_frame_id = 'imu0'
            transform.transform.translation.x = T_BS[0, 3]
            transform.transform.translation.y = T_BS[1, 3]
            transform.transform.translation.z = T_BS[2, 3]
            # DCM to quaternion
            q = np.zeros(4)
            q[0] = np.sqrt(1 + T_BS[0, 0] + T_BS[1, 1] + T_BS[2, 2]) / 2
            q[1] = (T_BS[2, 1] - T_BS[1, 2]) / (4 * q[0])
            q[2] = (T_BS[0, 2] - T_BS[2, 0]) / (4 * q[0])
            q[3] = (T_BS[1, 0] - T_BS[0, 1]) / (4 * q[0])
            transform.transform.rotation.x = q[1]
            transform.transform.rotation.y = q[2]
            transform.transform.rotation.z = q[3]
            transform.transform.rotation.w = q[0]
            self.tf_message.transforms.append(transform)
            
            ## vicon || vicon wrt body
            # read full transform into a 4x4 matrix
            T_BS = np.array(self.calibration['vicon']['T_BS']['data']).reshape(4, 4)
            transform = TransformStamped()
            transform.header.frame_id = 'vicon/firefly_sbx/firefly_sbx'
            transform.child_frame_id = 'body'
            transform.transform.translation.x = T_BS[0, 3]
            transform.transform.translation.y = T_BS[1, 3]
            transform.transform.translation.z = T_BS[2, 3]
            # DCM to quaternion
            q = np.zeros(4)
            q[0] = np.sqrt(1 + T_BS[0, 0] + T_BS[1, 1] + T_BS[2, 2]) / 2
            q[1] = (T_BS[2, 1] - T_BS[1, 2]) / (4 * q[0])
            q[2] = (T_BS[0, 2] - T_BS[2, 0]) / (4 * q[0])
            q[3] = (T_BS[1, 0] - T_BS[0, 1]) / (4 * q[0])
            transform.transform.rotation.x = q[1]
            transform.transform.rotation.y = q[2]
            transform.transform.rotation.z = q[3]
            transform.transform.rotation.w = q[0]
            self.tf_message.transforms.append(transform)
            
            # print the camera info
            print(self.camera_info)
            
    def publish_calibration(self):
        self.camera_info_pub.publish(self.camera_info)
        self.camera_info_pub_2.publish(self.camera_info_2)
        self.tf_pub.publish(self.tf_message)
        
def main(args=None):
    
    parser = argp.ArgumentParser(description='Calibration Publisher')
    parser.add_argument('calibration_file', type=str, help='Path to the calibration YAML file')
    calirbation_args = parser.parse_args()
    
    rclpy.init(args=args)
    calibration_publisher = Calibration(calirbation_args.calibration_file)
    rclpy.spin(calibration_publisher)
    calibration_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()