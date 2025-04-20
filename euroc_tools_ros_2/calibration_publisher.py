#! /use/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
import numpy as np
import yaml as yml
import argparse as argp

# this script reads in a calibration file containing all the camera calibration parameters from a EuRoC dataset (in yaml format)
# and pubslished the parameters on the camera_info topic and the transforms on the tf_static topic

class Calibration(Node) :
    def __init__(self, calibration_file):
        super().__init__('calibration_publisher')
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info_0', 10)
        self.camera_info_pub_2 = self.create_publisher(CameraInfo, 'camera_info_1', 10)
        self.vicon_sub = self.create_subscription(TransformStamped, '/vicon/firefly_sbx/firefly_sbx', self.vicon_callback, qos_profile=qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.publish_calibration_timer = self.create_timer(0.1, self.publish_calibration_cb)

        self.transformcam0 = TransformStamped()
        self.transformcam1 = TransformStamped()
        self.transformIMU = TransformStamped()
        self.transformViconBody = TransformStamped()

        self.camera_info = CameraInfo()
        self.camera_info_2 = CameraInfo()
        self.tf_message = TransformStamped()
        self.calibration_file = calibration_file
        self.read_calibration_file()
        
    def vicon_callback(self, msg):
        self.transformcam0.header.stamp = msg.header.stamp
        self.transformcam1.header.stamp = msg.header.stamp
        self.transformIMU.header.stamp = msg.header.stamp
        self.transformViconBody.header.stamp = msg.header.stamp
        self.tf_broadcaster.sendTransform(msg)
        self.tf_broadcaster.sendTransform(self.transformcam0)
        self.tf_broadcaster.sendTransform(self.transformcam1)
        self.tf_broadcaster.sendTransform(self.transformIMU)
        self.tf_broadcaster.sendTransform(self.transformViconBody)
        
        
    def read_calibration_file(self):
        # read in the calibration file
        with open(self.calibration_file, 'r') as file:
            self.calibration = yml.load(file, Loader=yml.FullLoader)
            
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
            
            ## camera 0 || camera wrt body
            # read full transform into a 4x4 matrix
            T_BS = np.array(self.calibration['camera_0']['T_BS']['data']).reshape(4, 4)
            self.transformcam0.header.frame_id = 'body'
            self.transformcam0.child_frame_id = 'cam0'
            self.transformcam0.transform.translation.x = T_BS[0, 3]
            self.transformcam0.transform.translation.y = T_BS[1, 3]
            self.transformcam0.transform.translation.z = T_BS[2, 3]
            # DCM to quaternion
            q = np.zeros(4)
            q[0] = np.sqrt(1 + T_BS[0, 0] + T_BS[1, 1] + T_BS[2, 2]) / 2
            q[1] = (T_BS[2, 1] - T_BS[1, 2]) / (4 * q[0])
            q[2] = (T_BS[0, 2] - T_BS[2, 0]) / (4 * q[0])
            q[3] = (T_BS[1, 0] - T_BS[0, 1]) / (4 * q[0])
            self.transformcam0.transform.rotation.x = q[1]
            self.transformcam0.transform.rotation.y = q[2]
            self.transformcam0.transform.rotation.z = q[3]
            self.transformcam0.transform.rotation.w = q[0]
            
            ## camera 1 || camera wrt body
            # read full transform into a 4x4 matrix
            T_BS = np.array(self.calibration['camera_1']['T_BS']['data']).reshape(4, 4)
            self.transformcam1.header.frame_id = 'body'
            self.transformcam1.child_frame_id = 'cam1'
            self.transformcam1.transform.translation.x = T_BS[0, 3]
            self.transformcam1.transform.translation.y = T_BS[1, 3]
            self.transformcam1.transform.translation.z = T_BS[2, 3]
            # DCM to quaternion
            q = np.zeros(4)
            q[0] = np.sqrt(1 + T_BS[0, 0] + T_BS[1, 1] + T_BS[2, 2]) / 2
            q[1] = (T_BS[2, 1] - T_BS[1, 2]) / (4 * q[0])
            q[2] = (T_BS[0, 2] - T_BS[2, 0]) / (4 * q[0])
            q[3] = (T_BS[1, 0] - T_BS[0, 1]) / (4 * q[0])
            self.transformcam1.transform.rotation.x = q[1]
            self.transformcam1.transform.rotation.y = q[2]
            self.transformcam1.transform.rotation.z = q[3]
            self.transformcam1.transform.rotation.w = q[0]
            
            ## imu || imu wrt body
            # read full transform into a 4x4 matrix
            T_BS = np.array(self.calibration['imu']['T_BS']['data']).reshape(4, 4)
            self.transformIMU.header.frame_id = 'body'
            self.transformIMU.child_frame_id = 'imu0'
            self.transformIMU.transform.translation.x = T_BS[0, 3]
            self.transformIMU.transform.translation.y = T_BS[1, 3]
            self.transformIMU.transform.translation.z = T_BS[2, 3]
            # DCM to quaternion
            q = np.zeros(4)
            q[0] = np.sqrt(1 + T_BS[0, 0] + T_BS[1, 1] + T_BS[2, 2]) / 2
            q[1] = (T_BS[2, 1] - T_BS[1, 2]) / (4 * q[0])
            q[2] = (T_BS[0, 2] - T_BS[2, 0]) / (4 * q[0])
            q[3] = (T_BS[1, 0] - T_BS[0, 1]) / (4 * q[0])
            self.transformIMU.transform.rotation.x = q[1]
            self.transformIMU.transform.rotation.y = q[2]
            self.transformIMU.transform.rotation.z = q[3]
            self.transformIMU.transform.rotation.w = q[0]
            
            ## vicon || vicon wrt body
            # read full transform into a 4x4 matrix
            T_BS = np.array(self.calibration['vicon']['T_BS']['data']).reshape(4, 4)
            self.transformViconBody.header.frame_id = 'vicon/firefly_sbx/firefly_sbx'
            self.transformViconBody.child_frame_id = 'body'
            self.transformViconBody.transform.translation.x = T_BS[0, 3]
            self.transformViconBody.transform.translation.y = T_BS[1, 3]
            self.transformViconBody.transform.translation.z = T_BS[2, 3]
            # DCM to quaternion
            q = np.zeros(4)
            q[0] = np.sqrt(1 + T_BS[0, 0] + T_BS[1, 1] + T_BS[2, 2]) / 2
            q[1] = (T_BS[2, 1] - T_BS[1, 2]) / (4 * q[0])
            q[2] = (T_BS[0, 2] - T_BS[2, 0]) / (4 * q[0])
            q[3] = (T_BS[1, 0] - T_BS[0, 1]) / (4 * q[0])
            self.transformViconBody.transform.rotation.x = q[1]
            self.transformViconBody.transform.rotation.y = q[2]
            self.transformViconBody.transform.rotation.z = q[3]
            self.transformViconBody.transform.rotation.w = q[0]
            
            # print the camera info
            print(self.camera_info)
            
    def publish_calibration_cb(self):
        self.camera_info_pub.publish(self.camera_info)
        self.camera_info_pub_2.publish(self.camera_info_2)
        
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