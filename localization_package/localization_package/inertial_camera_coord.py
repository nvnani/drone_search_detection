#!/usr/bin/env python3

import rclpy
import numpy as np
import math
from rclpy.node import Node
from voxl_msgs.msg import Aidetection
from std_msgs.msg import Float32, Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class ImageData(Node):
    def __init__(self):
        super().__init__('coordinate_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10,)
        self.publisher_data = self.create_publisher(PoseStamped,
                                               '/object_center',10)
        self.camera_coordinate_data = self.create_publisher(PoseStamped,
                                                     '/object/pose_camera',10)
        self.local_data = self.create_subscription(VehicleLocalPosition,
                            '/fmu/out/vehicle_local_position', self.local_cb, qos_profile)
        self.subscription_data = self.create_subscription(Aidetection,
                            '/tflite_data', self.listener_cb, qos_profile)
        self.attitude_data = self.create_subscription(VehicleAttitude,
                        'fmu/out/vehicle_attitude', self.attitude_cb, qos_profile)
        self.subscription_data
        # self.label_boundary = Aidetection()
        self._attitude = VehicleAttitude()
        self._position = VehicleLocalPosition()
        self.x = 0.0
        self.y = 0.0
        self._z_inertial = 0.0
        self._scaling_factor = 0.0
        
    def listener_cb(self,msg: Aidetection):
        """Callback function for the /tflite_data subscriber

        Args:
            msg (Aidetection): Containes frame_id and bounding box information
        """
        # self.label_boundary = msg
        center = PoseStamped()
        camera_coord = PoseStamped()
        inertial_coord = PoseStamped()
        # y_center = Float32MultiArray()
        # Calculates the centerpoint of the bounding box
        x_center = msg.x_min + (msg.x_max - msg.x_min)/2
        y_center = msg.y_min + (msg.y_max - msg.y_min)/2
        x_camera, y_camera = self.calculate_camera_coords(x_center, y_center)
        # x_inertial, y_inertial, z_inertial = self.calculate_inertial_coords(x_camera, y_camera, self._scaling_factor)
        # fc.header.stamp= self.get_clock().now().nanoseconds
        # Camera coordinates
        camera_coord.header.frame_id = msg.class_name
        camera_coord.pose.position.x = x_camera
        camera_coord.pose.position.y = y_camera
        camera_coord.pose.position.z = self._scaling_factor
        center.header.frame_id = msg.class_name
        center.pose.position.x = x_center
        center.pose.position.y = y_center
        # self.get_logger().info(f"The center point of the object is:({center.data[0]},{center.data[1]})")
        # Publish data when the object is detected
        if msg.class_name == 'handbag' or msg.class_name == 'backpack':
            # Publish camera coordinates 
            self.get_logger().info("Detectd handbag/backpack")
            self.publisher_data.publish(center)
            self.camera_coordinate_data.publish(camera_coord)
            # Publish inertail coordinates
            # self.inertial_coordinate_data.publish(inertial_coord)
        
    def local_cb (self, msg: VehicleLocalPosition):
        """Callback function for /fmu/out/vehicle_local_position

        Args:
            msg (VehicleLocalPosition): Contains position and heading of the vehicle
        """
        self._position = msg
        z_inertial = msg.z
        self._scaling_factor = z_inertial/(math.cos(45))
        
    def attitude_cb(self, msg: VehicleAttitude):
        """Callback function for /fmu/out/vehicle_attitude

        Args:
            msg (VehicleAttitude): Contains the heading of the vehicle
        """
        self._attitude = msg
            
    def calculate_camera_coords (self, x, y):
        pixel_matrix = np.array([[x],
                                [y],
                                [1]])
        I_00 = 2.797206*1e2
        I_01 = 0.0
        I_02 = 3.545686*1e2
        # I_03 = 0.0
        I_10 = 0.0
        I_11 = 2.797774*1e2
        I_12 = 2.183704*1e2
        # I_13 = 0.0
        I_20 = 0.0
        I_21 = 0.0
        I_22 = 1.0
        # I_23 = 0.0
        
        intrinsic = np.array([[I_00, I_01, I_02],
                             [I_10, I_11, I_12],
                             [I_20, I_21, I_22]])  
        
        inv_intrinsic = np.linalg.inv(intrinsic)
        coord = np.dot(inv_intrinsic,pixel_matrix)
        scale = self._scaling_factor/coord[2][0]
        # scaled_matrix = self._scaling_factor*coord
        x_ = scale*coord[0][0]
        y_ = scale*coord[1][0]
        
        return x_, y_
    
    # def calculate_inertial_coords (self, x_object, y_object, z_object):
    #     xp = self._position.x
    #     yp = self._position.y
    #     zp = self._position.z
        
    #     # EIGEN's convention is to have the SCALAR value FIRST!!!
    #     quat_lpp = np.quaternion(self._attitude.q[3], self._attitude.q[0], self._attitude.q[1], self._attitude.q[2])
    #     R_lpp = np.array(quat_lpp.rotation_matrix)

    #     # Populating a homogeneous transformation matrix with /mavros/local_position/pose data
    #     # converting quaternion to rotation matrix
    #     H_lpp = np.eye(4)
    #     H_lpp[:3, :3] = R_lpp
    #     H_lpp[:3, 3] = [xp, yp, zp]
        
    #     H_M_B = np.array([
    #         [0, -1, 0, 0],  
    #         [-0.707, 0, 0.707, 0],
    #         [0.707, 0, 0.707, 0],
    #         [0, 0, 0, 1]
    #         ])
    #     object_cc = np.array([[x_object],
    #                             [y_object],
    #                             [z_object],
    #                             [1]])

        
    #     H_M_B_inverse = np.linalg.inv(H_M_B)
    #     # Camera to body
    #     object_body = np.dot(H_M_B_inverse,object_cc)  
    #     # Body to inertial frame
    #     object_inertial = np.dot(H_lpp,object_body)     
        
    #     return object_inertial[0][0], object_inertial[1][0], object_inertial[2][0] 
        
def main(args = None):
    rclpy.init(args=args)
    object = ImageData()
    try:
        rclpy.spin(object)
    except KeyboardInterrupt:
        object.get_logger().info("Keyboard interrupt activated")
    finally:
        object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
