#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.service import Service
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_euler
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from aruco_nav.srv import GoToMarker
from aruco_nav.msg import NavigationFeedback
import numpy as np
from math import atan2

class ArUcoNavigation(Node):
    def __init__(self):
        super().__init__('aruco_navigation')

        # Parameters
        self.declare_parameter('marker_size', 0.2)  # Size of ArUco marker (in meters)
        self.declare_parameter('aruco_dictionary', 4) #id of aruco dictionary
        self.declare_parameter('camera_frame', "camera_link") # Frame id of the camera
        self.declare_parameter('goal_offset_x', 0.5) # distance from the marker in the x axis (in meters)
        self.declare_parameter('goal_offset_y', 0.0) # distance from the marker in the y axis (in meters)
        self.marker_size = self.get_parameter('marker_size').value
        self.aruco_dict_id = self.get_parameter('aruco_dictionary').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.goal_offset_x = self.get_parameter('goal_offset_x').value
        self.goal_offset_y = self.get_parameter('goal_offset_y').value

        # Setup
        self.bridge = CvBridge()
        self.navigator = BasicNavigator()
        self.aruco_dict = aruco.getPredefinedDictionary(self.aruco_dict_id)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self.image_callback,
            10
        )
        self.saved_marker_positions = {}  # Stores detected marker positions {marker_id: PoseStamped}
        self.is_navigating = False
        self.current_goal = None
        self.navigation_feedback_publisher = self.create_publisher(NavigationFeedback, 'navigation_feedback', 10)
        self.goto_marker_service = self.create_service(GoToMarker, 'go_to_marker', self.goto_marker_callback)

    def get_robot_frame_from_camera_frame(self, camera_pose):
         # Transform from camera to robot base frame
        # For now, we're assuming the transformation is static
        # TODO: get it from tf2 transform
        robot_pose = PoseStamped()
        robot_pose.header.frame_id = "base_link"
        robot_pose.pose.position.x = camera_pose.pose.position.x
        robot_pose.pose.position.y = camera_pose.pose.position.y
        robot_pose.pose.position.z = camera_pose.pose.position.z
        robot_pose.pose.orientation = camera_pose.pose.orientation
        return robot_pose


    def image_callback(self, msg):
         try:
            # Convert compressed image to OpenCV image
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect ArUco markers
            corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)
            
            if ids is not None and len(ids) > 0:
                for i, marker_id in enumerate(ids):
                    marker_id = marker_id[0]

                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs)
                    
                    
                    # Convert pose to PoseStamped msg in camera frame
                    marker_pose = PoseStamped()
                    marker_pose.header.frame_id = self.camera_frame
                    marker_pose.header.stamp = self.get_clock().now().to_msg()
                    marker_pose.pose.position.x = tvec[0][0][0]
                    marker_pose.pose.position.y = tvec[0][0][1]
                    marker_pose.pose.position.z = tvec[0][0][2]
                    
                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    rot_matrix_3x3 = np.array(rot_matrix).reshape(3,3)
                    x_axis = rot_matrix_3x3[:,0]
                    z_axis = rot_matrix_3x3[:,2]
                    x_dir = [1,0,0] # Marker's x axis in marker's frame
                    z_dir = [0,0,1] # marker's z axis in marker's frame

                    yaw_angle = atan2(np.dot(x_dir,np.cross(z_axis,x_axis)),np.dot(x_dir,x_axis))

                    q = quaternion_from_euler(0,0,yaw_angle)
                    
                    marker_pose.pose.orientation.x = q[0]
                    marker_pose.pose.orientation.y = q[1]
                    marker_pose.pose.orientation.z = q[2]
                    marker_pose.pose.orientation.w = q[3]
                    
                    #Create goal pose
                    robot_marker_pose = self.get_robot_frame_from_camera_frame(marker_pose)

                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = robot_marker_pose.header.frame_id
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.pose.position.x = robot_marker_pose.pose.position.x + self.goal_offset_x
                    goal_pose.pose.position.y = robot_marker_pose.pose.position.y + self.goal_offset_y
                    goal_pose.pose.position.z = robot_marker_pose.pose.position.z
                    goal_pose.pose.orientation = robot_marker_pose.pose.orientation

                    if marker_id not in self.saved_marker_positions:
                       self.get_logger().info(f"New marker detected with id:{marker_id}. saving position.")
                       self.saved_marker_positions[marker_id] = goal_pose
                    elif (not self.is_navigating) or (self.current_goal is None) or (not self.compare_poses(goal_pose, self.current_goal)):
                        self.current_goal = goal_pose
                        self.navigate_to_goal(goal_pose)

            else:
                if self.is_navigating:
                    self.get_logger().info(f"Lost marker during navigation, cancel navigation")
                    self.navigator.cancelTask()
                    self.is_navigating = False
         except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")

    def compare_poses(self, pose1, pose2):
         # Compare poses considering only position and orientation to avoid timestamp problems
        position_tolerance = 0.01
        orientation_tolerance = 0.01

        position_diff = np.sqrt(
            (pose1.pose.position.x - pose2.pose.position.x)**2 +
            (pose1.pose.position.y - pose2.pose.position.y)**2 +
            (pose1.pose.position.z - pose2.pose.position.z)**2
        )

        orientation_diff = np.sqrt(
            (pose1.pose.orientation.x - pose2.pose.orientation.x)**2 +
            (pose1.pose.orientation.y - pose2.pose.orientation.y)**2 +
            (pose1.pose.orientation.z - pose2.pose.orientation.z)**2 +
            (pose1.pose.orientation.w - pose2.pose.orientation.w)**2
        )

        return position_diff < position_tolerance and orientation_diff < orientation_tolerance

    def navigate_to_goal(self, goal):
        self.is_navigating = True
        self.get_logger().info(f"Navigating to: {goal.pose.position.x}, {goal.pose.position.y}")
        self.navigator.goToPose(goal)
        feedback_msg = NavigationFeedback()
        while not self.navigator.isTaskComplete():
             feedback = self.navigator.getFeedback()
             if feedback and feedback.navigation_time > 30.0:
                self.get_logger().info(f"Navigation timeout, canceling...")
                self.navigator.cancelTask()
             if not self.is_navigating:
                self.get_logger().info(f"Navigation canceled")
                feedback_msg.status = "canceled"
                self.navigation_feedback_publisher.publish(feedback_msg)
                break
        
        result = self.navigator.getResult()
        feedback_msg = NavigationFeedback()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal reached successfully!')
            self.current_goal = None
            self.is_navigating = False
            feedback_msg.status = "success"
            self.navigation_feedback_publisher.publish(feedback_msg)
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Navigation cancelled.')
            self.is_navigating = False
            feedback_msg.status = "canceled"
            self.navigation_feedback_publisher.publish(feedback_msg)
        elif result == TaskResult.FAILED:
            self.get_logger().info('Navigation failed!')
            self.is_navigating = False
            feedback_msg.status = "failed"
            self.navigation_feedback_publisher.publish(feedback_msg)
    
    def goto_marker_callback(self, request, response):
        marker_id = request.marker_id
        if marker_id in self.saved_marker_positions:
            goal = self.saved_marker_positions[marker_id]
            self.get_logger().info(f"Received request to navigate to saved marker {marker_id}")
            self.navigate_to_goal(goal)
            response.success = True
            response.message = f"Navigating to saved marker {marker_id}"
            return response
        else:
            response.success = False
            response.message = f"Marker ID {marker_id} not found in saved positions."
            self.get_logger().warn(f"Marker ID {marker_id} not found in saved positions.")
            return response

    def get_camera_info(self):
         # get camera info once and use it for pose estimation
         # this assumes that the camera info is published to /camera/camera/camera_info
         try:
            camera_info_topic = '/camera/camera/camera_info'
            camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 1)
            while not hasattr(self, 'camera_matrix'):
                self.get_logger().info("Waiting for camera info...")
                rclpy.spin_once(self, timeout_sec=1.0)
            camera_info_sub.destroy()
         except Exception as e:
             self.get_logger().error(f"Error retrieving camera info: {e}")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Got camera info.")
        
def main(args=None):
    rclpy.init(args=args)
    aruco_nav = ArUcoNavigation()
    aruco_nav.get_camera_info()
    rclpy.spin(aruco_nav)
    aruco_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()