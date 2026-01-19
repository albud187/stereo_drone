import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

import matplotlib.pyplot as plt

import numpy as np
import cv2


# TS_RPY_POSE = "/r_0/pose_RPY"
# TS_IMG_LEFT = "/r_0/left/image_raw"
# TS_IMG_RIGHT = "/r_0/right/image_raw"

# TS_INFO_LEFT = "/r_0/left/camera_info"
# TS_INFO_RIGHT = "/r_0/right/camera_info"

# TP_POINTCLOUD = "/r_0/pointcloud"

TS_RPY_POSE = "pose_RPY"
TS_IMG_LEFT = "left/image_raw"
TS_IMG_RIGHT = "right/image_raw"

TS_INFO_LEFT = "left/camera_info"
TS_INFO_RIGHT = "right/camera_info"

TP_POINTCLOUD = "pointcloud"

def homegeous_transformation(x: float, y: float, z: float, roll: float, pitch: float, yaw: float)->np.ndarray:
    """
    description:
        - calculates homogenous transformation matrix given position/translation and oreintation/rotation (euler angles)
    inputs:
        - position in meters `x`,`y`,`z`
        - rotation in radians `roll`, `pitch`, `yaw`
    outputs:
        - `result` homogeneous transformation matrix
    """
  
    h00 = np.cos(yaw) * np.cos(pitch)
    h01 = np.cos(yaw) * np.sin(pitch) * np.sin(roll) - np.sin(yaw) * np.cos(roll)
    h02 = np.cos(yaw) * np.sin(pitch) * np.cos(roll) + np.sin(yaw) * np.sin(roll)

    h10 = np.sin(yaw) * np.cos(pitch)
    h11 = np.sin(yaw) * np.sin(pitch) * np.sin(roll) + np.cos(yaw) * np.cos(roll)
    h12 = np.sin(yaw) * np.sin(pitch) * np.cos(roll) - np.cos(yaw) * np.sin(roll)

    h20 = -np.sin(pitch)
    h21 =  np.cos(pitch) * np.sin(roll)
    h22 =  np.cos(pitch) * np.cos(roll)

    result = np.array([
        [h00, h01, h02, x],
        [h10, h11, h12, y],
        [h20, h21, h22, z],
        [0.0, 0.0, 0.0, 1.0]])

    return result

class VisionNode(Node):
    def __init__(self)->None:
        super().__init__("vision_node")

        self.bridge = CvBridge()

        # camera images (numpy arrays)
        self.left_frame = None
        self.right_frame = None

        # camera info for rectification maps
        self.left_info = None
        self.right_info = None

        # rectification
        self.left_map1 = None
        self.left_map2 = None
        self.right_map1 = None
        self.right_map2 = None
        self.rect_ready = False

        # time stamps for camera images
        self.left_stamp = None
        self.right_stamp = None
        self.sync_time_threshold = 0.01 
        
        # camera intrinsics from left/camera_info
        # left and right cameras are the same
        # see https://github.com/albud187/sjtu-drone/blob/stereo-camera-sim/sjtu_drone_description/urdf/sjtu_drone_multi.urdf.xacro
        # for camera prorties or execute ros2 topic echo <ns>/left/camera_info
        self.fx = 701.614054597489
        self.fy = 701.614054597489
        self.cx = 480.5
        self.cy = 270.5

        # baseline_m is distance between cameras
        self.baseline_m = 0.10

        # depth map / disparity
        self.min_depth_m = 1.0
        self.max_depth_m = 100.0  
        self.num_disparities = 16*15   
        self.block_size = 21           
        self.stereo = cv2.StereoBM_create(
            numDisparities=self.num_disparities,
            blockSize=self.block_size
        )

        # pose for pointcloud generation
        self.RPY_pose = None

        # calculate and publish point clouds at a fixed rate of 30 Hz 
        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)

        cv2.namedWindow("left_camera", cv2.WINDOW_NORMAL)
        cv2.namedWindow("right_camera", cv2.WINDOW_NORMAL)
        cv2.namedWindow("left_depth_map", cv2.WINDOW_NORMAL)

        #subscribers
        self.create_subscription(Pose, TS_RPY_POSE, self.pose_RPY_cb, 10)
        self.create_subscription(Image, TS_IMG_LEFT, self.left_cb, 10)
        self.create_subscription(Image, TS_IMG_RIGHT, self.right_cb, 10)
        self.create_subscription(CameraInfo, TS_INFO_LEFT, self.left_info_cb, 10)
        self.create_subscription(CameraInfo, TS_INFO_RIGHT, self.right_info_cb, 10)

        #publishers
        self.pc_pub = self.create_publisher(PointCloud2, TP_POINTCLOUD, 10)
    
        # from sjtu drone
        self.Drone_to_LeftCameraLink = homegeous_transformation(0.2, 0.05, 0, 0, 0.25, 0)
        self.LeftCameraLink_to_LeftCameraFrame = homegeous_transformation(0.0, 0.0, 0.0, -np.pi/2, 0.0, -np.pi/2)
        
        # initialize transformation from drone to world
        self.world_to_drone = None
    
    def pose_RPY_cb(self, msg: Pose)->None:
        self.RPY_pose = msg
        self.world_to_drone = homegeous_transformation(msg.position.x, msg.position.y, msg.position.z,
                                                       msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def left_cb(self, msg: Image)->None:
        self.left_stamp = msg.header.stamp
        self.left_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def right_cb(self, msg: Image)->None:
        self.right_stamp = msg.header.stamp
        self.right_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def left_info_cb(self, msg: CameraInfo)->None:
        self.left_info = msg

    def right_info_cb(self, msg: CameraInfo)->None:
        self.right_info = msg

    def timer_cb(self)->None:
        """
        description:
            - entrypoint in node
            - executes functions to generate and publish pointcloud as well as display raw image and depth map
        inputs:
            - none
        outputs:
            - none
        """

        # do nothing if no camera image frame, or no camera info, or no timestamp
        if self.left_frame is None or self.right_frame is None:
            return
        if self.left_info is None or self.right_info is None:
            return
        if self.left_stamp is None or self.right_stamp is None:
            return

        left_timestamp = self.left_stamp.sec + self.left_stamp.nanosec * 1e-9
        right_timestamp = self.right_stamp.sec + self.right_stamp.nanosec * 1e-9
        dt = abs(left_timestamp - right_timestamp)
        # do nothing if time delta is too big (out of synch)
        if dt > self.sync_time_threshold:
            return

        # build rectification maps, do nothing if rectification mapping isnt ready
        self.build_rect_maps()
        if not self.rect_ready:
            return
        
        depth_map = self.calculate_depth_map()
        point_cloud = self.generate_point_array(depth_map, self.min_depth_m, self.max_depth_m)
        self.publish_pointcloud(point_cloud)
       
        # Visualize depth (clip far values so near objects stand out)
        depth_vis = np.clip(depth_map, 0.0, self.max_depth_m)
        depth_vis = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        cv2.imshow("left_camera", self.left_frame)
        cv2.imshow("right_camera", self.right_frame)
        cv2.imshow("left_depth_map", depth_vis)
        
        key = cv2.waitKey(1)
        if key == ord("q"):
            self.get_logger().info("q pressed: shutting down")
            rclpy.shutdown()

    def build_rect_maps(self)->None:
        """
        decription:
            - generates image rectification maps for use in calculating disparity between the camera images
                - lookup tables that tells opencv which subpixel location to sample from original image
        inputs:
            - `self.left_info` and `self.right_info` - camera info
        outputs:
            - `self.left_map1`, `self.left_map2`, `self.right_map1`, `self.right_map2`
                __map1 is primary coordinate may and _map2 is offset map
        """

        if self.rect_ready:
            return
        
        if self.left_info is None or self.right_info is None:
            return

        w = int(self.left_info.width)
        h = int(self.left_info.height)

        # left camera calibration (same is repeated for right camera - note both cameras are the same
        K1 = np.array(self.left_info.k).reshape(3, 3)
        D1 = np.array(self.left_info.d).reshape(-1, 1)
        R1 = np.array(self.left_info.r).reshape(3, 3)
        P1 = np.array(self.left_info.p).reshape(3, 4)
        # top-left 3x3 of P is the new intrinsic matrix after rectification
        K1_new = P1[:3, :3]

        K2 = np.array(self.right_info.k).reshape(3, 3)
        D2 = np.array(self.right_info.d).reshape(-1, 1)
        R2 = np.array(self.right_info.r).reshape(3, 3)
        P2 = np.array(self.right_info.p).reshape(3, 4)
        K2_new = P2[:3, :3]

        # pixel lookup maps for use with cv2.remap()
        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            K1, D1, R1, K1_new, (w, h), cv2.CV_16SC2
        )

        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            K2, D2, R2, K2_new, (w, h), cv2.CV_16SC2
        )
        
        self.rect_ready = True

    def calculate_depth_map(self)->np.ndarray:
        """
        description:
            - generates a numpy array containing distances (in 3D space) to matched pixels
            - combines information from both images, dispartiy and physical geometry to calculate 3D distance in meters for each pixel
            - uses the LEFT camera as reference camera
        inputs:
            - `left_frame` and `right_frame` are np.ndarray
        output:
            - `result` has shape (540, 960), same as size of camera image
            -  result[u,v] is the distance (in meters) of the detection at pixel coordinate (u, v) in optical frame, where positive u is right and positiv ev is down
        """

        left_frame = cv2.remap(self.left_frame, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
        right_frame = cv2.remap(self.right_frame, self.right_map1, self.right_map2, cv2.INTER_LINEAR)

        # convert to grayscale for StereoBM 
        left_frame_gray = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
        right_frame_gray = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)

        # disparity is fixed-point with 4 fractional bits (scale factor 16)
        # https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html
        disp = self.stereo.compute(left_frame_gray, right_frame_gray).astype(np.float32) / 16.0

        disp[disp < 0] = 0

        # depth in meters: Z = fx * B / disparity
        # add eps to avoid division by zero
        eps = 1e-6
        # 
        result = (self.fx * self.baseline_m) / (disp + eps)
        result[disp <= 0] = 0.0
        
        return result
  
    def generate_point_array(self, depth_map: np.ndarray, min_depth: float, max_depth: float, ratio: float = 0.05)->np.ndarray:
        """
        description:
            - filters out depth map points based on valid range of depth and desired number of points to ignore (to reduce computational demand)
            - transforms the depth map into an array of points in 3D space relative to the drone
        inputs:
            - `depth_map` is result of `calculate_depth_map`
            - `min_depth` and `max_depth` are allowable range of depths
            - `ratio` is fraction of valid points to keep (1.0 keeps all, 0.5 keeps 50%, 0.1 keeps 10%)
        outputs:
            - `result` is an Nx3 array, comprised of vectors that describe relative translation from the drone to a point in 3D space
        """
    
        if ratio <= 0.0:
            return np.empty((0, 3))
        if ratio > 1.0:
            ratio = 1.0

        #v_idx = vertical pixel coordinate (row index), u_idx = horizontal pixel coordinate (column index)
        mask = (depth_map > min_depth) & (depth_map < max_depth) & np.isfinite(depth_map)
        v_idx, u_idx = np.where(mask)
       
        # evenly remove some samples to reduce computational demand
        step = int(round(1.0 / ratio))
        if step < 1:
            step = 1

        # pixel coordinates are from indices of depth map
        # use step to stride and skip elements, to reduce computational demand
        v_idx = v_idx[::step]
        u_idx = u_idx[::step]

        # get depths based on column indices
        Z = depth_map[v_idx, u_idx]

        # point cloud in optical frame in pixel coordinates
        # uvz.shape == (N,3), N == len(u_dix) == len(v_idx) == len(Z)
        uvz = np.column_stack((u_idx, v_idx, Z))

        # get point cloud in optical frame in metric coordinates
        u = uvz[:,0]
        v = uvz[:,1]
        z = uvz[:,2]
        rx = (u - self.cx)*Z/self.fx
        ry = (v - self.cy)*Z/self.fy

        # stack into a 2D array of shape (N,3)
        xyz_opt = np.column_stack((rx, ry, z))

        # add a forth column of ones to faciliate homogeneous transformation
        xyz_h_opt = np.hstack([xyz_opt, np.ones((xyz_opt.shape[0], 1))])

        # slice to get only first 3 columns using[:, :3]
        # need to transpose xyz_h_opt to allow proper matrix multiplcation with homogenoeus transformation
        result = (self.Drone_to_LeftCameraLink @ self.LeftCameraLink_to_LeftCameraFrame @ xyz_h_opt.T).T[:, :3]

        return result

    def publish_pointcloud(self, pc_xyz: np.ndarray)->None:
        """
        description:
            - converts the numpy array of 3D points into a pointcloud that can be published as a ros2 topic
        inputs:
            - `pc_xyz` from `generate_point_array`
        outputs:
            - none
        """

        if pc_xyz is None or pc_xyz.shape[0] == 0:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        # PointCloud2 needs (x, y, z) in float32/float64
        # https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud.html
        points = pc_xyz.astype(np.float32)

        msg = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self.pc_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()