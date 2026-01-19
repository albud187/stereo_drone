import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
import cv2
from sklearn.cluster import HDBSCAN

TS_RPY_POSE = "pose_RPY"
TS_POINTCLOUD = "pointcloud"

# TS_RPY_POSE = "r_0/pose_RPY"
# TS_POINTCLOUD = "r_0/pointcloud"

## plot lines to represent field of view 
# eye ball it from plot
fov_deg = 68 
half = np.deg2rad(fov_deg/2)
L = 300
# left FOV is less because part of it cannot be seen by right camaera and stereoscopic vision needs both
# right FOV is full because right half of left camera field of view is completley visible by right camera
theta_left  = -half*0.55
theta_right = half

# Convert to (x, y) endpoints where forward is +y:
xL = L * np.sin(theta_left)
yL = L * np.cos(theta_left)
xR = L * np.sin(theta_right)
yR = L * np.cos(theta_right)


class PlotterNode(Node):
    def __init__(self)->None:
        super().__init__("plotter_node")

        self.point_array = None
        self.world_to_drone = None
        self.RPY_pose = None

        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)

        # subscribers
        self.pc_sub = self.create_subscription(PointCloud2, TS_POINTCLOUD, self.pointcloud_cb, 10)

        # clustering
        self.min_samples = 30
        self.clustering = HDBSCAN(min_cluster_size=80, min_samples=self.min_samples, cluster_selection_method="eom", metric="euclidean")

        # matplotlib code to visualize the point cloud
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.scatter_plot = self.ax.scatter([], [], s=0.15, c= "red")
        self.cluster_center_plot = self.ax.scatter([], [], s=80, c="gray")
        self.cluster_texts = []
        
        # Origin marker (drone position)
        self.origin_plot = self.ax.scatter([0], [0], c="blue", s=50)

        self.ax.set_title("Point Cloud Plot - Relative to Drone")
        self.ax.set_ylabel("longitudal axis distance (m)")
        self.ax.set_xlabel("lateral axis distance (m)")
        self.ax.set_aspect("equal")
        self.ax.grid(True)

        # hardcode simulation - based on `self.max_depth_m = 100.0` in `vision_node.py` 
        self.ax.set_xlim(-50, 50)
        self.ax.set_ylim(-5, 100)
        self.left_fov_line,  = self.ax.plot([0, xL], [0, yL], linewidth=1, c="blue")
        self.right_fov_line, = self.ax.plot([0, xR], [0, yR], linewidth=1, c="blue")

        self.cluster_id_texts = []
        self.cluster_info_box = self.ax.text(
            0.02, 0.02, "",
            transform=self.ax.transAxes,
            va="bottom", ha="left",
            fontsize=9,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8),
        )

        plt.show(block=False)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def timer_cb(self)->None:
        """
        description:
            - entrypoint in node
            - executes functions to continously update pointcloud plot
        inputs:
            - none
        outputs:
            - none
        """
        if self.point_array is None:
            return
        self.update_pointcloud_plot(self.point_array)

    def pointcloud_cb(self, msg: PointCloud2)->None:
        """
        description:
            - converts the pointcloud message into a numpy array
        inputs:
            - `msg` is pointcloud as a ros2 topic
        outpts:
            - `self.point_array` is numpy array contianing points in 3D space with positions relative to the drone
        """

        pts_iter = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        pts = np.fromiter((p for pt in pts_iter for p in pt), dtype=np.float32)

        # guard against empty clouds
        if pts.size == 0:
            self.point_array = np.empty((0, 3))
            return

        self.point_array = pts.reshape(-1, 3)

    def update_pointcloud_plot(self, point_array: np.ndarray)->None:
        """
        description:
            - updates the pointcloud plot with data from point_array
                - flips x and y coordinates of point_array for ease visualization
                - updates plot with new point cloud
                - updates plot with new cluster centers (distance and size)
        inputs:
            - `point_array` is a numpy array representation of the pointcloud published by `vision_node`
        outputs:
            - None
        """

        if point_array is None or point_array.shape[0] == 0:
            self.scatter_plot.set_offsets(np.empty((0, 2)))
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            plt.pause(0.001)
            return
    
        x = point_array[:, 0]
        y = point_array[:, 1]
        z = point_array[:, 2]

        arr_for_plot = np.column_stack((-y, x, z))
   
        object_clusters = self.detect_object_clusters(arr_for_plot)
        cluster_ids = object_clusters["cluster_ids"]
        cluster_distances = object_clusters["cluster_distnaces"]
        cluster_sizes = object_clusters["cluster_sizes"]

        cluster_centers = object_clusters["cluster_centers"]
        self.scatter_plot.set_offsets(arr_for_plot[:, :2])
     
        # update cluster centers on plot
        if cluster_centers.size:
            self.cluster_center_plot.set_offsets(cluster_centers[:, :2])
        else:
            self.cluster_center_plot.set_offsets(np.empty((0, 2)))
        
        for t in self.cluster_texts:
            t.remove()
        self.cluster_texts = []

        # remove old ID labels
        for t in self.cluster_id_texts:
            t.remove()
        self.cluster_id_texts = []

        # add new ID labels
        for i in range(cluster_centers.shape[0]):
            cx, cy = cluster_centers[i, 0], cluster_centers[i, 1]
            self.cluster_id_texts.append(
                self.ax.text(cx + 0.8, cy + 0.8, 
                    f"{int(cluster_ids[i])}",
                    fontsize=10,
                    color="black",
                )
            )
        if cluster_centers.size == 0:
            self.cluster_info_box.set_text("")
        else:
            lines = ["ID |  dist  | size"]
            for i in range(cluster_centers.shape[0]):
                lines.append(f"{int(cluster_ids[i])} | {cluster_distances[i]:5.1f} | {cluster_sizes[i]:5.1f}")
            self.cluster_info_box.set_text("\n".join(lines))

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def detect_object_clusters(self, point_array: np.ndarray)->dict[str,np.ndarray]:
        """
        description:
            - takes in a point_array (point cloud in numpy array form) and applies clustering. Identified clusters correspond to detected objects.
            - The position (cluster center, distance, direction) as well as extent (boundaries) and sizes are also calculated

        inputs:
            - `point_array` - numpy array conversion of subscribed pointcloud
        outputs:
            - `result` - dictionary with:
                - `cluster_centers` - center of each cluster in 2D space - for plotting
                - `cluster_distances` - relative distance from drone ((0,0) since plot is already relative to drone) to cluster_center
                - `cluster_angles` - direction to `cluster_center`
                - `cluster_extents` - min_x, min_y, max_x, max_y in cluster, to calculate its approximate size
                - `cluster_sizes` - distance between (min_x, min_y) to (max_x, max_y) in a cluster
        """

        empty_result = {"cluster_ids": np.empty((0,)), "cluster_centers": np.empty((0,3)), "cluster_distnaces": np.empty((0,)), 
                        "cluster_angles": np.empty((0,)), "cluster_extents": np.empty((0,4)), "cluster_sizes":np.empty((0,))}
        if point_array is None or point_array.shape[0] < self.min_samples:
            return empty_result
        
        # apply clustering algorithm and get cluster labels to use for mask to associate points to individual clusters
        self.clustering.fit(point_array)
        cluster_labels = self.clustering.labels_
    
        cluster_ids = np.array(sorted([k for k in set(cluster_labels) if k != -1]))

        cluster_centers = np.array([point_array[cluster_labels == k].mean(axis=0) for k in set(cluster_labels) if k != -1])
        if cluster_centers.size == 0:
            return empty_result

        cluster_distances = np.linalg.norm(cluster_centers, axis = 1)
        cluster_angles = np.arctan2(cluster_centers[:,1], cluster_centers[:,0])

        # cluster extents in min_x, min_y, max_x, max_y
        cluster_boundaries = np.array([
            [pts[:,0].min(), pts[:,1].min(), pts[:,0].max(), pts[:,1].max()]
            for k in set(cluster_labels) if k != -1 for pts in [point_array[cluster_labels == k]]
        ])
        cluster_sizes = np.array([self.calculate_cluster_size(b) for b in cluster_boundaries])

        result = {"cluster_ids": cluster_ids, "cluster_centers":cluster_centers, "cluster_distnaces": cluster_distances, 
                  "cluster_angles":cluster_angles, "cluster_extents":cluster_boundaries, "cluster_sizes":cluster_sizes}

        return result
    
    def calculate_cluster_size(self, cluster_border: np.ndarray) -> float:
        """
        description:
            - calculates the size of a cluster from distance between (min_x, min_y) to (max_x, max_y)
        inputs:
            -`cluster_border is array of size (4,), representing boundaries of cluster
            - indices 0 and 1 must be min_x, min_y and indices 2 and 3 must be max_x, max_y
        outputs:
            - `result` is distance
        """
            
        dx = cluster_border[2] - cluster_border[0]  # max_x - min_x
        dy = cluster_border[3] - cluster_border[1]  # max_y - min_y
        result = np.sqrt(dx**2 + dy**2)
        return result
    

def main(args=None):
    rclpy.init(args=args)
    node = PlotterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
