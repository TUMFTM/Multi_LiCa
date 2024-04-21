import numpy as np
import open3d as o3d
from geometry_msgs.msg import Transform

from .Geometry import Rotation, TransformationMatrix, Translation


class Lidar:
    """
    Represents a LiDAR sensor with associated point cloud data and transformation matrices.
    """

    def __init__(self, name: str, translation: Translation, rotation: Rotation):
        """
        Initialize a Lidar object.

        Args:
            name: The name of the Lidar sensor.
            translation: A Translation object representing the translation component of the sensor's pose.
            rotation: A Rotation object representing the rotation component of the sensor's pose.

        Returns:
            None
        """
        self.name = name
        self.translation = translation
        self.rotation = rotation
        self.tf_matrix = TransformationMatrix(translation, rotation)
        self.pcd = None
        self.pcd_transformed = None

    @classmethod
    def from_transform(cls, name: str, transform: Transform):
        """
        Create a Lidar object from a ROS Transform message.

        Args:
            name: The name of the Lidar sensor.
            transform: A ROS Transform message representing the sensor's pose.

        Returns:
            A Lidar object.
        """
        translation = Translation(
            transform.translation.x, transform.translation.y, transform.translation.z
        )
        rotation = Rotation.from_quaternion(
            [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        )
        return cls(name, translation, rotation)

    def read_pcd(self, path: str):
        """
        Read a point cloud from a file and store it in the Lidar object.

        Args:
            path: The path to the point cloud file.

        Returns:
            None
        """
        self.pcd = o3d.io.read_point_cloud(path)
        self.pcd_transformed = o3d.geometry.PointCloud(self.pcd)  # Deep copy

    def load_pcd(self, pcd):
        """
        Load a point cloud into the Lidar object.

        Args:
            pcd: An Open3D PointCloud object.

        Returns:
            None
        """
        self.pcd = o3d.geometry.PointCloud(pcd)
        self.pcd_transformed = o3d.geometry.PointCloud(self.pcd)

    def remove_ground_plane(
        self, pcd=None, distance_threshold=0.1, ransac_n=3, num_iterations=1000
    ):
        """
        Remove the ground plane from a point cloud using RANSAC.

        Args:
            pcd: An Open3D PointCloud object. If None, the point cloud of the Lidar object is used.
            distance_threshold: The distance threshold for the RANSAC algorithm.
            ransac_n: The number of points to sample for the RANSAC algorithm.
            num_iterations: The number of iterations for the RANSAC algorithm.

        Returns:
            An Open3D PointCloud object with the ground plane removed.
        """
        if pcd is None:
            pcd = self.pcd
        plane_model, inliers = pcd.segment_plane(distance_threshold, ransac_n, num_iterations)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        return outlier_cloud

    def calibrate_pitch(self, d_th=0.1, ransac_n=3, num_ransac_iter=1000, v_s=0.5, runs=5):
        """
        Calibrate the pitch angle between the LiDAR and the ground plane. This method assumes that the yaw angle is accurate and ignores it.

        Args:
            d_th: The distance threshold for the RANSAC algorithm.
            ransac_n: The number of points to sample for the RANSAC algorithm.
            num_ransac_iter: The number of iterations for the RANSAC algorithm.
            v_s: The voxel size for downsampling the point cloud. If 0, no downsampling is performed.
            runs: The number of times to run the calibration.

        Returns:
            The calibrated roll and pitch angles.
        """
        roll, pitch = np.zeros(runs), np.zeros(runs)
        for i in range(runs):
            pcd = o3d.geometry.PointCloud(self.pcd)
            if v_s > 0.0:
                pcd = pcd.voxel_down_sample(v_s)
            plane_model, inliers = pcd.segment_plane(
                distance_threshold=d_th, ransac_n=ransac_n, num_iterations=num_ransac_iter
            )
            [a, b, c, d] = plane_model
            # Compute Euler angles (yaw is not applicable)
            roll[i] = np.arctan2(b, c)
            pitch[i] = -np.arctan2(a, np.sqrt(b**2 + c**2))

        return [np.median(roll), np.median(pitch)]
