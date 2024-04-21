import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d

from .Geometry import Rotation, TransformationMatrix, Translation
from .Lidar import Lidar


def visualize_calibration(lidar_list: list[Lidar], transformed=True, only_paint=False):
    """
    Visualize the calibration of a list of LiDAR sensors.

    Args:
        lidar_list: A list of Lidar objects to visualize.
        transformed: If True, visualize the transformed point clouds. If False, visualize the original point clouds.
        only_paint: If true, point clouds are only painted but not transformed.
    Returns:
        None
    """
    # Define colors for the point clouds in RGB format
    colors = [
        [1, 1, 1],
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [0.7, 0, 0.9],
        [0.2, 0.8, 0.4],
        [0.5, 0.8, 0.9],
        [0.2, 0.4, 0.8],
        [0.8, 0.4, 0.2],
        [0.8, 0.6, 0.6],
    ]
    for lidar, color in zip(lidar_list, colors):
        if transformed:
            lidar.pcd_transformed.paint_uniform_color(color)
        else:
            lidar.pcd.paint_uniform_color(color)
    if only_paint:
        return

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=True)
    # Change the background color to black
    vis.get_render_option().background_color = [0, 0, 0]
    vis.get_render_option().point_size = 2

    if transformed:
        for lidar in lidar_list:
            vis.add_geometry(lidar.pcd_transformed)
    else:
        for lidar in lidar_list:
            vis.add_geometry(lidar.pcd)

    vis.run()


def modify_urdf_joint_origin(file_path: str, joint_name: str, tf_matrix: TransformationMatrix):
    """
    Modify the origin of a joint in a URDF file.

    Args:
        file_path: The path to the URDF file.
        joint_name: The name of the joint to modify.
        tf_matrix: The new transformation matrix for the joint.

    Returns:
        None
    """
    # Parse the URDF file
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(file_path, parser=parser)
    root = tree.getroot()

    # Find the joint with the given name
    for joint in root.iter("joint"):
        if "name" in joint.attrib and joint.attrib["name"] == joint_name:
            origin = joint.find("origin")
            if origin is not None:
                origin.attrib["xyz"] = tf_matrix.translation.__str__()
                origin.attrib["rpy"] = tf_matrix.rotation.__str__()
            else:
                raise Exception("joint has no origin to be modified")
    tree.write(file_path, xml_declaration=True)


class Calibration:
    """
    Handles initial and GICP calibration between source and target lidars.
    """

    def __init__(
        self,
        source: Lidar,
        target: Lidar,
        max_corresp_dist=1.0,
        epsilon=0.0001,
        rel_fitness=1e-6,
        rel_rmse=1e-6,
        max_iterations=30,
        distance_threshold=0.1,
        ransac_n=3,
        num_iterations=1000,
    ):
        """
        Initialize a Calibration object.

        Args:
            source: The source Lidar object.
            target: The target Lidar object.
            max_corresp_dist: The maximum correspondence distance for the GICP algorithm.
            epsilon: The epsilon parameter for the GICP algorithm.
            rel_fitness: The relative fitness convergence criterion for the GICP algorithm.
            rel_rmse: The relative RMSE convergence criterion for the GICP algorithm.
            max_iterations: The maximum number of iterations for the GICP algorithm.
            distance_threshold: The maximum distance for the point inlier to plane.
            ransac_n: The number of points used to define a plane.
            num_iterations: The number of iterations of RANSAC.
        """
        self.source = source
        self.target = target
        self.max_corresp_dist = max_corresp_dist
        self.epsilon = epsilon
        self.rel_fitness = rel_fitness
        self.rel_rmse = rel_rmse
        self.max_iterations = max_iterations
        self.distance_threshold = distance_threshold
        self.ransac_n = ransac_n
        self.num_iterations = num_iterations
        self.initial_transformation = self.compute_initial_transformation()
        self.calibrated_transformation = None
        self.reg_p2l = None

    def compute_initial_transformation(self) -> TransformationMatrix:
        """
        Compute the initial transformation matrix between the source and target lidars.

        Returns:
            The initial transformation matrix.
        """
        # Transform the source lidar to the basis coordinate system and
        # transform this lidar to the target coordinate system
        transformation_matrix = (
            np.linalg.inv(self.target.tf_matrix.matrix) @ self.source.tf_matrix.matrix
        )
        self.initial_transformation = TransformationMatrix.from_matrix(transformation_matrix)
        return self.initial_transformation

    def compute_gicp_transformation(self, voxel_size=0.0, remove_ground_plane=False):
        """
        Compute the GICP transformation between the source and target lidars.

        Args:
            voxel_size: The voxel size for downsampling the point clouds. If 0, no downsampling is performed.
            remove_ground_plane: If True, the ground plane is removed from the point clouds.

        Returns:
            The result of the GICP registration.
        """
        if self.source.pcd is None:
            raise Exception("no source point cloud")
        if self.target.pcd is None:
            raise Exception("no target point cloud")

        # Create copies of the source and target point clouds
        source_pcd = o3d.geometry.PointCloud(self.source.pcd)
        target_pcd = o3d.geometry.PointCloud(self.target.pcd)

        # Downsample the point clouds if a voxel size is provided
        if voxel_size > 0.0:
            source_pcd = source_pcd.voxel_down_sample(voxel_size)
            target_pcd = target_pcd.voxel_down_sample(voxel_size)

        # Remove the ground plane from the point clouds if requested
        if remove_ground_plane:
            source_pcd = self.source.remove_ground_plane(
                source_pcd, self.distance_threshold, self.ransac_n, self.num_iterations
            )
            target_pcd = self.source.remove_ground_plane(
                target_pcd, self.distance_threshold, self.ransac_n, self.num_iterations
            )

        # Estimate normals for the point clouds
        source_pcd.estimate_normals()
        target_pcd.estimate_normals()

        # Perform GICP registration
        reg_p2l = o3d.pipelines.registration.registration_generalized_icp(
            source_pcd,
            target_pcd,
            self.max_corresp_dist,
            self.initial_transformation.matrix,
            o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(self.epsilon),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                self.rel_fitness, self.rel_rmse, self.max_iterations
            ),
        )

        # Store the transformation matrix and registration result
        self.calibrated_transformation = TransformationMatrix.from_matrix(reg_p2l.transformation)
        self.reg_p2l = reg_p2l
        return reg_p2l

    def transform_pointcloud(self, transformation_matrix=None):
        """
        Apply a transformation to the source point cloud.

        Args:
            transformation_matrix: The transformation matrix to apply. If None, the calibrated transformation is used.

        Returns:
            None
        """
        if transformation_matrix is None:
            if self.calibrated_transformation is None:
                self.compute_gicp_transformation()
            self.source.pcd_transformed.transform(self.calibrated_transformation.matrix)
            self.source.calib_tf_matrix = self.calibrated_transformation
        else:
            self.source.pcd_transformed.transform(transformation_matrix)
            self.source.calib_tf_matrix = TransformationMatrix.from_matrix(transformation_matrix)

    def info(self, degrees=False, matrix=False):
        """
        Generate a string containing information about the calibration.

        Args:
            degrees: If True, the rotation angles are converted to degrees. If False, they are left in radians.
            matrix: If True, prints the transformation matrix.
        Returns:
            A string containing information about the calibration.
        """
        if matrix:
            s = (
                "calibrated transformation matrix:\n"
                + str(self.calibrated_transformation.matrix)
                + "\n"
            )
        else:
            s = ""
        return (
            self.source.name
            + " to "
            + self.target.name
            + " calibration\n"
            + "initial xyz = "
            + self.initial_transformation.translation.__str__()
            + "\n"
            + "initial rpy = "
            + self.initial_transformation.rotation.__str__(degrees)
            + "\n"
            + "calibrated xyz = "
            + self.calibrated_transformation.translation.__str__()
            + "\n"
            + "calibrated rpy = "
            + self.calibrated_transformation.rotation.__str__(degrees)
            + "\n"
            + "fitness: "
            + str(self.reg_p2l.fitness)
            + ", inlier_rmse: "
            + str(self.reg_p2l.inlier_rmse)
            + "\n"
            + s
            + "_" * 100
        )
