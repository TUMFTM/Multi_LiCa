import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d

import teaserpp_python
from scipy.spatial import cKDTree

from .Geometry import Rotation, TransformationMatrix, Translation
from .Lidar import Lidar
from scipy.spatial.transform import Rotation as R


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
        [0, 0.4, 0.74],
        [1, 1, 1],
        [0.89, 0.45, 0.13],
        [0.64, 0.68, 0],
        [0.6, 0.78, 0.92],
        [0.5, 0.5, 0.5],
        [0.8, 0.8, 0.8],
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
        crop_cloud=20,
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
        self.crop_cloud = crop_cloud
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

        if self.source.pcd is None:
            raise Exception("no source point cloud")
        if self.target.pcd is None:
            raise Exception("no target point cloud")

        # Create copies of the source and target point clouds
        source_pcd = o3d.geometry.PointCloud(self.source.pcd.transform(transformation_matrix))
        target_pcd = o3d.geometry.PointCloud(self.target.pcd)

        method = 'TEASER'
        if method == 'FPFH':
            fpfh_voxel_size = 0.5
            source_fpfh = self.preprocess_point_cloud(source_pcd, fpfh_voxel_size)
            target_fpfh = self.preprocess_point_cloud(target_pcd, fpfh_voxel_size)
            distance_threshold = fpfh_voxel_size * 10
            reg_fpfh = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
                source_pcd, target_pcd, source_fpfh, target_fpfh,
                o3d.pipelines.registration.FastGlobalRegistrationOption(
                    use_absolute_scale=True,
                    maximum_correspondence_distance=distance_threshold))
            print("FPFH Trafo:")
            print(reg_fpfh.transformation)
            self.initial_transformation = TransformationMatrix.from_matrix(reg_fpfh.transformation)
        
        elif method == 'RANSAC':
            voxel_size = 0.35
            num_iterations = 20
            distance_threshold = voxel_size * 20
            transformations = [self.run_ransac_registration(source_pcd, target_pcd, voxel_size, distance_threshold) for _ in range(num_iterations)]
            median_trans = self.median_transformation(transformations)
            self.initial_transformation = TransformationMatrix.from_matrix(median_trans)

        elif method == 'TEASER':
            voxel_size = 0.35
            teaser_transformation = self.teaser_initial_registration(source_pcd, target_pcd, voxel_size)
            self.initial_transformation = TransformationMatrix.from_matrix(teaser_transformation)
        return self.initial_transformation
    
    def teaser_initial_registration(self, source_pcd, target_pcd, voxel_size):
        source_down, source_fpfh = self.preprocess_point_cloud(source_pcd, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target_pcd, voxel_size)

        source_fpfh = np.array(source_fpfh.data).T
        target_fpfh = np.array(target_fpfh.data).T

        source_down = np.asarray(source_down.points).T
        target_down = np.asarray(target_down.points).T

        corrs_A, corrs_B = self.find_correspondences(
            source_fpfh, target_fpfh, mutual_filter=True)
        A_corr = source_down[:,corrs_A]
        B_corr = target_down[:,corrs_B]

        teaser_solver = self.get_teaser_solver(voxel_size)
        teaser_solver.solve(A_corr,B_corr)
        solution = teaser_solver.getSolution()
        transformation = np.eye(4)
        transformation[:3, :3] = solution.rotation
        transformation[:3, 3] = solution.translation
        return transformation
    
    def find_correspondences(self, feats0, feats1, mutual_filter=True):
        nns01 = self.find_knn_cpu(feats0, feats1, knn=1, return_distance=False)
        corres01_idx0 = np.arange(len(nns01))
        corres01_idx1 = nns01

        if not mutual_filter:
            return corres01_idx0, corres01_idx1

        nns10 = self.find_knn_cpu(feats1, feats0, knn=1, return_distance=False)
        corres10_idx1 = np.arange(len(nns10))
        corres10_idx0 = nns10

        mutual_filter = (corres10_idx0[corres01_idx1] == corres01_idx0)
        corres_idx0 = corres01_idx0[mutual_filter]
        corres_idx1 = corres01_idx1[mutual_filter]

        return corres_idx0, corres_idx1
    
    def find_knn_cpu(self, feat0, feat1, knn=1, return_distance=False):
        feat1tree = cKDTree(feat1)
        dists, nn_inds = feat1tree.query(feat0, k=knn, n_jobs=-1)
        if return_distance:
            return nn_inds, dists
        else:
            return nn_inds
        
    
    def get_teaser_solver(self, noise_bound):
        solver_params = teaserpp_python.RobustRegistrationSolver.Params()
        solver_params.cbar2 = 1.0
        solver_params.noise_bound = noise_bound
        solver_params.estimate_scaling = False
        solver_params.inlier_selection_mode = \
            teaserpp_python.RobustRegistrationSolver.INLIER_SELECTION_MODE.PMC_EXACT
        solver_params.rotation_tim_graph = \
            teaserpp_python.RobustRegistrationSolver.INLIER_GRAPH_FORMULATION.CHAIN
        solver_params.rotation_estimation_algorithm = \
            teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
        solver_params.rotation_gnc_factor = 1.4
        solver_params.rotation_max_iterations = 10000
        solver_params.rotation_cost_threshold = 1e-16
        solver = teaserpp_python.RobustRegistrationSolver(solver_params)
        return solver

    def run_ransac_registration(self, source_pcd, target_pcd, voxel_size, distance_threshold):
        source_down, source_fpfh = self.preprocess_point_cloud(source_pcd, voxel_size)
        target_down, target_fpfh = self.preprocess_point_cloud(target_pcd, voxel_size)
    
        reg_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh, True,
            distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            3,
            [],
            o3d.pipelines.registration.RANSACConvergenceCriteria(1000, 0.925)
        )
        return reg_ransac.transformation

    def transformation_to_rpy_xyz(self, transformation):
        rotation_matrix = transformation[:3, :3].copy()
        r = R.from_matrix(rotation_matrix)
        roll, pitch, yaw = r.as_euler('xyz', degrees=False)
        translation = transformation[:3, 3]
        return roll, pitch, yaw, translation[0], translation[1], translation[2]

    def median_transformation(self, transformations):
        rpy_xyz_list = [self.transformation_to_rpy_xyz(trans) for trans in transformations]
        rpy_xyz_array = np.array(rpy_xyz_list)
        
        median_rpy_xyz = np.median(rpy_xyz_array, axis=0)
        roll, pitch, yaw, x, y, z = median_rpy_xyz
        
        median_rotation_matrix = R.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
        median_transformation_matrix = np.eye(4)
        median_transformation_matrix[:3, :3] = median_rotation_matrix
        median_transformation_matrix[:3, 3] = [x, y, z]
        return median_transformation_matrix

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

    def preprocess_point_cloud(self, pcd, voxel_size):
        pcd_down = pcd
        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-self.crop_cloud, -self.crop_cloud, -self.crop_cloud),
                                                    max_bound=(self.crop_cloud, self.crop_cloud, self.crop_cloud))
        pcd_down = pcd.crop(bbox)
        pcd_down = pcd_down.voxel_down_sample(voxel_size)
        # pcd_down = self.source.remove_ground_plane(
        #         pcd, self.distance_threshold, self.ransac_n, self.num_iterations
        #     )
        radius_normal = voxel_size * 5
        pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))

        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
            pcd_down,
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh