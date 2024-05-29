import glob
import os
from time import time

import numpy as np
import open3d as o3d
import rclpy
import ros2_numpy as rnp
from geometry_msgs.msg import Transform
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from .calibration.Calibration import *

 
def get_transfrom(tf_msg: TFMessage, child_frame_id: str) -> Transform:
    """
    Extract the transform for a specific child frame from a TFMessage.

    Args:
        tf_msg: A TFMessage containing transforms.
        child_frame_id: The ID of the child frame for which to get the transform.

    Returns:
        The transform for the specified child frame, or None if the child frame is not in the TFMessage.
    """
    for tf in tf_msg.transforms:
        if tf.child_frame_id == child_frame_id:
            return tf.transform
    return None


class MultiLidarCalibrator(Node):
    """
    A ROS node for calibrating multiple LiDAR sensors.

    This node subscribes to multiple LiDAR and /tf_static topics or reads the data from files
    and performs calibration using the Generalized Iterative Closest Point (GICP) algorithm.
    The calibration parameters can be adjusted using ROS parameters.
    """

    def __init__(self):
        super().__init__("multi_subscriber_node")
        self.tf_topic = self.declare_parameter("tf_topic", "/tf_static").value
        self.visualize = self.declare_parameter("visualize", False).value
        self.use_fitness_based_calibration = self.declare_parameter(
            "use_fitness_based_calibration", False
        ).value
        self.read_tf_from_table = self.declare_parameter("read_tf_from_table", True).value
        self.table_degrees = self.declare_parameter("table_degrees", True).value
        self.topic_names = self.declare_parameter("lidar_topics", ["lidar_1, lidar_2"]).value
        self.target_lidar = self.declare_parameter("target_frame_id", "lidar_1").value
        self.base_frame_id = self.declare_parameter("base_frame_id", "base_link").value
        self.calibrate_target = self.declare_parameter("calibrate_target", False).value
        self.calibrate_to_base = self.declare_parameter("calibrate_to_base", False).value
        self.base_to_ground_z = self.declare_parameter("base_to_ground_z", 0.0).value
        self.frame_count = self.declare_parameter("frame_count", 1).value
        self.runs_count = self.declare_parameter("runs_count", 1).value
        self.crop_cloud = self.declare_parameter("crop_cloud", 25).value

        self.output_dir = (
            os.path.dirname(os.path.realpath(__file__))
            + self.declare_parameter("output_dir", "/../output/").value
        )
        self.pcd_in_dir = (
            os.path.dirname(os.path.realpath(__file__))
            + self.declare_parameter("pcd_directory", "/../data/demo/").value
        )

        self.rel_fitness = self.declare_parameter("rel_fitness", 1e-7).value
        self.rel_rmse = self.declare_parameter("rel_rmse", 1e-7).value
        self.max_iterations = self.declare_parameter("max_iterations", 100).value
        self.max_corresp_dist = self.declare_parameter("max_corresp_dist", 1.0).value
        self.epsilon = self.declare_parameter("epsilon", 0.005).value
        self.voxel_size = self.declare_parameter("voxel_size", 0.05).value
        self.r_voxel_size = self.declare_parameter("r_voxel_size", 0.15).value
        self.remove_ground_flag = self.declare_parameter("remove_ground_flag", False).value
        self.fitness_score_threshold = self.declare_parameter("fitness_score_threshold", 0.2).value
        self.distance_threshold = self.declare_parameter("distance_threshold", 0.1).value
        self.ransac_n = self.declare_parameter("ransac_n", 15).value
        self.num_iterations = self.declare_parameter("num_iterations", 5000).value
        self.r_runs = self.declare_parameter("r_runs", 5).value
        self.urdf_path = self.declare_parameter("urdf_path", "").value
        self.results_file = self.declare_parameter("results_file", "results.txt").value
        self.lidar_data = {}
        self.lidar_dict = {}
        self.subscribers = []
        self.counter = 0
        self.read_pcds_from_file = self.declare_parameter("read_pcds_from_file", True).value
        with open(self.output_dir + self.results_file, "w") as file:  # clean the file
            file.write("")
        self.tf_msg: TFMessage = None
        for topic in self.topic_names:
            self.subscribers.append(
                self.create_subscription(PointCloud2, topic, self.pointcloud_callback, 10)
            )
        self.tf_subscriber = self.create_subscription(
            TFMessage, self.tf_topic, self.tf_callback, 10
        )
        self.declared_lidars_flag = False

        # read all the data from files (without ROS)
        if self.read_pcds_from_file and self.read_tf_from_table:
            # get LiDAR names as subdirectory names
            lidar_list = [
                os.path.splitext(os.path.basename(path))[0]
                for path in glob.glob(self.pcd_in_dir + "*")
            ]
            pcd_paths = dict(zip(lidar_list, [None] * len(lidar_list)))
            for lidar in lidar_list:
                self.declare_parameter(lidar, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                pcd_paths[lidar] = glob.glob(self.pcd_in_dir + lidar + "*")
            self.lidar_dict = dict(
                zip(
                    lidar_list,
                    [
                        Lidar(
                            lidar,
                            Translation(*self.get_parameter(lidar).value[0:3]),
                            Rotation(*self.get_parameter(lidar).value[3:], True),
                        )
                        for lidar in lidar_list
                    ],
                )
            )
            for lidar in lidar_list:
                pcd = o3d.io.read_point_cloud(pcd_paths[lidar][0])
                for i in range(1, self.frame_count):
                    pcd += o3d.io.read_point_cloud(pcd_paths[lidar][i])
                self.lidar_dict[lidar].load_pcd(pcd)
            self.process_data()
            exit(0)
        elif self.read_pcds_from_file and not self.read_tf_from_table:
            self.get_logger().error("read_tf_from_table is set to False")
            exit(1)
        else:
            self.get_logger().info("waiting for point cloud messages...")

    def log_calibration_info(self, calibration: Calibration):
        """Log calibration information in ROS and output file"""
        calibration_info = f"calibration info:\n{calibration.info(True, True)}"
        self.get_logger().info(calibration_info)
        with open(self.output_dir + self.results_file, "a") as file:
            file.write(calibration_info + "\n")

    def read_data(self):
        """Read point clouds from ROS and LiDAR initial transformation from either ROS or table."""
        self.get_logger().info("Received all the needed point clouds")
        self.get_logger().info(
            "Reading initial transformations from tf and converting point clouds..."
        )
        # create a dict of <lidar_name>:<Lidar> whereas Lidar is created using data from
        # /tf_static or parameter table
        if self.read_tf_from_table:
            self.lidar_dict = dict(
                zip(
                    self.lidar_data.keys(),
                    [
                        Lidar(
                            lidar,
                            Translation(*self.get_parameter(lidar).value[0:3]),
                            Rotation(*self.get_parameter(lidar).value[3:], True),
                        )
                        for lidar in self.lidar_data.keys()
                    ],
                )
            )
        else:
            self.lidar_dict = dict(
                zip(
                    self.lidar_data.keys(),
                    [
                        Lidar.from_transform(lidar, get_transfrom(self.tf_msg, lidar))
                        for lidar in self.lidar_data.keys()
                    ],
                )
            )
        for key in self.lidar_data.keys():
            # convert data from ros to pcd needed for open3d
            t = rnp.numpify(self.lidar_data[key][0])
            pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(t["xyz"]))
            for i in range(1, self.frame_count):
                t = rnp.numpify(self.lidar_data[key][i])
                pcd += o3d.geometry.PointCloud(o3d.utility.Vector3dVector(t["xyz"]))
            self.lidar_dict[key].load_pcd(pcd)
        self.get_logger().info("Converted all the needed ros-data")

    def standard_calibration(self, target_lidar: Lidar):
        """
        Perform standard calibration using the Generalized Iterative Closest Point (GICP) algorithm.

        Args:
            target_lidar: The target LiDAR sensor to which other sensors will be calibrated.

        Returns:
            None
        """
        calibrated_lidars, problematic_lidars = [], []
        for source_lidar in list(self.lidar_dict.values()):
            if source_lidar.name == target_lidar.name:
                continue
            # Create a Calibration object and compute the GICP transformation
            calibration = Calibration(
                source_lidar,
                target_lidar,
                self.max_corresp_dist,
                self.epsilon,
                self.rel_fitness,
                self.rel_rmse,
                self.max_iterations,
                self.distance_threshold,
                self.ransac_n,
                self.num_iterations,
                self.crop_cloud,
            )
            calibration.compute_gicp_transformation(self.voxel_size, self.remove_ground_flag)
            self.get_logger().info('%s -> %s: %f' % (calibration.source.name, calibration.target.name, calibration.reg_p2l.fitness))
            # Check the fitness score of the calibration
            if calibration.reg_p2l.fitness <= self.fitness_score_threshold:
                problematic_lidars.append(source_lidar)
                self.get_logger().info(
                    f"Unsuccessful {source_lidar.name} calibration. " + "Trying other targets..."
                )
                continue
            # Apply the transformation to the point cloud
            calibration.transform_pointcloud()
            # Log the calibration information
            self.log_calibration_info(calibration)
            # Modify the URDF file if a path is provided
            if self.urdf_path != "":
                modify_urdf_joint_origin(
                    self.urdf_path,
                    source_lidar.name + "_joint",
                    calibration.calibrated_transformation,
                )
            calibrated_lidars.append(source_lidar)

        # If there are problematic lidars, try to calibrate them to other lidars
        if len(problematic_lidars) > 0 and len(calibrated_lidars) > 0:
            # Create a combined point cloud from all successfully calibrated lidars
            combined_lidar = Lidar(
                target_lidar.name, target_lidar.translation, target_lidar.rotation
            )
            combined_lidar.load_pcd(
                np.sum([lidar.pcd_transformed for lidar in calibrated_lidars]) + target_lidar.pcd
            )
            for source_lidar in problematic_lidars:
                # Calibrate the problematic lidar to the combined lidar
                calibration = Calibration(
                    source_lidar,
                    combined_lidar,
                    self.max_corresp_dist,
                    self.epsilon,
                    self.rel_fitness,
                    self.rel_rmse,
                    self.max_iterations,
                    self.distance_threshold,
                    self.ransac_n,
                    self.num_iterations,
                    self.crop_cloud,
                )
                calibration.compute_gicp_transformation(self.voxel_size, self.remove_ground_flag)
                if calibration.reg_p2l.fitness <= self.fitness_score_threshold:
                    self.get_logger().info(f"Problem with lidar {source_lidar.name} calibration")
                    with open(self.output_dir + self.results_file, "a") as file:
                        file.write("Calibration NOT SUCCESSFUL!\n")
                calibration.transform_pointcloud()
                self.log_calibration_info(calibration)
                if self.urdf_path != "":
                    modify_urdf_joint_origin(
                        self.urdf_path,
                        source_lidar.name + "_joint",
                        calibration.calibrated_transformation,
                    )

        elif len(problematic_lidars) > 0 and len(calibrated_lidars) == 0:
            self.get_logger().error(
                "calibration failed. Try choosing other target lidar "
                + ", providing more accurate initial transformation, "
                + "or using fitness based calibration (params)"
            )

    def fitness_based_calibration(self, target_lidar: Lidar):
        """
        Perform fitness-based calibration using the Generalized Iterative Closest Point (GICP) algorithm.

        This method first finds the source and target that result in the highest fitness score, performs the calibration, and saves the resulting cloud. Then, it finds a new calibration with the highest score (including the calibrated cloud). This process is repeated until all other LiDARs are calibrated to the target.

        Args:
            target_lidar: The target LiDAR sensor to which other sensors will be calibrated.

        Returns:
            None
        """
        # Initialize lists for calibrated and problematic lidars
        calibrations_tmp = []
        not_calibrated = list(self.lidar_dict.values())

        # Compute GICP transformations for all pairs of lidars
        for source_lidar in not_calibrated:
            for other_lidar in not_calibrated:
                if source_lidar.name != other_lidar.name:
                    calibration = Calibration(
                        source_lidar,
                        other_lidar,
                        self.max_corresp_dist,
                        self.epsilon,
                        self.rel_fitness,
                        self.rel_rmse,
                        self.max_iterations,
                        self.distance_threshold,
                        self.ransac_n,
                        self.num_iterations,
                        self.crop_cloud,
                    )
                    self.get_logger().info('target: %s' % (calibration.target.name))
                    self.get_logger().info('source: %s' % (calibration.source.name))
                    calibration.compute_gicp_transformation(
                        self.voxel_size, self.remove_ground_flag
                    )
                    self.get_logger().info('fitness score: %f' % (calibration.reg_p2l.fitness))
                    calibrations_tmp.append(calibration)

        # Repeat until only the target lidar is left
        while not_calibrated != [target_lidar]:
            self.get_logger().info('target_LiDAR: %s' % (target_lidar.name))
            # Choose the calibration with the highest fitness score
            self.get_logger().info('fitness score list')
            for _ in calibrations_tmp:
                self.get_logger().info('%s -> %s: %f' % (_.source.name, _.target.name, _.reg_p2l.fitness))
            max_fitness_index = np.argmax(
                [calibration.reg_p2l.fitness for calibration in calibrations_tmp]
            )
            self.get_logger().info('max_fitness_index: %f' % (max_fitness_index))
            calibration = calibrations_tmp[max_fitness_index]
            self.get_logger().info('target: %s' % (calibration.target.name))
            self.get_logger().info('source: %s' % (calibration.source.name))
            

            # If the fitness score is below the threshold, exit with an error
            if calibration.reg_p2l.fitness <= self.fitness_score_threshold:
                self.get_logger().error(
                    f"no calibration within given threshold possible, "
                    + "try reducing the thershold, make more overlap between the point clouds "
                    + "or provide more accurate initial transformation"
                )
                exit(1)

            # Don't transform the target lidar
            if calibration.source == target_lidar:
                calibration.source, calibration.target = calibration.target, calibration.source
                calibration.initial_transformation = TransformationMatrix.from_matrix(
                    np.linalg.inv(calibration.initial_transformation.matrix)
                )
                calibration.calibrated_transformation = TransformationMatrix.from_matrix(
                    np.linalg.inv(calibration.calibrated_transformation.matrix)
                )

            # Apply the transformation to the point cloud
            calibration.transform_pointcloud()

            # Combine the point clouds of the source and target lidars
            calibration.target.pcd += calibration.source.pcd_transformed

            # If the target lidar is the original target, remove the source from the list of not calibrated lidars
            if calibration.target == target_lidar:
                not_calibrated.remove(calibration.source)
                self.log_calibration_info(calibration)
                if self.urdf_path != "":
                    modify_urdf_joint_origin(
                        self.urdf_path,
                        calibration.source.name + "_joint",
                        calibration.calibrated_transformation,
                    )

            # Remove all calibrations involving the source lidar
            calibrations_tmp = [
                c
                for c in calibrations_tmp
                if calibration.source != c.source and calibration.source != c.target
            ]

            # Recompute fitness scores for calibrations involving the target lidar
            for c in calibrations_tmp:
                if c.source == calibration.target or c.target == calibration.target:
                    c.compute_gicp_transformation(self.voxel_size, self.remove_ground_flag)

    def process_data(self):
        """
        Start the calibration pipeline.

        If calibrate_to_base and calibrate_target are set, the target LiDAR is first calibrated to the base frame.
        Then, other LiDARs are calibrated to the calibrated target LiDAR, i.e., to the base frame.

        If only LiDAR-to-LiDAR calibration is required, it can be performed without calibrating the target LiDAR to the base frame.

        Returns:
            None
        """
        self.get_logger().info("Starting the calibration...")
        if self.visualize:
            visualize_calibration(list(self.lidar_dict.values()), False)
        target_lidar = self.lidar_dict[self.target_lidar]
        if self.calibrate_to_base and self.calibrate_target:
            # Perform target to ground (base) calibration. This computes the z-distance between the ground and the target
            # LiDAR and calibrates the pitch angle. It assumes that x, y, and yaw are precisely known
            # as well as the transformation between the base and ground.
            self.get_logger().info(f"Calibrating target lidar to the ground")
            roll, pitch = self.lidar_dict[self.target_lidar].calibrate_pitch(
                self.distance_threshold,
                self.ransac_n,
                self.num_iterations,
                self.r_voxel_size,
                self.r_runs,
            )

            # Update the target lidar's rotation
            target_lidar.rotation.y = pitch
            rotation = target_lidar.rotation

            # Create a horizontal point cloud to use as a ground reference
            horizontal = Lidar(self.base_frame_id, Translation(0, 0, 0), Rotation(0, 0, 0))
            dir_path = os.path.dirname(os.path.realpath(__file__))
            horizontal.read_pcd(dir_path + "/calibration/points_horizontal.pcd")

            # Calibrate the target lidar to the ground
            calibration = Calibration(
                target_lidar,
                horizontal,
                self.max_corresp_dist,
                self.epsilon,
                self.rel_fitness,
                self.rel_rmse,
                self.max_iterations,
                self.crop_cloud,
            )
            translation = target_lidar.translation
            calibration.compute_gicp_transformation(self.voxel_size, remove_ground_plane=False)

            # Update the target lidar's translation
            if self.read_tf_from_table:
                translation.z = (
                    calibration.calibrated_transformation.translation.z - self.base_to_ground_z
                )
            else:
                translation.z = (
                    calibration.calibrated_transformation.translation.z
                    - get_transfrom(self.tf_msg, self.base_frame_id).translation.z
                )

            # Update the target lidar's transformation and point cloud
            calibration.calibrated_transformation = TransformationMatrix(translation, rotation)
            self.log_calibration_info(calibration)
            if self.urdf_path != "":
                modify_urdf_joint_origin(
                    self.urdf_path,
                    self.target_lidar + "_joint",
                    calibration.calibrated_transformation,
                )
            calibration.transform_pointcloud()

            # Reset the target lidar's transformation and update its point cloud
            self.lidar_dict.pop(self.target_lidar)
            target_lidar = Lidar(self.base_frame_id, Translation(0, 0, 0), Rotation(0, 0, 0))
            target_lidar.load_pcd(calibration.source.pcd_transformed)
            self.lidar_dict[target_lidar.name] = target_lidar

        elif self.calibrate_to_base:
            # If the target to base transformation is already known, just transform the point cloud
            target_lidar.pcd.transform(target_lidar.tf_matrix.matrix)
            target_lidar.translation = Translation(0, 0, 0)
            target_lidar.rotation = Rotation(0, 0, 0)

        # Perform the main LiDAR-to-LiDAR calibration
        if self.use_fitness_based_calibration:
            self.fitness_based_calibration(target_lidar)
        else:
            self.standard_calibration(target_lidar)

        visualize_calibration(list(self.lidar_dict.values()), True, not self.visualize)
        # Stitch point clouds
        stitched_pcd = o3d.geometry.PointCloud()
        for lidar in self.lidar_dict.values():
            stitched_pcd += lidar.pcd
        # Save stitched point clouds to a .pcd file
        o3d.io.write_point_cloud(self.output_dir + "stitched_initial.pcd", stitched_pcd)
        # Create a point cloud for the transformed data
        stitched_pcd_transformed = o3d.geometry.PointCloud()
        for lidar in self.lidar_dict.values():
            stitched_pcd_transformed += lidar.pcd_transformed
        # Save the transformed point clouds to a .pcd file
        o3d.io.write_point_cloud(
            self.output_dir + "stitched_transformed.pcd", stitched_pcd_transformed
        )
        self.get_logger().info(f"Saved fused point cloud: {self.output_dir}")
        self.get_logger().info(f"Calibrations results are stored in: {self.output_dir}")

    def tf_callback(self, msg):
        self.get_logger().info("Received TFMessage data")
        self.tf_msg = msg

    def pointcloud_callback(self, msg: PointCloud2):
        # Wait for the TFMessage before processing the point cloud if table is not used
        if self.tf_msg is None and not self.read_tf_from_table:
            self.get_logger().info("Waiting for tf...")
            return

        # Add the point cloud to the lidar_data dictionary
        if msg.header.frame_id not in self.lidar_data.keys():
            self.lidar_data[msg.header.frame_id] = [msg]
        else:
            if len(self.lidar_data[msg.header.frame_id]) >= self.frame_count:
                return  # Don't save more point clouds than needed
            self.lidar_data[msg.header.frame_id].append(msg)

        # Process the data after receiving the required number of point clouds for each LiDAR
        if [len(self.lidar_data[i]) == self.frame_count for i in self.lidar_data.keys()] == [
            True
        ] * len(self.topic_names):
            if self.read_tf_from_table and not self.declared_lidars_flag:
                for lidar in self.lidar_data.keys():
                    self.declare_parameter(lidar)
                self.declared_lidars_flag = True  # Don't repeatedly declare the same parameters
            begin = time()
            self.read_data()
            self.process_data()
            end = time()
            with open(self.output_dir + self.results_file, "a") as file:
                file.write(f"Complete calibration time: {end - begin}\n")
            self.lidar_data = {}  # Clean the data after each calibration (for multiple runs)
            self.counter += 1
            if self.counter >= self.runs_count:
                exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = MultiLidarCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
