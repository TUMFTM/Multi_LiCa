import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

def load_config(file_path):
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def pose_to_matrix(x, y, z, qx, qy, qz, qw):
    rotation = R.from_quat([qx, qy, qz, qw])
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation.as_matrix()
    transformation_matrix[:3, 3] = [x, y, z]
    return transformation_matrix

def relative_to_absolute(main_matrix, rel_pose):
    x, y, z, roll, pitch, yaw = rel_pose
    roll, pitch, yaw = np.deg2rad([roll, pitch, yaw])
    rotation = R.from_euler('xyz', [roll, pitch, yaw])
    rel_matrix = np.eye(4)
    rel_matrix[:3, :3] = rotation.as_matrix()
    rel_matrix[:3, 3] = [x, y, z]
    abs_matrix = np.dot(main_matrix, rel_matrix)
    return abs_matrix

def matrix_to_pose(matrix):
    rotation = R.from_matrix(matrix[:3, :3])
    quat = rotation.as_quat()
    translation = matrix[:3, 3]
    return translation.tolist() + quat.tolist()

def calculate_relative_pose(main_matrix, matrix):
    rel_matrix = np.dot(np.linalg.inv(main_matrix), matrix)
    rotation = R.from_matrix(rel_matrix[:3, :3])
    translation = rel_matrix[:3, 3]
    euler_angles = np.rad2deg(rotation.as_euler('xyz'))
    return translation.tolist() + euler_angles.tolist()

def calculate_rmse_error(gt_pose, calc_pose):
    gt_translation = np.array(gt_pose[:3])
    calc_translation = np.array(calc_pose[:3])
    translation_errors = gt_translation - calc_translation
    translation_error_rmse = np.sqrt(np.mean(translation_errors ** 2))

    gt_rotation = R.from_euler('xyz', gt_pose[3:], degrees=True)
    calc_rotation = R.from_euler('xyz', calc_pose[3:], degrees=True)
    rotation_error = gt_rotation.inv() * calc_rotation
    rotation_error_degrees = rotation_error.magnitude() * (180.0 / np.pi)

    gt_euler = np.array(gt_pose[3:])
    calc_euler = np.array(calc_pose[3:])
    rotation_errors = gt_euler - calc_euler

    return translation_errors, translation_error_rmse, rotation_errors, rotation_error_degrees

def evaluate_poses(ground_truth, calibration):
    main_lidar_pose = ground_truth['main_lidar']
    main_lidar_matrix = pose_to_matrix(*main_lidar_pose)

    # Ground truth relative poses
    ground_truth_rel = {}
    for lidar, pose in ground_truth.items():
        if lidar != 'main_lidar':
            matrix = pose_to_matrix(*pose)
            ground_truth_rel[lidar] = calculate_relative_pose(main_lidar_matrix, matrix)

    # Comparison
    comparison = {}
    for lidar, rel_pose in calibration.items():
        abs_matrix = relative_to_absolute(main_lidar_matrix, rel_pose)
        gt_rel_pose = ground_truth_rel.get(lidar, None)
        if gt_rel_pose:
            calc_rel_pose = calculate_relative_pose(main_lidar_matrix, abs_matrix)
            translation_errors, translation_error_rmse, rotation_errors, rotation_error_degrees = calculate_rmse_error(gt_rel_pose, calc_rel_pose)
            comparison[lidar] = {
                "ground_truth": gt_rel_pose,
                "calculated": rel_pose,
                "absolute_calculated": matrix_to_pose(abs_matrix),
                "translation_error_rmse": translation_error_rmse,
                "rotation_error_degrees": rotation_error_degrees,
                "translation_errors": translation_errors.tolist(),
                "rotation_errors": rotation_errors.tolist()
            }

    return comparison

def main(config_path):
    config = load_config(config_path)
    ground_truth = config['ground_truth']
    calibration = config['calibration']
    
    evaluation = evaluate_poses(ground_truth, calibration)
    
    for lidar, data in evaluation.items():
        print(f"Evaluation for {lidar}:")
        print(f"  Ground Truth Relative Pose: {data['ground_truth']}")
        print(f"  Calculated Relative Pose: {data['calculated']}")
        print(f"  Absolute Calculated Pose: {data['absolute_calculated']}")
        print(f"  Translation Errors [x, y, z]: {data['translation_errors']}")
        print(f"  Rotation Errors [r, p, y] (degrees): {data['rotation_errors']}")
        print(f"  Translation Error (RMSE): {data['translation_error_rmse']}")
        print(f"  Rotation Error (Degrees): {data['rotation_error_degrees']}")
        print()

if __name__ == "__main__":
    config_path = "config.yaml"
    main(config_path)
