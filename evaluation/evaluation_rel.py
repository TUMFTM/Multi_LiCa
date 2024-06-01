import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

def load_config(file_path):
    with open(file_path, 'r') as file:
        config = yaml.safe_load(file)
    return config

def calculate_rmse_error(gt_pose, calc_pose):
    gt_translation = np.array(gt_pose[:3])
    calc_translation = np.array(calc_pose[:3])
    translation_errors = gt_translation - calc_translation
    translation_error_rmse = np.sqrt(np.mean(translation_errors ** 2))

    gt_rotation = R.from_euler('xyz', gt_pose[3:], degrees=True)
    calc_rotation = R.from_euler('xyz', calc_pose[3:], degrees=True)
    rotation_error = gt_rotation.inv() * calc_rotation
    rotation_error_degrees = rotation_error.magnitude() * (180.0 / np.pi)
    rotation_errors_individual = rotation_error.as_euler('xyz', degrees=True)

    return translation_errors, translation_error_rmse, rotation_error_degrees, rotation_errors_individual

def main(config_path):
    config = load_config(config_path)
    ground_truth = config['ground_truth']
    calibration = config['calibration']
    
    main_lidar_pose = ground_truth['main_lidar']
    
    translation_errors_all = []
    rotation_errors_all = []
    rotation_errors_individual_all = []
    
    for lidar, gt_pose in ground_truth.items():
        if lidar != 'main_lidar':
            gt_pose_matrix = np.array(gt_pose)
            calc_pose = calibration[lidar]
            translation_errors, translation_error_rmse, rotation_error_degrees, rotation_errors_individual = calculate_rmse_error(gt_pose_matrix, calc_pose)
            print(f"Errors for {lidar}:")
            print(f"  Translation Errors [x, y, z]: {translation_errors}")
            print(f"  Translation Error (RMSE) [m]: {translation_error_rmse}")
            print(f"  Rotation Error (Degrees): {rotation_error_degrees}")
            print(f"  Rotation Errors Individual [r, p, y]: {rotation_errors_individual}")
            print()
            translation_errors_all.append(translation_errors)
            rotation_errors_all.append(rotation_error_degrees)
            rotation_errors_individual_all.append(rotation_errors_individual)
    
    avg_translation_errors = np.mean(translation_errors_all, axis=0)
    avg_rotation_errors = np.mean(rotation_errors_all)
    avg_rotation_errors_individual = np.mean(rotation_errors_individual_all, axis=0)
    
    print(f"Average Translation Errors [m]: {avg_translation_errors}")
    print(f"Average Rotation Error (Degrees): {avg_rotation_errors}")
    print(f"Average Rotation Errors Individual [r, p, y]: {avg_rotation_errors_individual}")

if __name__ == "__main__":
    config_path = "config.yaml"
    main(config_path)
