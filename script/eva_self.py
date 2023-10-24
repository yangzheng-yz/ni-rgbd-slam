import numpy as np
from scipy.spatial.transform import Rotation as R

def read_last_line(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        last_line = lines[-1]
        data = list(map(float, last_line.strip().split()))
    return data[1:4], data[4:]  # Separate position and quaternion

def calculate_error(groundtruth_file, estimated_file):
    # Read the last line (position and quaternion) from each file
    gt_pos, gt_quat = read_last_line(groundtruth_file)
    est_pos, est_quat = read_last_line(estimated_file)

    # Build the pose matrices
    T_gt = np.eye(4)
    T_gt[:3, 3] = gt_pos
    T_gt[:3, :3] = R.from_quat(gt_quat).as_matrix()

    T_est = np.eye(4)
    T_est[:3, 3] = est_pos
    T_est[:3, :3] = R.from_quat(est_quat).as_matrix()

    # Compute the error matrix
    T_error = np.linalg.inv(T_gt) @ T_est

    # Compute the norm of error
    error_norm = np.linalg.norm(T_error)
    print(T_error)
    print(f"Error norm: {error_norm}")

# File paths
groundtruth_file = '/home/zheng/datasets/public_dataset/ICL-NUIM/L515_test_desk1_icl/14-600_office_table2/groundtruth.txt'
estimated_file = '/home/zheng/projects/ORB_SLAM2/Examples/RGB-D/CameraTrajectory_office_table2.txt'

# Calculate the error
calculate_error(groundtruth_file, estimated_file)
