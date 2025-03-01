import sys
from pathlib import Path

# Path where `tpnp_cpp` Python binding has been generated.
# In this case, `tpnp_cpp` is defined in a shared library that looks like:
# tpnp_cpp.cpython-<python-version>-<architecture>-<platform>.so
sys.path.append(str(Path.cwd() / "build/install/lib"))

import numpy as np

# `tpnp_cpp` depends on OpenCV C++ shared libraries.
# Make sure that OpenCV C++ libraries are found by the dynamic link loader
# in order to import `tpnp_cpp` correctly
import tpnp_cpp


def main():
    ## DEFINE PARAMETERS

    # Define runway parameters
    runway_true_heading = -60.9843658153
    runway_ltp_lla = np.array([45.58051475, -122.58390175, -15.0677304067])
    runway_keypoints_lla = np.array(
        [
            [45.59496878, -122.62161525, -15.08052988],
            [45.59532827, -122.62133041, -15.08052988],
            [45.58069445, -122.58375925, -15.06773041],
            [45.58033505, -122.58404425, -15.06773041],
        ]
    )

    # Define camera parameters
    cam_intrinsic = np.array(
        [
            [7.41050131e03, 0, 2.04698374e03],
            [0, 7.41050131e03, 1.50325490e03],
            [0, 0, 1],
        ]
    )
    cam_dist = np.array([0.10571245, 0, 0, 0, 0])
    cam_offset = np.array([0.12984, -0.02857, -0.918739606368141])
    cam_attitude_rpy = np.array([0.02046669, -0.26113752, -0.76237039])

    # Define A/C parameters
    ac_attitude_rpy = np.array([-2.13718960e-01, -7.03531966, 3.02248654e02])

    # Define detected 2D points
    points_2d = np.array(
        [
            [1693.96928761, 674.34990729],
            [1772.79367443, 674.51209595],
            [1917.81319812, 1075.57163956],
            [1574.07248558, 1075.79556393],
        ]
    )

    # Define pose estimator
    pose_estimator = tpnp_cpp.Pose()

    ## COMPUTE A/C POSITION

    # Define pose estimator setup
    pose_setup = tpnp_cpp.PoseSetup(
        runway_true_heading,
        runway_ltp_lla,
        cam_intrinsic,
        cam_dist,
        cam_offset,
        cam_attitude_rpy,
    )

    # Define pose estimator input
    pose_input_attitude = tpnp_cpp.PoseInput(
        points_2d, runway_keypoints_lla, ac_attitude_rpy
    )
    pose_input_no_attitude = tpnp_cpp.PoseInput(points_2d, runway_keypoints_lla)

    # Setup pose estimator
    pose_estimator.setup(pose_setup)

    # Compute A/C position using TPnP
    xyz_tpnp = pose_estimator.compute_pose(pose_input_attitude)

    # Compute A/C position using OpenCV PnP
    xyz_opencvpnp = pose_estimator.compute_pose(pose_input_no_attitude)

    ## DISPLAY RESULTS

    expected_xyz = np.array([-986.509, 2.05654, 68.8754])

    print("[PYTHON EXAMPLE RESULTS]")
    print(f"GT: {expected_xyz}")
    print(f"AnP: {xyz_tpnp}")
    print(f"OpenCV PnP: {xyz_opencvpnp}")


if __name__ == "__main__":
    main()
