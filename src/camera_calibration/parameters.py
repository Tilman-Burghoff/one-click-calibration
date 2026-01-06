from dataclasses import dataclass
from cv2 import aruco
from numpy import pi


@dataclass(slots=True)
class Parameters:
    """Parameters for camera calibration and optimization.
    Args:
        number_of_poses:
            Number of different robot poses generated during data collection.
        images_per_pose:
            Number of images captured per robot pose to average marker position.
        min_distance:
            Minimum distance between camera and target in meters for pose generation.
        max_distance:
            Maximum distance between camera and target in meters for pose generation.
        min_angle:
            Minimum angle between camera optical axis and global z-axis in radians for pose generation.
        max_angle:
            Maximum angle between camera optical axis and global z-axis in radians for pose generation.
        max_target_offset:
            Maximum random offset applied to target position in meters for pose generation.
        seed:
            Seed used for pose generation.
        marker_dict:
            cv2.aruco dictionary used for marker detection.
        marker_ids:
            IDs of the ArUco markers used for calibration. Filters out other detected markers.
        output_file:
            Path to the h5 file where the collected data will be stored.
        marker_positions_file:
            Path to the json file containing the marker positions.
        config_file:
            Path to the g-file used for robot and camera control.
        panda_prefix:
            Relevant when working with r_panda.
        camera_name:
            Name of the camera in the g-file.
        optimize_joints:
            Whether the optimizer adds a small offset to the joints which is optimized along with the camera pose.
    """
    number_of_poses: int = 100
    images_per_pose: int = 10
    min_distance: float = .2
    max_distance: float = .7
    min_angle: float = 0
    max_angle: float = 1/4 * pi
    max_target_offset: float = 0.05
    seed: int = 0
    marker_dict: int = aruco.DICT_4X4_100
    marker_ids: tuple[int] = (0, 1, 2, 3, 4, 5, 6, 7, 8)
    output_file: str = 'aruco_calibration_data_test.h5'
    marker_positions_file: str = 'src/camera_calibration/marker_positions.json'
    config_file: str = '/../../../../../$RAI_PATH/scenarios/pandaSingle.g'
    panda_prefix: str = 'l_'
    camera_name: str = 'cameraWrist'
    optimize_joints: bool = False


defaults = Parameters()


if __name__ == "__main__":
    from pprint import pprint
    pprint(defaults)