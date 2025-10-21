from dataclasses import dataclass
from pprint import pprint
from numpy import pi


@dataclass(slots=True)
class Parameters:
    number_of_poses: int = 100
    images_per_pose: int = 10
    min_distance: float = .2
    max_distance: float = .7
    min_angle: float = 0
    max_angle: float = 1/4 * pi
    max_target_offset: float = 0.05
    seed: int = 0
    marker_ids: tuple[int] = (0, 1, 2, 3, 4, 5, 6, 7, 8)
    output_file: str = 'aruco_calibration_data.h5'
    marker_positions_file: str = 'marker_positions.json'
    config_file: str = '/../../../../../$RAI_PATH/scenarios/pandaSingle.g'
    panda_prefix: str = 'l_'
    camera_name: str = 'cameraWrist'
    optimize_joints: bool = False


defaults = Parameters()


if __name__ == "__main__":
    pprint(defaults)