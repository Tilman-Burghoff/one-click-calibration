from dataclasses import dataclass
from typing import TypeAlias
import json

import robotic as ry
from robotic.src import h5_helper

import numpy as np
from cv2 import aruco


MarkerPositions: TypeAlias = dict[int, np.ndarray]

@dataclass
class Parameters:
    number_of_poses: int = 5
    images_per_pose: int = 10
    min_distance: float = .2
    max_distance: float = .7
    min_angle: float = 0
    max_angle: float = 1/4 * np.pi
    max_target_offset: float = 0.05
    seed: int = 0
    output_file: str = 'aruco_calibration_data.h5'
    marker_positions_file: str = 'marker_positions.json'
    config_file: str = '/../../../../../$RAI_PATH/scenarios/pandaSingle.g'
    panda_prefix: str = 'l_'
    camera_name: str = 'cameraWrist'

defaults = Parameters()

class DataCollection:
    def __init__(self, 
                 params: Parameters=defaults):
        
        self.params = params
        self.h5_writer = h5_helper.H5Writer(self.params.output_file)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        self.C = ry.Config()
        self.C.addFile(self.params.config_file)
        self._setup_markers()

        self.bot = ry.BotOp(self.C, True)
        self.bot.getImageAndDepth(self.params.camera_name) # initialize camera

        self.target = self.C.addFrame('target')

        self.rng = np.random.default_rng(self.params.seed)


    def _setup_markers(self):
        with open(self.params.marker_positions_file, 'r') as f:
            marker_positions = json.load(f, object_hook=self._jsonKeys2int)
        for id, pos in marker_positions.items():
            marker = self.C.addFrame(f'marker_{id}', self.params.panda_prefix+'panda_base')
            marker.setShape(ry.ST.marker, [.05])
            marker.setRelativePosition(pos)
        self.markers = list(marker_positions.keys())


    def _jsonKeys2int(self, x):
        # https://stackoverflow.com/a/34346202
        if isinstance(x, dict):
            return {int(k):v for k,v in x.items()}
        return x


    def run(self):
        i = 0
        while i < self.params.number_of_poses:
            pose = self._generate_pose()
            if pose is None:
                continue
            
            self.bot.moveTo(pose)
            self.bot.wait(self.C)
            self.bot.hold(floating=False)

            result = self._get_pose_and_coords()
            if result is None:
                continue

            joint_states, marker_coords = result
            self._write_data(i, marker_coords, joint_states)

            i += 1
        self._write_manifest()

    def _generate_pose(self) -> None|np.ndarray:
        marker_id = self.rng.choice(self.markers)
        offset = self.rng.random(2) * self.params.max_target_offset
        self.target.setPosition(self.C.getFrame(f'marker_{marker_id}').getPosition() + np.concatenate([offset, [0]]))

        angle = self.rng.random(1)[0] * (self.params.max_angle - self.params.min_angle) + self.params.min_angle
        distance = self.rng.random(1)[0] * (self.params.max_distance - self.params.min_distance) + self.params.min_distance

        komo = self._look_with_angle(distance, angle)
        ret = ry.NLP_Solver(komo.nlp(), verbose=-1).solve()
        if not ret.feasible:
            print('No feasible pose found')
            return None
        path = komo.getPath()
        return path[-1]
    

    def _look_with_angle(self, distance: float, angle: float) -> ry.KOMO:
        komo = ry.KOMO(self.C, 1, 1, 0, True)
        
        komo.addControlObjective([], 0, 1e-1)

        height = distance * np.cos(angle)


        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq, [1])
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

        komo.addObjective([], ry.FS.negDistance, [self.params.camera_name, 'target'], ry.OT.eq, [1], [-distance]) 
        komo.addObjective([], ry.FS.positionDiff, [self.params.camera_name, 'target'], ry.OT.eq, [0,0,1], [0,0,height])
        # point camera at target
        komo.addObjective([], ry.FS.positionRel, ['target', self.params.camera_name], ry.OT.eq, [[1,0,0],[0,1,0]])

        return komo


    def _get_pose_and_coords(self) -> None|tuple[np.ndarray, MarkerPositions]:
        coords = dict()
        count = dict()
        joint_states = []
        for _ in range(self.params.images_per_pose):
            joint_states.append(self.C.getJointState())
            rgb, depth = self.bot.getImageAndDepth(self.params.camera_name)
            corners, ids, _ = aruco.detectMarkers(rgb, self.aruco_dict, parameters=self.aruco_params)
            if ids is None:
                return None
            for id, corner in zip(ids.flatten(), corners):
                pixel_coord = corner[0, 0, :].astype(int)
                d = self._bilinear_depth_interpolation(depth, pixel_coord[0], pixel_coord[1])
                if id not in coords:
                    coords[id] = np.concatenate([corner[0, 0, :], [d]])
                    count[id] = 1
                else:
                    coords[id] += np.concatenate([corner[0, 0, :], [d]])
                    count[id] += 1
            
        if len(coords) == 0:
            print('No markers detected')
            return None
        
        corners = []
        ids = []
        for id in coords:
            if count[id] < 3:
                continue
            coords[id] /= count[id]
        if len(coords) == 0:
            print('No markers detected')
            return None
        return np.mean(joint_states, axis=0), coords
    

    def _bilinear_depth_interpolation(self, depth: np.ndarray, x:float, y:float) -> float:
        x0 = int(np.floor(x))
        x1 = min(x0 + 1, depth.shape[1] - 1)
        y0 = int(np.floor(y))
        y1 = min(y0 + 1, depth.shape[0] - 1)

        Q11 = depth[y0, x0]
        Q21 = depth[y0, x1]
        Q12 = depth[y1, x0]
        Q22 = depth[y1, x1]

        if np.isnan(Q11) or np.isnan(Q21) or np.isnan(Q12) or np.isnan(Q22):
            return np.nan

        return (Q11 * (x1 - x) * (y1 - y) +
                Q21 * (x - x0) * (y1 - y) +
                Q12 * (x1 - x) * (y - y0) +
                Q22 * (x - x0) * (y - y0))


    def _write_data(self, index: int, coords: MarkerPositions, joint_state: np.ndarray):
        self.h5_writer.write(f'dataset_{index}/joint_state', joint_state, dtype='float64')
        self.h5_writer.write(f'dataset_{index}/marker_positions', np.array(list(coords.values())), dtype='float32')
        self.h5_writer.write(f'dataset_{index}/marker_ids', np.array(list(coords.keys())), dtype='int32')


    def _write_manifest(self):
        manifest = {
        'description': 'for various poses: joint state of the panda and ids and and positions (first corner) of arUco markers as (p_x, p_y, d) coordinates. The parameters entry contains the parameters used for data collection.',
        'n_datasets': self.params.number_of_poses,
        'marker_ids': self.markers,
        'keys': ['manifest', 'dataset_[i]/joint_state', 'dataset_[i]/marker_positions', 'dataset_[i]/marker_ids'],
        'parameters': {
            'NUMBER_OF_POSES': self.params.number_of_poses,
            'IMAGES_PER_POSE': self.params.images_per_pose,
            'MIN_DISTANCE': self.params.min_distance,
            'MAX_DISTANCE': self.params.max_distance,
            'MIN_ANGLE': self.params.min_angle,
            'MAX_ANGLE': self.params.max_angle,
            'MAX_TARGET_OFFSET': self.params.max_target_offset,
            'SEED': self.params.seed
            }
        }
        self.h5_writer.write_dict('manifest', manifest)

            

if __name__ == '__main__':
    data_collector = DataCollection()
    data_collector.run()