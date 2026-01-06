"""This module provides a class to collect calibration data with the panda robot.
"""

__authors__ = ["Tilman Burghoff"]
__version__ = 0.1


from typing import TypeAlias, Optional
import json

import robotic as ry
from robotic.src import h5_helper

import numpy as np
from cv2 import aruco

from . import Parameters


MarkerPositions: TypeAlias = dict[int, np.ndarray]

class DataCollection:
    """Collects data by tracking aruco-markers from different random poses
    specified by the parameters object.
    """
    def __init__(self, 
                 params: Optional[Parameters]=None):
        
        self.params = params
        self.h5_writer = h5_helper.H5Writer(self.params.output_file)

        self.aruco_dict = aruco.getPredefinedDictionary(params.marker_dict)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        self.C = ry.Config()
        self.C.addFile(self.params.config_file)
        self._setup_markers()

        self.bot = ry.BotOp(self.C, True)
        self.bot.getImageAndDepth(self.params.camera_name) # initialize camera

        self.target = self.C.addFrame('target')

        self.rng = np.random.default_rng(self.params.seed)


    def _setup_markers(self):
        """Reads marker position from .json and places them in the Config."""
        with open(self.params.marker_positions_file, 'r') as f:
            marker_positions = json.load(f, object_hook=self._jsonKeys2int)
        for id, pos in marker_positions.items():
            marker = self.C.addFrame(f'marker_{id}', self.params.panda_prefix+'panda_base')
            marker.setShape(ry.ST.marker, [.05])
            marker.setRelativePosition(pos)
        self.markers = list(marker_positions.keys())


    def _jsonKeys2int(self, x):
        """helper to convert ids, source: https://stackoverflow.com/a/34346202"""
        if isinstance(x, dict):
            return {int(k):v for k,v in x.items()}
        return x


    def run(self):
        """Runs the data-collection. This will move the robot 
        and overwrite the data-file specified in the parameters."""
        i = 0
        while i < self.params.number_of_poses:
            pose = self._generate_pose()
            if pose is None:
                continue
            
            self.bot.moveTo(pose)
            self.bot.wait(self.C)
            self.bot.hold(floating=False)

            try:
                joint_states, marker_coords = self._get_pose_and_coords()
            except RuntimeError as e:
                print(e)
                continue

            self._write_data(i, marker_coords, joint_states)

            i += 1
        self._write_manifest()

    def _generate_pose(self) -> None|np.ndarray:
        """Generates constraints for a pose and tries to solve the resulting komo-problem.
        Returns:
            pose: 
                Pose as array if possible else None
        """
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
        """Generates a komo problem to look at target frame 
        with a specified angle and distance.
        """
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
        """Measures pose as well as visible aruco markers multiple times as specified in params.
        
        Returns:
            Pose and Marker positions as a dict mapping marker id to its (p_x, p_y, d) position
        """
        coords = dict()
        count = dict()
        joint_states = []
        for _ in range(self.params.images_per_pose):
            joint_states.append(self.C.getJointState())
            rgb, depth = self.bot.getImageAndDepth(self.params.camera_name)
            corners, ids, _ = aruco.detectMarkers(rgb, self.aruco_dict, parameters=self.aruco_params)

            if ids is None:
                continue
            
            for id, corner in zip(ids.flatten(), corners):
                if id not in self.params.marker_ids: # filter out artifacts
                    continue

                pixel_coord = corner[0, 0, :].astype(int)
                d = self._bilinear_depth_interpolation(depth, pixel_coord[0], pixel_coord[1])
                if id not in coords:
                    coords[id] = np.concatenate([corner[0, 0, :], [d]])
                    count[id] = 1
                else:
                    coords[id] += np.concatenate([corner[0, 0, :], [d]])
                    count[id] += 1
            
        if len(coords) == 0:
            raise RuntimeError("No markers found")
        
        corners = []
        ids = []
        for id in coords:
            if count[id] < 3:
                continue
            coords[id] /= count[id]
        if len(coords) == 0:
            raise RuntimeError("Not enough successful detections for averaging")
        return np.mean(joint_states, axis=0), coords
    

    def _bilinear_depth_interpolation(self, depth: np.ndarray, x:float, y:float) -> float:
        """Returns interpolated depth according to the (x,y) subpixel coordinates."""
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
        """Writes a single dataset to the data-file specified in the params."""
        self.h5_writer.write(f'dataset_{index}/joint_state', joint_state, dtype='float64')
        self.h5_writer.write(f'dataset_{index}/marker_positions', np.array(list(coords.values())), dtype='float32')
        self.h5_writer.write(f'dataset_{index}/marker_ids', np.array(list(coords.keys())), dtype='int32')


    def _write_manifest(self):
        """Adds the manifest to the data file."""
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