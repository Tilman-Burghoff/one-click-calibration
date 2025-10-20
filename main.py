from parameters import Parameters
from data_collection import DataCollection
import subprocess
import re
import argparse

def write_g_file(Q):
    with open('pandaSingle_fixedCam.g', 'w') as f:
        f.write('# optimized camera pose\n')
        f.write('Include: <../../../../../../../../$RAI_PATH/scenarios/pandaSingle.g>\n')
        f.write('Edit cameraWrist { Q: '+Q+' }')

def parse_args() -> Parameters:
    parser = argparse.ArgumentParser(description='Collect Data and Optimize Camera Pose')
    for field in Parameters.__dataclass_fields__.values():
        type_string = str(field.type).split("'")[1]
        parser.add_argument(
            f'--{field.name}', 
            type=field.type, 
            default=field.default,
            help=f'{field.name}: {type_string} = {field.default}'
        )
    args = parser.parse_args()
    return Parameters(**vars(args))

def main():
    params = parse_args()
    data_collector = DataCollection(params)
    data_collector.run()
    del data_collector

    out = subprocess.getoutput(f'./optimize.exe --data_file {params.output_file} {"--optimize_joints" if params.optimize_joints else ""}')
    if re.match(r'\[(?:-?0\.\d*, ){6}-?0\.\d*\]\n0\.\d*', out) is None: # expected output format: [x,y,z,qw,qx,qy,qz]\nRMSE
        raise RuntimeError('optimize.exe encountered Error:\n'+out)
    
    pose, err = out.splitlines()
    if float(err) > 0.01:
        raise RuntimeError('high RMSE error:', err, 'pose ', pose, 'is likely unusable')
    print('Optimal camera pose:\n', pose, "\nRMSE error:", err)
    write_g_file(pose)

if __name__ == "__main__":
    main()