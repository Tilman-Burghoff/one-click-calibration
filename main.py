from data_collection import DataCollection, Parameters
import subprocess
import re

def write_g_file(Q):
    with open('pandaSingle_fixedCam.g', 'w') as f:
        f.write('# optimized camera pose\n')
        f.write('Include: <../../../../../../../../$RAI_PATH/scenarios/pandaSingle.g>\n')
        f.write('Edit cameraWrist { Q: '+Q+' }')

def main():
    params = Parameters() # TODO read in command line arguments
    data_collector = DataCollection(params)
    data_collector.run()
    del data_collector

    out = subprocess.getoutput(f'./optimize.exe')
    if re.match(r'\[(?:-?0\.\d*, ){6}-?0\.\d*\]\n0\.\d*', out) is None: # expected output format: [x,y,z,qw,qx,qy,qz]\nRMSE
        raise RuntimeError('optimize.exe encountered Error:\n'+out)
    
    pose, err = out.splitlines()
    if float(err) > 0.01:
        raise RuntimeError('high RMSE error:', err, 'pose ', pose, 'is likely unusable')
    print('Optimal camera pose:\n', pose, "\nRMSE error:", err)
    write_g_file(pose)

if __name__ == "__main__":
    main()