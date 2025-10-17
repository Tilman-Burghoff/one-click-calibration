from robotic.src import h5_helper
from pprint import pprint

h5 = h5_helper.H5Reader('aruco_calibration_data.h5')
pprint(h5.read_dict('manifest'))
h5.print_info()