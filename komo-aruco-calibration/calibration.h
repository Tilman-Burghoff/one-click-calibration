#include <Core/array.h>
#include <Kin/kin.h>


void decomposeProjectionMatrix(arr& K, arr& R, arr& t, const arr& P);

void computeCalibration(const arr& X3, const arr& p, const arr& d);

struct CalibFromArucos{
  //data
  arrA Q, P;
  uintAA I;
  //calibration configuration
  rai::Configuration C;
  FrameL Fjoint_calibs, Farucos;
  rai::Frame* Fcam;
  DofL stableDofs, qDofs;

  rai::String aruco_parent = rai::String("table"); 

  //parameters
  double fx = 318.814, fy = 318.814, px = 320.990, py = 177.566;
  // double fx = 318.564, fy = 316.479, px = 321.245, py = 177.673;
  // double fx = 317.793, fy = 317.512, px = 321.481, py = 177.669;
  // [-0.0225989, 0.0514454, 0.169145, 0.389763, 0.00753591, -0.00500915, -0.920871]
  uintA aruco_ids = {0,1,2,3,4,5,6,7,8};
  // uintA blockList = {25, 21, 24};
  uintA blockList = {};

  CalibFromArucos(bool calibJoints = true);

  void load_data(const char* file="aruco_calibration_data.h5");
  void display_data();
  void solve();
  void calibFullProjectionMatrix(const arr& q_opt);
};
