#include "calibration.h"
#include <Core/util.h>
#include <pybind11/pybind11.h>



//===========================================================================

int main(int argc, char** argv){
  rai::initCmdLine(argc, argv);
  rai::setRaiPath("$HOME/git/rai-robotModels");

  bool optimize_joints = rai::checkCmdLineTag("opt-joints");
  char* data_file = rai::getCmdLineArgument("data-file");
  // rnd.seed_random();

  // display_data();
  CalibFromArucos cal(optimize_joints);
  cal.load_data(data_file!=nullptr ? data_file : "aruco_calibration_data.h5");
  // cal.display_data();
  cal.solve();

  return 0;
}


/*

 with joint calib: [-0.0234164, 0.0460334, 0.166626, 0.393056, 0.00504293, -0.0104293, -0.919442]
 w/o  joint calib: [-0.0239713, 0.0481723, 0.16886 , 0.39354 , 0.00971287, -0.0028329, -0.919252]
*/
