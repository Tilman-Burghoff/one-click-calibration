This repo contains the necessary scripts to calibrate the 6-d camera pose in relation to the panda in the LIS lab.

To use it, compile the optimization program with
```
cd komo-aruco-calibration
cmake -DCMAKE_INSTALL_PREFIX:PATH=~/.local -B build .
make -C build
```
If you have compiled robotic in another directory, you might have to change the install prefix.

After compiling, just run main.py to collect data and optimize the position.

If you do not use the calibration plate, you have to manually configure the aruco-positions by running ```measure_marker_positions.py``` before starting the calibration process.

To see the possible command line arguments, run ```main.py -h```

This is still WIP, no guarantees that it works out of the box yet :)
