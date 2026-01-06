cmake_minimum_required(VERSION 3.5)

project(komo_calibration_solver)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")
set(PY_VERSION 3.12 CACHE STRING "python version")
# set(EXECUTABLE_OUTPUT_PATH ../../)



# if(DEFINED ENV{VIRTUAL_ENV})
#   set(PY_LIB_SITE "$ENV{VIRTUAL_ENV}/lib/python${PY_VERSION}/site-packages")
# else()
#   set(PY_LIB_SITE "${CMAKE_INSTALL_PREFIX}/lib/python${PY_VERSION}/site-packages")
# endif()

add_compile_options(
  -O3 -g -Wall -Wpedantic -std=c++17
  )

################################################################################

find_package(Python REQUIRED COMPONENTS Interpreter)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -c
          "import robotic, pathlib; print(pathlib.Path(robotic.__file__).parent)"
  OUTPUT_VARIABLE ROBOTIC_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "robotic include dir: ${ROBOTIC_DIR}")

#message(STATUS "[rai] python site: " ${PY_LIB_SITE})

include_directories(
  ${ROBOTIC_DIR}/include/rai
#  /usr/include/python3.10/include
#  $ENV{HOME}/git/rai/src
  )

link_directories(
  ${ROBOTIC_DIR}
#  $ENV{HOME}/git/rai/build
  )

################################################################################

add_executable(optimize.exe main.cpp calibration.cpp)

target_link_libraries(optimize.exe PRIVATE rai)

set_target_properties(optimize.exe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${SKBUILD_PLATLIB_DIR}/camera_calibration"
)

install(FILES ${ROBOTIC_DIR}/librai.so
        DESTINATION "${SKBUILD_PLATLIB_DIR}/camera_calibration")

set_target_properties(optimize.exe PROPERTIES
    BUILD_RPATH "$ORIGIN"  # $ORIGIN = folder containing executable
)