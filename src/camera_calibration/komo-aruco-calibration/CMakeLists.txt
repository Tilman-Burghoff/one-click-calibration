cmake_minimum_required(VERSION 3.5)

project(komo_calibration_solver)

set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")
set(PY_VERSION 3.10 CACHE STRING "python version")
set(EXECUTABLE_OUTPUT_PATH ../../)



if(DEFINED ENV{VIRTUAL_ENV})
  set(PY_LIB_SITE "$ENV{VIRTUAL_ENV}/lib/python${PY_VERSION}/site-packages")
else()
  set(PY_LIB_SITE "${CMAKE_INSTALL_PREFIX}/lib/python${PY_VERSION}/site-packages")
endif()

add_compile_options(
  -O3 -g -Wall -Wpedantic -std=c++17
  )

################################################################################

find_package(Python REQUIRED)

message(STATUS "[rai] python site: " ${PY_LIB_SITE})

include_directories(
  ${PY_LIB_SITE}/robotic/include/rai
#  /usr/include/python3.10/include
#  $ENV{HOME}/git/rai/src
  )

link_directories(
  ${PY_LIB_SITE}/robotic/
#  $ENV{HOME}/git/rai/build
  )

################################################################################

add_executable(optimize.exe main.cpp calibration.cpp)

target_link_libraries(optimize.exe rai)
