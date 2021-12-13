cmake_minimum_required(VERSION 2.8.3)
project(hw56code)

## Set the package dependencies: Find catkin macros/libraries and
## other required packages.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  urdfdom_py)

find_package(Eigen3 REQUIRED)

## Uncomment if the package has a setup.py
# catkin_python_setup()

## Catkin configuration.  We don't have dependent projects.
catkin_package()


###########
## Build ##
###########

## Specify locations of header files
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

## Declare the C++ executables

## Add cmake target dependencies of the executables

## Specify libraries to link a library or executable target against
