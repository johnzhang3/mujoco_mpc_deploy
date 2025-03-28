cmake_minimum_required(VERSION 3.0.2)
project(mocap_optitrack)

# Require c++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
  tf)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/MocapOptitrack.cfg
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  dynamic_reconfigure
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

# add_library(mocap_optitrack
#   src/VelocityEstimatorFilter.cpp
#   src/rigid_body_publisher.cpp
#   src/data_model.cpp
#   src/mocap_config.cpp
#   src/mocap_node.cpp
#   src/socket.cpp
#   src/version.cpp
#   # Add other source files if needed
# )

# add_executable(mocap_node
#   src/mocap_node.cpp
#   # Other source files for the executable if needed
# )

# target_link_libraries(
#   include/velocity_estimator.h
#   ${catkin_LIBRARIES}
# )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/mocap.launch)

  find_package(roslint REQUIRED)
  file(GLOB LINT_SRCS
    src/*.cpp
    include/*.h)
  set(ROSLINT_CPP_OPTS "--filter=-runtime/string,-readability/casting,-runtime/references")
  roslint_cpp(${LINT_SRCS})
  roslint_add_test()
endif()
