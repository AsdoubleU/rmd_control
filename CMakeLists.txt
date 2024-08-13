cmake_minimum_required(VERSION 3.0.2)
project(rmd_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 COMPONENTS Core REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rmd_control
  CATKIN_DEPENDS 
    cmake_modules 
    geometry_msgs 
    roscpp 
    sensor_msgs 
    std_msgs 
  DEPENDS 
    Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


# add_message_files(
#   DIRECTORY msg
#   FILES
#   CustomJointState.msg)

# generate_messages(DEPENDENCIES geometry_msgs std_msgs)

add_executable(
  rmd_control 
  src/main.cpp 
  src/rt_utils.cpp 
  src/rmd_can.cpp
  src/spi2can.cpp
  src/rmd_motor.cpp
  src/motor_controller.cpp
  src/TrajectoryGenerator.cpp
  src/callback.cpp)
add_dependencies(rmd_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rmd_control ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} Qt5::Core ${RBDL_LIBRARY})

install(TARGETS rmd_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/update_spi_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}  
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)