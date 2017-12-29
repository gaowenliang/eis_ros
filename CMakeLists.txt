cmake_minimum_required(VERSION 2.8.3)
project(eis_ros)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(catkin REQUIRED COMPONENTS
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    cv_bridge
    camera_model
    image_cropper
    code_utils
    )

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(Ceres REQUIRED)


catkin_package(
     INCLUDE_DIRS include
     LIBRARIES eis
     CATKIN_DEPENDS image_transport roscpp sensor_msgs std_msgs
#     DEPENDS system_lib
    )

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
    ${camera_model_INCLUDE_DIRS}
    ${code_utils_INCLUDE_DIRS}
    ${image_cropper_INCLUDE_DIRS}
    )

include_directories("include")

add_library(eis src/eis.cpp  )
target_link_libraries(eis ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(eis_node src/eis_ros_node.cpp src/eis.cpp  )
target_link_libraries(eis_node dw  ${catkin_LIBRARIES} ${OpenCV_LIBS})

