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
     DEPENDS system_lib
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
target_link_libraries(eis_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(camera_model_test     src/camera_model_test.cpp    )
#target_link_libraries(camera_model_test  bfd  ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(reproject_test     src/reproject_test.cpp    )
#target_link_libraries(reproject_test  ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(fisheyeMaskGenerate  src/fisheyeMaskGenerate.cpp    )
#target_link_libraries(fisheyeMaskGenerate  bfd  ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(reproject_test2     src/reproject_test_gpu.cpp    )
#target_link_libraries(reproject_test2    ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(fastTestUp     src/fisheye/omniFisheyeCropper.cpp
#    src/fast_test_up.cc    )
#target_link_libraries(fastTestUp     ${catkin_LIBRARIES}    ${OpenCV_LIBS})

#add_executable(fastTestDown     src/fisheye/omniFisheyeCropper.cpp
#    src/fast_test_down.cc    )
#target_link_libraries(fastTestDown     ${catkin_LIBRARIES}    ${OpenCV_LIBS})

#add_executable(omniFisheyeCropper     src/fisheye/omniFisheyeCropper.cpp
#    src/omniFisheye_rosnode.cpp    )
#target_link_libraries(omniFisheyeCropper     ${catkin_LIBRARIES}    ${OpenCV_LIBS})

#add_executable(omniFisheyeCropper_jfr     src/fisheye/omniFisheyeCropper.cpp
#    src/omniFisheye_rosnode_jfr.cpp    )
#target_link_libraries(omniFisheyeCropper_jfr     ${catkin_LIBRARIES}    ${OpenCV_LIBS})

#add_executable(frontFisheyeCropper     src/fisheye/frontFisheyeCropper.cpp
#    src/frontFisheye_rosnode.cpp    )
#target_link_libraries(frontFisheyeCropper     ${catkin_LIBRARIES}    ${OpenCV_LIBS})

#add_executable(frontFisheyeCropper_jfr     src/fisheye/frontFisheyeCropper.cpp
#    src/frontFisheye_rosnode_jfr.cpp    )
#target_link_libraries(frontFisheyeCropper_jfr     ${catkin_LIBRARIES}    ${OpenCV_LIBS})

#add_executable(testPub     src/testPub.cpp    )
#target_link_libraries(testPub    ${catkin_LIBRARIES}    ${OpenCV_LIBS})

#add_executable(testPubImg     src/testPubImg.cc    )
#target_link_libraries(testPubImg    ${catkin_LIBRARIES}    ${OpenCV_LIBS})

#add_executable(fisheyePreprocess    src/fisheyePreprocess.cc )
#target_link_libraries(fisheyePreprocess    ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(debugPolynomialError     src/debugPolynomialError.cpp    )
#target_link_libraries(debugPolynomialError    ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(stereoCropper     src/fisheye/stereoCropper.cpp
#    src/stereo_rosnode.cpp    )
#target_link_libraries(stereoCropper     ${catkin_LIBRARIES}    ${OpenCV_LIBS})

#add_executable(transpose_view     src/transpose_view.cpp    )
#target_link_libraries(transpose_view    ${catkin_LIBRARIES}    ${OpenCV_LIBS})

