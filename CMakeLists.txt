cmake_minimum_required(VERSION 2.8)

set(
    CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_HOME_DIRECTORY}/bin
    )

add_definitions(
    -std=c++11
    )

find_package( OpenCV 2 REQUIRED )
find_package( PCL 1.7 REQUIRED )
find_package( Eigen3 REQUIRED )
add_definitions( ${PCL_DEFINITIONS} )

include_directories(
    ${PCL_INCLUDE_DIRS}
    $ENV{OPENNI2_INCLUDE}
    ${OpenCV_INCLUDE_DIRS}
    /usr/include
    /usr/include/libusb-1.0/
    ${CMAKE_HOME_DIRECTORY}/include
    )

link_directories(
    /usr/local/lib
    )

link_libraries(
    -lopencv_objdetect
    -lopencv_core
    -lopencv_highgui
    -lopencv_imgproc
    -lpthread
    -lOpenNI2
    -lboost_system
    -lfreenect
    ${PCL_LIBRARY_DIRS}
    ${FREENECT_LIBRARIES}
    )

add_executable(
    depthmap_to_pcd
    src/KinectV1Sensor.cpp
    src/OpenNIDepthSensor.cpp
    src/DepthCameraReader.cpp
    src/Utils.cpp
    src/OpenCVUtils.cpp
    src/PCLUtils.cpp
    )

target_link_libraries(depthmap_to_pcd ${PCL_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES})
