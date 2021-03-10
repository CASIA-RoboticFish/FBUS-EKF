cmake_minimum_required(VERSION 3.0)
# fiducial-based underwater stereo visual-inertial ekf
project(FBUS_EKF VERSION "1.0.0" LANGUAGES CXX)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

# Third Party Liabrary
# Eigen
find_package(Eigen3)
include_directories(${EIGEN3_INCLUDE_DIR})
if(Eigen3_FOUND)
    message(STATUS "Eigen3 FOUND: ${EIGEN3_INCLUDE_DIR}")
endif()

# OpenCV
set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OPENCV FOUND: ${OpenCV_INCLUDE_DIRS}")
    include_directories( ${OpenCV_INCLUDE_DIRS} )
else()
    message(STATUS "OPENCV NOT FOUND. Please check the path of OpenCV_DIR")
endif()

# Aruco
set(aruco_DIR "/usr/local/share/aruco")
find_package(aruco REQUIRED)
if(aruco_FOUND)
    message(STATUS "ARUCO FOUND: ${aruco_INCLUDE_DIRS}")
else()
    message(STATUS "ARUCO NOT FOUND. Please check the path of aruco_DIR")
endif()

# Yaml
set(Yaml_INCLUDE_DIRS "/usr/local/include")
set(Yaml_LIBRARY_DIRS "/usr/local/lib")
set(Yaml_LIBS yaml-cpp)
include_directories(${Yaml_INCLUDE_DIRS})
link_directories(${Yaml_LIBRARY_DIRS})
message(STATUS "YAML FOUND: ${Yaml_INCLUDE_DIRS}")


# Pangolin
find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
    include_directories(${Pangolin_INCLUDE_DIRS})
    message(STATUS "PANGOLIN FOUND: ${Pangolin_INCLUDE_DIRS}")
else()
    message(STATUS "PANGOLIN NOT FOUND. Please check the path of Pangolin")
endif()

# IMSEE-SDK
set(INDEMIND_LIB_DIR ${PROJECT_SOURCE_DIR}/driver/IMSEE-SDK/lib/x86-64)
include_directories(${PROJECT_SOURCE_DIR}/driver)
set(INDEMIND_LIBS ${INDEMIND_LIB_DIR}/libindemind.so)
message(STATUS "IMSEE-SDK FOUND")


# 
add_subdirectory(src)
add_subdirectory(utils)
