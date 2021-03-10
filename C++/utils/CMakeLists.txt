include_directories(../include)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/utils)

# 
add_executable(mytest test.cpp)

# 
add_executable(record_video record_video.cpp)
target_link_libraries(record_video ${OpenCV_LIBS} ${INDEMIND_LIBS})

#
add_executable(take_photo take_photo.cpp)
target_link_libraries(take_photo ${OpenCV_LIBS} ${INDEMIND_LIBS})

#
add_executable(calibrate_camera calibrate_camera.cpp)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} ${INDEMIND_LIBS})

# 
add_executable(test_eigen testEigen.cpp)

#
add_executable(test_fisheye testFisheye.cpp)
target_link_libraries(test_fisheye ${OpenCV_LIBS})

#
add_executable(test_yaml testYaml.cpp)
target_link_libraries(test_yaml ${Yaml_LIBS})