include_directories(../include)

# 
set(sources main.cpp filter.cpp vision.cpp visualizer.cpp GlogHelper.cpp)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin)
add_executable(${PROJECT_NAME} ${sources})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${aruco_LIBS} ${Yaml_LIBS} ${Pangolin_LIBRARIES} ${INDEMIND_LIBS} -lpthread -lm -lglog)

