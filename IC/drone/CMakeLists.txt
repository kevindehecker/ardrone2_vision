cmake_minimum_required(VERSION 2.8)
project( IC )

# directories
set (PROJECT_SRC_DIR ../src)
set ( CMAKE_PREFIX_PATH /opt/arm_light/gst)

# sources
FILE(GLOB MY_SRC_FILES "../src/*.cpp" "../src/*.h" "../src/*.c")

#set compile flags, for threading
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pthread")

# add packages
set(OpenCV_DIR "/opt/arm_light/gst/")
find_package( OpenCV REQUIRED )

# include directory
include_directories("/opt/arm_light/gst/include/" "/opt/arm_light/gst/include/opencv/" ${OpenCV_INCLUDE_DIRS})

# build demo program
add_executable( IC ${LIBELAS_SRC_FILES} ${MY_SRC_FILES})

target_link_libraries( IC ${OpenCV_LIBS} -lrt)
