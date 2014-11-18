cmake_minimum_required(VERSION 2.8)
project( IC_GUI )

# directories
set (PROJECT_SRC_DIR ../src_gui)
set ( CMAKE_PREFIX_PATH /opt/x86_full/gst)

# add packages
set(OpenCV_DIR "/opt/x86_full/gst/")
find_package( OpenCV REQUIRED )
include_directories("/opt/x86_full/gst/include/opencv/" ${OpenCV_INCLUDE_DIRS})

# sources
FILE(GLOB MY_SRC_FILES "../src_gui/*.cpp" "../src_gui/*.h" "../src_gui/*.c")

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-std=c++11")

# build demo program
add_executable( IC_GUI ${MY_SRC_FILES})

target_link_libraries( IC_GUI ${OpenCV_LIBS} )

