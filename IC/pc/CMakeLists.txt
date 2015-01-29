cmake_minimum_required(VERSION 2.8)
project( IC )

# directories
set (LIBELAS_SRC_DIR ../src/Geiger_src)
set (PROJECT_SRC_DIR ../src)
#set ( CMAKE_PREFIX_PATH /opt/x86_full/gst) #can be done from command line

# add packages

find_package( OpenCV REQUIRED )
#MESSAGE( STATUS "OpenCV_DIR: " ${OpenCV_DIR} )
#MESSAGE( STATUS "OpenCV_LIBS: " ${OpenCV_LIBS} )
#MESSAGE( STATUS "OpenCV_LIB_DIR: " ${OpenCV_LIB_DIR} )
#MESSAGE( STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS} )
#MESSAGE( STATUS "OpenCV_COMPUTE_CAPABILITIES: " ${OpenCV_COMPUTE_CAPABILITIES} )
#MESSAGE( STATUS "OpenCV_SHARED: " ${OpenCV_SHARED} )
#MESSAGE( STATUS "OpenCV_CONFIG_PATH: " ${OpenCV_CONFIG_PATH} )
#MESSAGE( STATUS "OpenCV_INSTALL_PATH: " ${OpenCV_INSTALL_PATH} )

MESSAGE( STATUS "OPENCV_VERSION: " ${OpenCV_VERSION} )

SET(CMAKE_MODULE_PATH ~/caffe/qtcreator-build)
find_package(Caffe REQUIRED)
include_directories(${Caffe_INCLUDE_DIRS})
MESSAGE( STATUS "Caffe_INCLUDE_DIRS: " ${Caffe_INCLUDE_DIRS} )
add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY
MESSAGE( STATUS "Caffe_DEFINITIONS: " ${Caffe_DEFINITIONS} )
#MESSAGE( STATUS "Caffe_LIBRARIES: " ${Caffe_LIBRARIES} )

include_directories("${LIBELAS_SRC_DIR}")

# sources
FILE(GLOB LIBELAS_SRC_FILES "../src/Geiger_src/*.cpp")
FILE(GLOB MY_SRC_FILES "../src/*.cpp" "../src/*.h" "../src/*.c")

# use sse3 instruction set for libelas and c11 for threading (among others)
SET(CMAKE_CXX_FLAGS "-msse3 -std=c++11")
#SET(CMAKE_CXX_FLAGS "-msse3 -std=c++0x -pthread")

#detect whether we build for ARM drone or pc
IF (NOT DEFINED _PC)
    SET(_PC TRUE)
ENDIF()
ADD_DEFINITIONS(-D_PC=${_PC})


# build program
add_executable( IC ${LIBELAS_SRC_FILES} ${MY_SRC_FILES})

target_link_libraries( IC ${OpenCV_LIBS} ${Caffe_LIBRARIES} )


