cmake_minimum_required(VERSION 2.8)

project(seperate_singleCalibrate ${PROJECT_NAME_EXTENSION})
set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

list(APPEND SOURCES
        main.cpp
        ../../_libs/xiAPIplusOpenCV/xiApiPlusOcv
        stdafx
        XIMEA
        )

include(../../_libs/_cmake/add_xiapi.cmake)
include(../../_libs/_cmake/add_opencv.cmake)

include(../../_libs/_cmake/add_executable.cmake)


target_link_libraries(seperate_singleCalibrate  ${OpenCV_LIBS})
