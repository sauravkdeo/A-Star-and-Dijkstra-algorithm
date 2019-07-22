cmake_minimum_required(VERSION 3.2.1)
project (Project2)

# Add project cmake modules to path.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

find_package( OpenCV REQUIRED )
# highgui imgproc core videoio imgcodecs

# We probably don't want this to run on every build.
option(COVERAGE "Generate Coverage Data" OFF)


include(CMakeToolsHelpers OPTIONAL)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

add_subdirectory(app)

