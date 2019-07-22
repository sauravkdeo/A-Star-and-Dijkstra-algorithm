add_executable(Project2 main.cpp node.cpp map.cpp coord.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries( Project2 ${OpenCV_LIBS} )