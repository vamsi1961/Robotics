cmake_minimum_required(VERSION 2.8)
project( Planning )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( a_star a_star.cpp )
target_link_libraries( a_star ${OpenCV_LIBS} )

add_executable( Dijkstra Dijkstra.cpp )
target_link_libraries( Dijkstra ${OpenCV_LIBS} )

add_executable( RRT RRT.cpp )
target_link_libraries( RRT ${OpenCV_LIBS} )

