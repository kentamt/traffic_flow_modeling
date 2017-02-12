cmake_minimum_required (VERSION 2.6)


project (asep)
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_executable (asep ASEP.cpp)
target_link_libraries( asep ${OpenCV_LIBS})
