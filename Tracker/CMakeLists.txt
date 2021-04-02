#####################################
cmake_minimum_required (VERSION 2.8) 
project (tracker)
find_package(raspicam REQUIRED)
add_executable (tracker main.cpp)  
target_link_libraries (tracker ${raspicam_LIBS})
#####################################
