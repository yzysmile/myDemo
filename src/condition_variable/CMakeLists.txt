set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
cmake_minimum_required(VERSION 3.9) 

project(CV_demo) 

add_executable(main main.cpp A.hpp B.hpp) 

 

