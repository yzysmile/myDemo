cmake_minimum_required (VERSION 3.9) 

project (CV_demo) 

set(CMAKE_CXX_FLAGS " -std=c++11 -march=native -O3 -pthread" )

add_executable(main main.cpp A.hpp B.hpp) 

