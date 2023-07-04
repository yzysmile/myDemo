set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
cmake_minimum_required(VERSION 3.9) 
set(CMAKE_CXX_STANDARD 14)
project(C++evlution) 

add_executable(main main.cpp) 