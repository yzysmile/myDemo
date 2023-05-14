cmake_minimum_required (VERSION 3.9) 
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
project (Variadic_template) 

add_executable(variadic_template maximum.cpp) 