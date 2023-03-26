cmake_minimum_required (VERSION 3.9) 
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
project (PF_demo) 

add_executable(main main.cpp consumer.cpp producer.cpp) 
