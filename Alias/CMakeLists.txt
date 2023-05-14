cmake_minimum_required (VERSION 3.9) 
set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
project (Alias_template) 

add_executable(template_alias template_alias.cpp) 
