cmake_minimum_required (VERSION 3.9) 

project (CV_demo) 

find_package(Threads REQUIRED)
target_link_libraries(CV_demo Threads::Threads)

add_executable(main main.cpp A.hpp B.hpp) 

