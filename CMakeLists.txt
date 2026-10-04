cmake_minimum_required(VERSION 3.16)
project(my_robot CXX)
set(CMAKE_CXX_STANDARD 17)

set(WEBOTS_HOME "/Applications/Webots.app/Contents")

add_executable(my_robot main.cpp)
target_include_directories(my_robot PRIVATE
        ${WEBOTS_HOME}/include/controller/cpp
        ${WEBOTS_HOME}/include/controller/c)
target_link_directories(my_robot PRIVATE ${WEBOTS_HOME}/lib/controller)
target_link_libraries(my_robot PRIVATE CppController Controller)
set_target_properties(my_robot PROPERTIES
        BUILD_RPATH "/Applications/Webots.app")