project(SignalLocator)
cmake_minimum_required(VERSION 2.8)

file(GLOB SOURCES src/*.cpp)

include_directories("./include")

add_executable(${PROJECT_NAME} ${SOURCES})
