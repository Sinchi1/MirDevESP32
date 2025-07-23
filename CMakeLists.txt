cmake_minimum_required(VERSION 3.10.0)
project(esp32 VERSION 0.1.0 LANGUAGES C CXX)

add_executable(esp32 main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
