cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)

project(attitude-estimation-control VERSION 1.0.0)

set(PROJECT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Add a list of subdirectories #
add_subdirectory(thirdParty)
add_subdirectory(src)

add_executable(test main.cpp)
target_link_libraries(test PRIVATE attitude-control-libs)
