cmake_minimum_required(VERSION 3.12)
set(CMAKE_CXX_STANDARD 17)

# Set project name
set(This attitudeEstimationControl)

project(${This} VERSION 1.0.0)

set(PROJECT_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Enable CMake tests
enable_testing()

# Add third party directories
add_subdirectory(thirdParty)

# Add project library
add_library(${This} INTERFACE)

target_include_directories(${This} INTERFACE
    ${PROJECT_BASE_DIR}/include
    ${PROJECT_BASE_DIR}/thirdParty/eigen
)

target_link_libraries(${This} INTERFACE
  eigen
)

# Add unit tests and sim directories
add_subdirectory(tests)
add_subdirectory(sim)
