add_executable(gimbalSim gimbal.cpp)
target_link_libraries(gimbalSim PRIVATE
    attitudeEstimationControl
)

add_executable(satellite satellite.cpp)
target_link_libraries(satellite PRIVATE
    attitudeEstimationControl
)
