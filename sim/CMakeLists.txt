add_executable(stationaryAttitude gimbalOne.cpp)
target_link_libraries(stationaryAttitude PRIVATE
    attitudeEstimationControl
)

add_executable(gimbalOmegaOnly gimbalTwo.cpp)
target_link_libraries(gimbalOmegaOnly PRIVATE
    attitudeEstimationControl
)
