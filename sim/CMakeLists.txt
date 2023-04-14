set(This simulation)

add_executable(${This} sim.cpp)
target_link_libraries(${This} PRIVATE
    attitudeEstimationControl
)
