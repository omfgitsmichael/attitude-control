set(This simulation)

add_executable(${This} gimbalOne.cpp)
target_link_libraries(${This} PRIVATE
    attitudeEstimationControl
)
