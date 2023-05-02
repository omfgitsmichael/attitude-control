set(This filterTests)

set(Sources
    filterTests.cpp
)

add_executable(${This} ${Sources})
target_link_libraries(${This} PRIVATE
    gtest_main
    attitudeEstimationControl
)

add_test(
    NAME ${This}
    COMMAND ${This}
)
