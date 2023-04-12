set(UTILS ${CMAKE_CURRENT_SOURCE_DIR}/utils)

add_library(attitude-control-libs
  ${UTILS}/attitudeUtils.cpp
)

target_include_directories(attitude-control-libs PUBLIC
    ${PROJECT_BASE_DIR}/include
    ${PROJECT_BASE_DIR}/thirdParty/eigen
)

target_link_libraries(attitude-control-libs PUBLIC
  eigen
)