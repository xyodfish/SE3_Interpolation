include_directories(
        ${INC_DIR}
        ${THIRD_PARTY_INC}/eigen/include
        ${THIRD_PARTY_INC}
        ${THIRD_PARTY_INC}/common
)


link_directories(${THIRD_PARTY_LIB})
        
add_library(
        SE3Interpolation SHARED
        SE3InterpolationSlerp.cpp SE3InterpolationSquad.cpp SE3InterpolationBezier.cpp lieGroupUtils.cpp rigidBodyMotion.cpp
)

target_link_libraries(SE3Interpolation spdlog)

add_library(SE3TrajectoryInterpolator SE3TrajectoryInterpolator.cpp)
target_link_libraries(SE3TrajectoryInterpolator SE3Interpolation)
