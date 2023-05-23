
add_executable(matrix_arithmetic matrix_arithmetic.cpp)

add_executable(isometry3d isometry3d.cpp)
target_link_libraries(isometry3d ${OpenCV_LIBRARIES})

add_executable(mat mat.cpp)
target_link_libraries(mat ${OpenCV_LIBRARIES})