set(CMAKE_CXX_STANDARD 14)



include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)

add_executable (architecture_test unit/Fibre_t.cpp src/Fibre.cpp)
target_link_libraries(architecture_test gtest_main)
add_test(architecture_test architecture_test)