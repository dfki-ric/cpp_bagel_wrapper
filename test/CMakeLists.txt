add_executable(test_graph test_graph.cpp ../src/BagelGraph.cpp)
target_link_libraries(test_graph ${PKGCONFIG_LIBRARIES})
install(FILES test_graph.yml DESTINATION ${CMAKE_BINARY_DIR}/test)

enable_testing()
add_test(test_graph test_graph)
