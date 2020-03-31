cmake_minimum_required(VERSION 3.7)
project(my_solution)

add_subdirectory("${PROJECT_SOURCE_DIR}/../my_library" "${PROJECT_SOURCE_DIR}/my_library_output")
add_subdirectory("${PROJECT_SOURCE_DIR}/../my_application" "${PROJECT_SOURCE_DIR}/my_application_output")