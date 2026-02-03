# =============================================================================
# this root cmakelists.txt is the "entrypoint" to the firmware one in
# src/cmakelists.txt, and the unittest one in tests/cmakelists.txt
# =============================================================================
cmake_minimum_required(VERSION 3.22)

project(processor_board)

# Set the project name
set(CMAKE_PROJECT_NAME processor_board)
set(BUILD_TARGET "firmware" CACHE STRING "Select the build target: firmware or test")
set_property(CACHE BUILD_TARGET PROPERTY STRINGS firmware test)

# Setup C compiler settings for project-wide consistency
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include the appropriate subdirectory based on BUILD_TARGET.
if(BUILD_TARGET STREQUAL "test")
    message(STATUS "Configuring unit test build")
    # need to enable_testing before adding the tests subdir
    enable_testing()
    add_subdirectory(tests)
else()
    message(FATAL_ERROR "Unknown build target: ${BUILD_TARGET}. Please choose 'firmware' or 'test'.")
endif()
