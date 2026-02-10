cmake_minimum_required(VERSION 3.22)

# Set coverage data directory
set(COVERAGE_DATA_DIR "${CMAKE_BINARY_DIR}/coverage_data")
file(MAKE_DIRECTORY ${COVERAGE_DATA_DIR})

# Add coverage flags - safely to avoid libgcov errors
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0 -Wno-unused-parameter")

# Add linker flags for gcov
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed -Wl,--gc-sections")

# Set coverage flags per-file to avoid conflicts - add atomic update to prevent timestamp errors
set(COVERAGE_COMPILE_FLAGS "--coverage -fprofile-arcs -ftest-coverage -fprofile-correction -fprofile-update=atomic")

# Environment variables to prevent libgcov errors - set globally for all processes
set(ENV{GCOV_PREFIX} "${COVERAGE_DATA_DIR}")
set(ENV{GCOV_PREFIX_STRIP} "4") # Strip first 4 directory levels to get relative paths

# Enable faster linking with the gold linker if available
# This is commonly supported in CI environments but we'll check first
if(UNIX AND NOT APPLE)
  include(CheckLinkerFlag)
  check_linker_flag(CXX "-fuse-ld=gold" GOLD_LINKER_SUPPORTED)
  if(GOLD_LINKER_SUPPORTED)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")
  endif()
endif()

add_compile_definitions(GTEST)


# Define paths
set(MOCKS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
set(FFF_DIR ${MOCKS_DIR}/fff)
set(UNIT_TESTS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/unit)
# Full path to the file
set(PROC_SRC_DIR "${CMAKE_SOURCE_DIR}/src")

# add ctest
include(CTest)
# Add GoogleTest
add_subdirectory(external/googletest)
enable_testing()
include(GoogleTest)
add_compile_definitions(GTEST)

# helper to create a test executable
# usage: `add_test_group(${test_name} ${src_files})`
# src_files must be a list of the test src file(s) and the proc src file(s) being tested
macro(add_test_group TEST_NAME)
    add_executable(${TEST_NAME} ${ARGN})  # Expands all additional arguments as a list
    # all tests get the same common include paths and common libraries
    target_include_directories(${TEST_NAME} PRIVATE ${TEST_COMMON_INCS})
    target_link_libraries(${TEST_NAME} PRIVATE ${TEST_COMMON_LIBS})
    
    # Create a unique coverage directory for this test
    file(MAKE_DIRECTORY "${COVERAGE_DATA_DIR}/${TEST_NAME}")
    
    # Add coverage flags only to source files being tested, not test files or external libs
    foreach(src_file ${ARGN})
        if(src_file MATCHES ".*\\.c$" AND NOT src_file MATCHES ".*/mocks/.*" AND NOT src_file MATCHES ".*/external/.*")
            # Extract the filename for more unique coverage data path
            get_filename_component(src_file_name ${src_file} NAME)
            set_source_files_properties(${src_file} PROPERTIES 
                COMPILE_FLAGS "${COVERAGE_COMPILE_FLAGS}"
                # Set per-file coverage environment variables using compiler definitions
                COMPILE_DEFINITIONS "GCOV_PREFIX=\"${COVERAGE_DATA_DIR}/${TEST_NAME}/${src_file_name}\";GCOV_PREFIX_STRIP=4")
        endif()
    endforeach()
    
    # Link with gcov for coverage
    target_link_libraries(${TEST_NAME} PRIVATE gcov)
    
    # Each test should store its coverage data in a unique directory
    set_target_properties(${TEST_NAME} PROPERTIES 
        ENVIRONMENT "GCOV_PREFIX=${COVERAGE_DATA_DIR}/${TEST_NAME};GCOV_PREFIX_STRIP=4")
    

    # # order of operations: clean_coverage, build tests (incl gcda/gcno files), then run test executables
    add_dependencies(${TEST_NAME} clean_coverage)
    add_dependencies(coverage ${TEST_NAME})
    
    # canlib needs this so give it to everyone ig
    target_compile_definitions(${TEST_NAME} PRIVATE
        BOARD_INST_UNIQUE_ID=0x01
        BOARD_TYPE_UNIQUE_ID=0X06
        STM32H733xx
    )
    # register the tests with ctest (via this gtest helper)
    gtest_discover_tests(${TEST_NAME})
endmacro()

# Make tests runnable without coverage for now
add_custom_target(runtests
    # Clean coverage data before running tests
    COMMAND ${CMAKE_COMMAND} -E make_directory ${COVERAGE_DATA_DIR}
    COMMAND find ${CMAKE_BINARY_DIR} -name "*.gcda" -delete
    # Run all the test groups
    COMMAND ctest --output-on-failure
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running tests..."
)
add_dependencies(runtests clean_coverage)


# Make tests automatically generate coverage report after running
add_custom_target(coverage ALL
    # first, run all the test groups
    COMMAND ctest --output-on-failure
    # then, generate the coverage report
    COMMAND lcov -b ${CMAKE_BINARY_DIR} -capture --directory ${CMAKE_BINARY_DIR} --output-file src_coverage.info --rc branch_coverage=1
    # only include the 'unit' subdirs in the coverage report cuz those are the proc source files we're testing
    # (ie, idc how much of canlib we tested cuz thats not our code)
    COMMAND lcov -b ${CMAKE_BINARY_DIR} --extract src_coverage.info ${PROC_SRC_DIR}/common/*/* ${PROC_SRC_DIR}/drivers/*/* ${PROC_SRC_DIR}/application/*/* --output-file src_coverage_filtered.info --rc branch_coverage=1 --ignore-errors unused,unused
    # generate the html coverage report
    # TODO: make this display for github pipeline in terminal? idk
    COMMAND genhtml src_coverage_filtered.info --output-directory coverage_report --branch-coverage 

    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running tests and generating coverage report (only including 'unit' subdirectories)..."
)

# Add coverage target
# add_custom_target(coverage
#     # No need to clean here since we depend on clean_coverage
    
#     # Run the tests to generate coverage data
#     COMMAND ctest --output-on-failure
    
#     # Generate coverage report
#     COMMAND lcov -b ${CMAKE_BINARY_DIR} -d ${CMAKE_BINARY_DIR} -c -o ${CMAKE_BINARY_DIR}/coverage.info --no-external
#     COMMAND lcov --remove ${CMAKE_BINARY_DIR}/coverage.info '/usr/*' 'tests/*' '*/external/*' -o ${CMAKE_BINARY_DIR}/coverage_filtered.info
#     COMMAND genhtml ${CMAKE_BINARY_DIR}/coverage_filtered.info --output-directory ${CMAKE_BINARY_DIR}/coverage_report
    
#     WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
#     COMMENT "Generating coverage report..."
# )
# add_dependencies(coverage clean_coverage)

# Add a custom target to clean coverage data
add_custom_target(clean_coverage
    # !!! not working because clean_coverage deletes gcda files for ALL tests, which requires ALL tests to be rebuilt
    # but thanks to caching thats not always true... !!!

    # COMMAND ${CMAKE_COMMAND} -E remove_directory ${COVERAGE_DATA_DIR}
    # COMMAND ${CMAKE_COMMAND} -E make_directory ${COVERAGE_DATA_DIR}
    # COMMAND find ${CMAKE_BINARY_DIR} -name "*.gcda" -delete
    # COMMAND find ${CMAKE_BINARY_DIR} -name "*.gcno" -delete
    # WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    # COMMENT "Cleaning coverage data..."
)

# add cmsis dsp library. Do this manually instead of using the cmsis-dsp
# CMakeLists.txt to avoid building unused cmsisdsp modules which would be slow and fat (derogatory)
set(CMSISDSP_DIR ${PROC_SRC_DIR}/third_party/CMSIS-DSP)
add_library(CMSISDSP STATIC
    ${CMSISDSP_DIR}/Source/BasicMathFunctions/BasicMathFunctions.c 
    ${CMSISDSP_DIR}/Source/InterpolationFunctions/InterpolationFunctions.c 
    ${CMSISDSP_DIR}/Source/MatrixFunctions/MatrixFunctions.c 
    ${CMSISDSP_DIR}/Source/QuaternionMathFunctions/QuaternionMathFunctions.c 
    ${CMSISDSP_DIR}/Source/FastMathFunctions/FastMathFunctions.c 
)
# the include paths needed by cmsis-dsp itself
target_include_directories(CMSISDSP PUBLIC "${CMSISDSP_DIR}/Include")
target_include_directories(CMSISDSP PUBLIC "${CMSISDSP_DIR}/PrivateInclude")
# cmsis-dsp also requires the core cmsis files. use the cmsis folder provided by cubemx
target_include_directories(CMSISDSP PUBLIC "${PROC_SRC_DIR}/third_party/cubemx_autogen/Drivers/CMSIS/Include")


# define the common include paths that all tests may use
set(TEST_COMMON_INCS
    ${MOCKS_DIR}
    ${FFF_DIR}
    ${PROC_SRC_DIR}
    ${PROC_SRC_DIR}/third_party
    ${PROC_SRC_DIR}/third_party/canlib
)

# Add FFF as a header-only library
add_library(fff INTERFACE)
target_include_directories(fff INTERFACE ${FFF_DIR})

# Add mocks as a static lib, without coverage
file(GLOB MOCK_SOURCES "${MOCKS_DIR}/*.c")
add_library(mocks STATIC ${MOCK_SOURCES})
target_include_directories(mocks PUBLIC ${TEST_COMMON_INCS})

# define the common libs that all tests may use
set(TEST_COMMON_LIBS
    fff           # Fake Function Framework
    mocks         # our mocks
    gtest_main    # GoogleTest
    CMSISDSP
)

###############################################################################
# Add test groups one by one using the helper macro `add_test_group(${test_name} ${src_files})`
# src_files is the list of the test src file(s) and the proc src file(s) being tested
# The list of src_files must be in quotations since it's technically 1 argument
###############################################################################
add_test_group(test_example
    # test file(s)
    ${UNIT_TESTS_DIR}/example_test.cpp
    # src file(s)
    # (example test doesn't actually test any proc source files)
)

add_test_group(test_gpio
    # test file(s)
    ${UNIT_TESTS_DIR}/gpio_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/gpio/gpio.c
)

add_test_group(test_uart
    # test file(s)
    ${UNIT_TESTS_DIR}/uart_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/uart/uart.c
)

add_test_group(test_estimator
    # test file(s)
    ${UNIT_TESTS_DIR}/estimator_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/estimator/estimator.c
    ${PROC_SRC_DIR}/application/estimator/estimator_module.c
    ${PROC_SRC_DIR}/application/estimator/ekf.c
    ${PROC_SRC_DIR}/application/estimator/pad_filter.c
    ${PROC_SRC_DIR}/application/estimator/model/model_dynamics.c
    ${PROC_SRC_DIR}/application/estimator/model/model_aerodynamics.c
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
    ${PROC_SRC_DIR}/application/estimator/model/model_imu.c
    ${PROC_SRC_DIR}/application/estimator/model/quaternion.c
    ${PROC_SRC_DIR}/application/estimator/model/model_airdata.c
    ${PROC_SRC_DIR}/application/estimator/model/jacobians.c
    ${PROC_SRC_DIR}/application/estimator/model/model_acceleration.c
    ${PROC_SRC_DIR}/application/estimator/projector.c
    ${PROC_SRC_DIR}/application/estimator/model/model_encoder.c
)

add_test_group(test_estimator_module
    # test file(s)
    ${UNIT_TESTS_DIR}/estimator_module_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/estimator/estimator_module.c
    ${PROC_SRC_DIR}/application/estimator/ekf.c
    ${PROC_SRC_DIR}/application/estimator/pad_filter.c
    ${PROC_SRC_DIR}/application/estimator/model/model_dynamics.c
    ${PROC_SRC_DIR}/application/estimator/model/model_aerodynamics.c
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
    ${PROC_SRC_DIR}/application/estimator/model/model_imu.c
    ${PROC_SRC_DIR}/application/estimator/model/quaternion.c
    ${PROC_SRC_DIR}/application/estimator/model/model_airdata.c
    ${PROC_SRC_DIR}/application/estimator/model/jacobians.c
    ${PROC_SRC_DIR}/application/estimator/model/model_acceleration.c
    ${PROC_SRC_DIR}/application/estimator/projector.c
    ${PROC_SRC_DIR}/application/estimator/model/model_encoder.c
)

add_test_group(test_i2c
    # test file(s)
    ${UNIT_TESTS_DIR}/i2c_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/i2c/i2c.c
)

add_test_group(test_timer
    # test file(s)
    ${UNIT_TESTS_DIR}/timer_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/timer/timer.c
)

add_test_group(test_imu_handler
    # test file(s)
    ${UNIT_TESTS_DIR}/imu_handler_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/imu_handler/imu_handler.c
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
)

add_test_group(test_flight_phase
    # test files
    ${UNIT_TESTS_DIR}/flight_phase_test.cpp
    # src files
    ${PROC_SRC_DIR}/application/flight_phase/flight_phase.c
)

add_test_group(test_altimu
    # test file(s)
    ${UNIT_TESTS_DIR}/altimu_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/drivers/altimu-10/altimu-10.c
)

add_test_group(test_model_airdata
    # test file(s)
    ${UNIT_TESTS_DIR}/model_airdata_test.cpp
    # src file(s)
        ${PROC_SRC_DIR}/application/estimator/model/model_airdata.c
)

add_test_group(test_pad_filter
    # test file(s)
    ${UNIT_TESTS_DIR}/pad_filter_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/estimator/pad_filter.c
    ${PROC_SRC_DIR}/application/estimator/model/quaternion.c
    ${PROC_SRC_DIR}/application/estimator/model/model_airdata.c
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
)

add_test_group(test_common_math
    # test file(s)
    ${UNIT_TESTS_DIR}/common_math_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/estimator/model/quaternion.c
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
)

add_test_group(test_quaternion
    # test file(s)
    ${UNIT_TESTS_DIR}/quaternion_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/estimator/model/quaternion.c
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
)

add_test_group(test_model_imu
# test file(s)
${UNIT_TESTS_DIR}/model_imu_test.cpp
# src file(s)
${PROC_SRC_DIR}/application/estimator/model/model_imu.c
${PROC_SRC_DIR}/common/math/math-algebra3d.c
${PROC_SRC_DIR}/application/estimator/model/quaternion.c
${PROC_SRC_DIR}/application/estimator/model/model_airdata.c
${PROC_SRC_DIR}/application/estimator/model/jacobians.c
)
add_test_group(test_health_check
    # test file(s)
    ${UNIT_TESTS_DIR}/health_checks_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/health_checks/health_checks.c
)

add_test_group(test_projector
    # test file(s)
    ${UNIT_TESTS_DIR}/projector_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/estimator/projector.c
    ${PROC_SRC_DIR}/application/estimator/model/quaternion.c
    ${PROC_SRC_DIR}/application/estimator/model/model_airdata.c
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
)

add_test_group(test_model_dynamics
    # test file(s)
    ${UNIT_TESTS_DIR}/model_dynamic_test.cpp
    # src file(s)
    # (example test doesn't actually test any proc source files)
    ${PROC_SRC_DIR}/application/estimator/model/quaternion.c
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
    ${PROC_SRC_DIR}/application/estimator/model/model_airdata.c
    ${PROC_SRC_DIR}/application/estimator/model/model_dynamics.c
    ${PROC_SRC_DIR}/application/estimator/model/model_aerodynamics.c
    ${PROC_SRC_DIR}/application/estimator/model/jacobians.c
)

add_test_group(test_controller_module
    # test file(s)
    ${UNIT_TESTS_DIR}/controller_module_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/controller/controller_module.c
    ${PROC_SRC_DIR}/application/controller/gain_table.c
)

add_test_group(test_controller
    # test file(s)
    ${UNIT_TESTS_DIR}/controller_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/controller/controller.c
    ${PROC_SRC_DIR}/application/controller/controller_module.c
    ${PROC_SRC_DIR}/application/controller/gain_table.c
)

add_test_group(test_model_encoder
    # test file(s)
    ${UNIT_TESTS_DIR}/model_encoder_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/estimator/model/model_encoder.c
)

add_test_group(test_estimator_ekf
    # test file(s)
    ${UNIT_TESTS_DIR}/ekf_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/application/estimator/ekf.c
    ${PROC_SRC_DIR}/application/estimator/model/model_dynamics.c
    ${PROC_SRC_DIR}/application/estimator/model/model_aerodynamics.c
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
    ${PROC_SRC_DIR}/application/estimator/model/model_imu.c
    ${PROC_SRC_DIR}/application/estimator/model/quaternion.c
    ${PROC_SRC_DIR}/application/estimator/model/model_airdata.c
    ${PROC_SRC_DIR}/application/estimator/model/jacobians.c
    ${PROC_SRC_DIR}/application/estimator/model/model_acceleration.c
    ${PROC_SRC_DIR}/application/estimator/model/model_encoder.c
)

add_test_group(test_model_acceleration
    # test file(s)
    ${UNIT_TESTS_DIR}/model_acceleration_test.cpp
    # src file(s)
    ${PROC_SRC_DIR}/common/math/math-algebra3d.c
    ${PROC_SRC_DIR}/application/estimator/model/model_acceleration.c
)
