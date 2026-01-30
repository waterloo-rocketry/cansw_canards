# ------------------ bookkeeping settings stuff ------------------
cmake_minimum_required(VERSION 3.22)

# needed for the assembly startup file
enable_language(C ASM)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set debug/release optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-O0 -g3)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3)
endif()

# Export compile commands for caching the build for speed
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# this is required to make the debug info in the elf file point to the correct source files
# instead of the absolute path in the build directory. this makes gdb search relative to the
# root of the project instead of the build directory
# add_compile_options(-fdebug-prefix-map=${CMAKE_SOURCE_DIR}=../..)
add_compile_options(-fdebug-prefix-map=${CMAKE_SOURCE_DIR}=.)

# Custom target to run clang-format. The cmakepreset adds this to the main build
# NOTE: Don't make this a `add_dependency` so that the github pipeline doesn't format
# automatically (that causes it to always pass the pipeline :skullemotijytjnlfkjd:)
add_custom_target(format
    COMMAND echo "running clang-format"
    COMMAND clang-format -i --style=file:${CMAKE_SOURCE_DIR}/src/third_party/rocketlib/.clang-format
    ${CMAKE_SOURCE_DIR}/src/application/*/*.c ${CMAKE_SOURCE_DIR}/src/application/*/*.h
    ${CMAKE_SOURCE_DIR}/src/drivers/*/*.c ${CMAKE_SOURCE_DIR}/src/drivers/*/*.h
    ${CMAKE_SOURCE_DIR}/src/common/*/*.c ${CMAKE_SOURCE_DIR}/src/common/*/*.h
)

# ------------------ third-party sources ------------------
# an earlier version of this cmake setup had each thirdparty module as a STATIC library,
# including the stm32cubemx library. But that causes super sus hidden linking issues (adc
# breaking, who knows what else). So now, compile canlib and printf libraries directly into
# the final proc_binary target, but keep xsens and cmsisdsp as STATIC libraries since they
# do not mingle with stm32cubemx or our source code. (canlibb and printf both use stm32cubemx)

# ------------------ stm32 hal ------------------
# Include stm32cubemx auto generated cmake stuff
include("${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake")
add_subdirectory("${CMAKE_SOURCE_DIR}/cmake/stm32cubemx" "cmake/stm32cubemx")

# ------------------ canlib, xsens, cmsisdsp ------------------
# dont use glob here because libs have lots of unused files
set(THIRD_PARTY_SOURCES
    third_party/canlib/message/msg_actuator.c
    third_party/canlib/message/msg_common.c
    third_party/canlib/message/msg_general.c
    third_party/canlib/message/msg_gps.c
    third_party/canlib/message/msg_recovery.c
    third_party/canlib/message/msg_sensor.c
    third_party/canlib/message/msg_state_est.c
    third_party/canlib/stm32h7/stm32h7_can.c
    third_party/canlib/util/can_rcv_buffer.c
    third_party/canlib/util/can_tx_buffer.c
    third_party/canlib/util/safe_ring_buffer.c
    third_party/canlib/util/timing_util.c
    # tinyprintf sources
    third_party/printf/printf.c
    # xsens-mti sources
    third_party/xsens-mti/src/xsens_mdata2.c
    third_party/xsens-mti/src/xsens_mti.c
    third_party/xsens-mti/src/xsens_utility.c
    # cmsis-dsp sources
    third_party/CMSIS-DSP/Source/BasicMathFunctions/BasicMathFunctions.c 
    third_party/CMSIS-DSP/Source/InterpolationFunctions/InterpolationFunctions.c 
    third_party/CMSIS-DSP/Source/MatrixFunctions/MatrixFunctions.c 
    third_party/CMSIS-DSP/Source/QuaternionMathFunctions/QuaternionMathFunctions.c 
    third_party/CMSIS-DSP/Source/FastMathFunctions/FastMathFunctions.c
)

# ------------------ processor binary target ------------------

# Create the proc firmware binary target
add_executable(proc_binary)

# get processor source files
file(GLOB_RECURSE PROC_SOURCES
    "application/*.c"
    "drivers/*.c"
    "common/*.c"
)

# add thr source files
target_sources(proc_binary PRIVATE
    ${PROC_SOURCES}
    ${THIRD_PARTY_SOURCES}
)

# processor target include paths (made all paths relative to project root)
set(COMMON_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    third_party
    third_party/canlib
    third_party/printf
    third_party/xsens-mti/src
    third_party/CMSIS-DSP/Include
    third_party/CMSIS-DSP/PrivateInclude
)

# processor target definitions
set(COMPILE_DEFINITIONS
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H733xx
    STM32_THREAD_SAFE_STRATEGY=4
    $<$<CONFIG:Debug>:DEBUG>
    BOARD_INST_UNIQUE_ID=BOARD_INST_ID_ROCKET
    BOARD_TYPE_UNIQUE_ID=BOARD_TYPE_ID_PROCESSOR
)

# set the include paths and definitions defined above
target_include_directories(proc_binary PRIVATE ${COMMON_INCLUDES})
target_compile_definitions(proc_binary PRIVATE ${COMPILE_DEFINITIONS})

# enable gcc "all warnings are errors" for all processor code
target_compile_options(proc_binary PRIVATE -Wall -Werror)

# however, disable those gcc warnings for the various third-party sources
foreach(src ${STM32CUBEMX_SOURCES} ${THIRD_PARTY_SOURCES})
    set_source_files_properties(${src} PROPERTIES COMPILE_FLAGS -w)
endforeach()

# Add stm32 cube into the proc binary target
target_link_libraries(proc_binary stm32cubemx)
