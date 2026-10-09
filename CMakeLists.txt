cmake_minimum_required(VERSION 3.10)

project(G1_Arm_IK_Control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Path to Unitree SDK2
set(UNITREE_SDK_ROOT
    "$ENV{HOME}/unitree_sdk2"
    CACHE PATH "Unitree SDK2 source directory"
)

if(NOT EXISTS "${UNITREE_SDK_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR
        "Unitree SDK2 not found: ${UNITREE_SDK_ROOT}"
    )
endif()

# Do not build SDK examples
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# Import Unitree SDK2
add_subdirectory(
    "${UNITREE_SDK_ROOT}"
    "${CMAKE_BINARY_DIR}/unitree_sdk_build"
    EXCLUDE_FROM_ALL
)

# Create executable
add_executable(
    g1_arm_state
    src/main.cpp
)

# Link SDK
target_link_libraries(
    g1_arm_state
    PRIVATE
    unitree_sdk2
)
# =====================================================
# Pinocchio
# =====================================================

find_package(pinocchio REQUIRED)

# =====================================================
# G1 FK Reader
# =====================================================

add_executable(
    g1_arm_fk
    src/fk_reader.cpp
)

target_link_libraries(
    g1_arm_fk
    PRIVATE
    unitree_sdk2
    pinocchio::pinocchio
)
# =====================================================
# G1 Dual Arm Offline IK Test
# =====================================================

add_executable(
    g1_arm_ik_test
    src/ik_test.cpp
)

target_link_libraries(
    g1_arm_ik_test
    PRIVATE
    unitree_sdk2
    pinocchio::pinocchio
)
