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
# =====================================================
# G1 Keyboard IK Control
# =====================================================

add_executable(
    g1_keyboard_ik
    src/keyboard_ik.cpp
)

target_link_libraries(
    g1_keyboard_ik
    PRIVATE
    unitree_sdk2
    pinocchio::pinocchio
)
# =====================================================
# G1 ArmSDK Real Robot Hold Test
# =====================================================

add_executable(
    g1_arm_hold_test
    src/arm_hold_test.cpp
)

target_link_libraries(
    g1_arm_hold_test
    PRIVATE
    unitree_sdk2
)

# =====================================================
# G1 Passive Arm Diagnostic
# =====================================================

add_executable(
    g1_arm_diagnostic
    src/arm_diagnostic.cpp
)

target_link_libraries(
    g1_arm_diagnostic
    PRIVATE
    unitree_sdk2
)
# =====================================================
# G1 ArmSDK Handover Dry Run
# =====================================================

add_executable(
    g1_arm_handover_dryrun
    src/arm_handover_dryrun.cpp
)

target_link_libraries(
    g1_arm_handover_dryrun
    PRIVATE
    unitree_sdk2
)
# ==========================================
# G1 Robot Mode Reader (READ ONLY)
# ==========================================

add_executable(
    g1_mode_reader
    src/g1_mode_reader.cpp
)

target_link_libraries(
    g1_mode_reader
    PRIVATE
    unitree_sdk2
)

# ==========================================
# G1 Motion Service Status Reader (QUERY ONLY)
# ==========================================

add_executable(
    g1_motion_status
    src/g1_motion_status.cpp
)

target_link_libraries(
    g1_motion_status
    PRIVATE
    unitree_sdk2
)
# ==========================================
# G1 ArmSDK Command Dry Run (READ ONLY)
# ==========================================

add_executable(
    g1_arm_command_dryrun
    src/g1_arm_command_dryrun.cpp
)

target_link_libraries(
    g1_arm_command_dryrun
    PRIVATE
    unitree_sdk2
)
# ==========================================
# G1 CRC32 Comparison (OFFLINE ONLY)
# ==========================================

add_executable(
    g1_crc_compare
    src/g1_crc_compare.cpp
)

target_link_libraries(
    g1_crc_compare
    PRIVATE
    unitree_sdk2
)


# =====================================================
# G1 Keyboard-Controlled Palm IK (OFFLINE ONLY)
# =====================================================

add_executable(
    g1_keyboard_palm_ik
    src/keyboard_palm_ik.cpp
)

target_link_libraries(
    g1_keyboard_palm_ik
    PRIVATE
    unitree_sdk2
    pinocchio::pinocchio
)

# ==========================================
# G1 Keyboard Palm IK + ArmSDK
# ==========================================

add_executable(
    g1_keyboard_arm_control
    src/g1_keyboard_arm_control.cpp
)

target_link_libraries(
    g1_keyboard_arm_control
    PRIVATE
    unitree_sdk2
    pinocchio::pinocchio
)
