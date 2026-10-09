#include "pinocchio/fwd.hpp"

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Core>

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using namespace unitree::robot;

using LowState = unitree_hg::msg::dds_::LowState_;


// =====================================================
// 1. Robot state
// =====================================================

std::mutex state_mutex;

LowState latest_state;

std::atomic<bool> received_state{false};

std::chrono::steady_clock::time_point last_receive_time;


// =====================================================
// 2. DDS callback
// =====================================================

void StateCallback(const void* message)
{
    if (message == nullptr) {
        return;
    }

    const auto* state =
        static_cast<const LowState*>(message);

    {
        std::lock_guard<std::mutex> lock(state_mutex);

        latest_state = *state;

        last_receive_time =
            std::chrono::steady_clock::now();
    }

    received_state.store(true);
}


// =====================================================
// 3. Joint mapping
// =====================================================

// The motor indices follow the Unitree G1 29-DOF
// layout used by the official 7-DOF arm example.
//
// For torso-relative FK, the arm joints are required.
// Waist joints are also mapped for completeness.

struct JointMapping
{
    std::string name;

    int motor_index;
};

const std::vector<JointMapping> joint_mapping = {

    // Waist
    {"waist_yaw_joint",   12},
    {"waist_roll_joint",  13},
    {"waist_pitch_joint", 14},

    // Left arm
    {"left_shoulder_pitch_joint", 15},
    {"left_shoulder_roll_joint",  16},
    {"left_shoulder_yaw_joint",   17},
    {"left_elbow_joint",          18},
    {"left_wrist_roll_joint",     19},
    {"left_wrist_pitch_joint",    20},
    {"left_wrist_yaw_joint",      21},

    // Right arm
    {"right_shoulder_pitch_joint", 22},
    {"right_shoulder_roll_joint",  23},
    {"right_shoulder_yaw_joint",   24},
    {"right_elbow_joint",          25},
    {"right_wrist_roll_joint",     26},
    {"right_wrist_pitch_joint",    27},
    {"right_wrist_yaw_joint",      28}
};


// =====================================================
// 4. Convert motor states to Pinocchio configuration
// =====================================================

void UpdateConfiguration(
    const pinocchio::Model& model,
    const LowState& state,
    Eigen::VectorXd& q)
{
    for (const auto& item : joint_mapping) {

        if (!model.existJointName(item.name)) {
            throw std::runtime_error(
                "URDF joint not found: " + item.name
            );
        }

        pinocchio::JointIndex joint_id =
            model.getJointId(item.name);

        const auto& joint = model.joints[joint_id];

        if (joint.nq() != 1) {
            throw std::runtime_error(
                "Expected a 1-DOF joint: " + item.name
            );
        }

        int q_index = joint.idx_q();

        // Read the actual joint angle
        double angle =
            state.motor_state().at(item.motor_index).q();

        q[q_index] = angle;
    }
}


// =====================================================
// 5. Print XYZ
// =====================================================

void PrintPosition(
    const std::string& name,
    const Eigen::Vector3d& position)
{
    std::cout << "\n" << name << "\n";

    std::cout << std::fixed
              << std::setprecision(4);

    std::cout << "X = " << position.x()
              << " m\n";

    std::cout << "Y = " << position.y()
              << " m\n";

    std::cout << "Z = " << position.z()
              << " m\n";
}


// =====================================================
// 6. Main
// =====================================================

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 5) {

        std::cerr
            << "Usage: "
            << argv[0]
            << " <network_interface> <urdf_path>"
            << " [left_frame] [right_frame]\n";

        return 1;
    }

    std::string network_interface = argv[1];

    std::string urdf_path = argv[2];

    std::string left_frame =
        argc >= 4 ? argv[3] : "left_wrist_yaw_link";

    std::string right_frame =
        argc >= 5 ? argv[4] : "right_wrist_yaw_link";

    try {

        // ---------------------------------------------
        // Load URDF model
        // ---------------------------------------------

        pinocchio::Model model;

        pinocchio::urdf::buildModel(
            urdf_path,
            model
        );

        pinocchio::Data data(model);

        std::cout << "URDF loaded successfully.\n";

        std::cout << "Model nq: "
                  << model.nq << "\n";

        std::cout << "Model nv: "
                  << model.nv << "\n";

        // ---------------------------------------------
        // Check frames
        // ---------------------------------------------

        if (!model.existFrame("torso_link")) {
            throw std::runtime_error(
                "Frame not found: torso_link"
            );
        }

        if (!model.existFrame(left_frame)) {
            throw std::runtime_error(
                "Left frame not found: " + left_frame
            );
        }

        if (!model.existFrame(right_frame)) {
            throw std::runtime_error(
                "Right frame not found: " + right_frame
            );
        }

        // Validate all required joints before reading.
        for (const auto& item : joint_mapping) {

            if (!model.existJointName(item.name)) {
                throw std::runtime_error(
                    "Joint not found: " + item.name
                );
            }

            auto joint_id = model.getJointId(item.name);

            if (model.joints[joint_id].nq() != 1) {
                throw std::runtime_error(
                    "Unsupported joint: " + item.name
                );
            }
        }

        auto torso_id =
            model.getFrameId("torso_link");

        auto left_id =
            model.getFrameId(left_frame);

        auto right_id =
            model.getFrameId(right_frame);

        // ---------------------------------------------
        // Initialize DDS
        // ---------------------------------------------

        ChannelFactory::Instance()->Init(
            0,
            network_interface
        );

        auto subscriber =
            std::make_shared<ChannelSubscriber<LowState>>(
                "rt/lowstate"
            );

        subscriber->InitChannel(
            StateCallback,
            1
        );

        std::cout
            << "\nWaiting for G1 state...\n";

        // Wait for the first state message
        for (int i = 0; i < 100; ++i) {

            if (received_state.load()) {
                break;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(100)
            );
        }

        if (!received_state.load()) {

            std::cerr
                << "ERROR: No G1 state received.\n";

            return 1;
        }

        std::cout << "G1 state received.\n";

        // ---------------------------------------------
        // Compute FK
        // ---------------------------------------------

        for (int count = 0; count < 10; ++count) {

            LowState snapshot;

            std::chrono::steady_clock::time_point
                receive_time;

            {
                std::lock_guard<std::mutex> lock(
                    state_mutex
                );

                snapshot = latest_state;

                receive_time = last_receive_time;
            }

            // Avoid displaying stale robot states
            auto state_age =
                std::chrono::duration_cast<
                    std::chrono::milliseconds
                >(
                    std::chrono::steady_clock::now()
                    - receive_time
                ).count();

            if (state_age > 1000) {

                std::cerr
                    << "ERROR: Robot state is stale.\n";

                return 1;
            }

            // Initialize robot configuration
            Eigen::VectorXd q =
                pinocchio::neutral(model);

            // Fill arm and waist joint angles
            UpdateConfiguration(
                model,
                snapshot,
                q
            );

            // Forward kinematics
            pinocchio::forwardKinematics(
                model,
                data,
                q
            );

            pinocchio::updateFramePlacements(
                model,
                data
            );

            // Frame transformations
            const auto& oMtorso =
                data.oMf[torso_id];

            const auto& oMleft =
                data.oMf[left_id];

            const auto& oMright =
                data.oMf[right_id];

            // Convert to torso coordinates
            Eigen::Vector3d left_xyz =
                (oMtorso.inverse() * oMleft)
                    .translation();

            Eigen::Vector3d right_xyz =
                (oMtorso.inverse() * oMright)
                    .translation();

            std::cout
                << "\n========== G1 HAND FK ==========\n";

            PrintPosition(
                "LEFT HAND",
                left_xyz
            );

            PrintPosition(
                "RIGHT HAND",
                right_xyz
            );

            std::cout
                << "================================\n";

            std::this_thread::sleep_for(
                std::chrono::seconds(1)
            );
        }

        std::cout << "\nFK test finished.\n";

    } catch (const std::exception& e) {

        std::cerr
            << "ERROR: " << e.what() << "\n";

        return 1;
    }

    return 0;
}
