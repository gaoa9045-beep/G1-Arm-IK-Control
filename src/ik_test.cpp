#include "pinocchio/fwd.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

using namespace unitree::robot;

using LowState = unitree_hg::msg::dds_::LowState_;

std::mutex state_mutex;
LowState latest_state;
std::atomic<bool> received_state{false};

std::chrono::steady_clock::time_point last_receive_time;

// -----------------------------------------------------
// DDS state callback
// -----------------------------------------------------

void StateCallback(const void* message)
{
    if (!message) return;

    const auto* state =
        static_cast<const LowState*>(message);

    std::lock_guard<std::mutex> lock(state_mutex);

    latest_state = *state;

    last_receive_time =
        std::chrono::steady_clock::now();

    received_state.store(true);
}

// -----------------------------------------------------
// Joint names
// -----------------------------------------------------

const std::vector<std::string> waist_names = {
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint"
};

const std::vector<std::string> left_names = {
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint"
};

const std::vector<std::string> right_names = {
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint"
};

// -----------------------------------------------------
// Get configuration indices from URDF names
// -----------------------------------------------------

std::vector<int> GetJointIndices(
    const pinocchio::Model& model,
    const std::vector<std::string>& names)
{
    std::vector<int> indices;

    for (const auto& name : names) {

        if (!model.existJointName(name)) {
            throw std::runtime_error(
                "Joint not found: " + name
            );
        }

        auto id = model.getJointId(name);

        if (model.joints[id].nq() != 1 ||
            model.joints[id].nv() != 1) {

            throw std::runtime_error(
                "Expected 1-DOF joint: " + name
            );
        }

        indices.push_back(model.joints[id].idx_q());
    }

    return indices;
}

// -----------------------------------------------------
// Load actual arm joint states
// -----------------------------------------------------

void FillJoints(
    Eigen::VectorXd& q,
    const std::vector<int>& indices,
    const LowState& state,
    int motor_start)
{
    for (size_t i = 0; i < indices.size(); ++i) {
        q[indices[i]] =
            state.motor_state().at(motor_start + i).q();
    }
}

// -----------------------------------------------------
// FK position in model world frame
// -----------------------------------------------------

Eigen::Vector3d ComputePosition(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q,
    pinocchio::FrameIndex frame_id)
{
    pinocchio::forwardKinematics(model, data, q);

    pinocchio::updateFramePlacements(model, data);

    return data.oMf[frame_id].translation();
}

// -----------------------------------------------------
// Damped least squares IK
// Only the selected arm's 7 joints are modified.
// -----------------------------------------------------

bool SolveIK(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    Eigen::VectorXd& q,
    pinocchio::FrameIndex frame_id,
    const std::vector<int>& arm_indices,
    const Eigen::Vector3d& target_world)
{
    constexpr int max_iterations = 150;

    constexpr double tolerance = 0.0005;

    constexpr double damping = 0.03;

    constexpr double max_joint_step = 0.02;

    constexpr double max_total_change = 0.20;

    const Eigen::VectorXd q_start = q;

    for (int iter = 0; iter < max_iterations; ++iter) {

        Eigen::Vector3d current =
            ComputePosition(
                model, data, q, frame_id
            );

        Eigen::Vector3d error =
            target_world - current;

        if (!error.allFinite()) {
            std::cerr << "Non-finite IK error.\n";
            return false;
        }

        double error_norm = error.norm();

        if (error_norm < tolerance) {

            std::cout
                << "IK converged in "
                << iter
                << " iterations.\n";

            return true;
        }

        // Full 6 x nv Jacobian
        Eigen::MatrixXd Jfull =
            Eigen::MatrixXd::Zero(6, model.nv);

        pinocchio::computeFrameJacobian(
            model,
            data,
            q,
            frame_id,
            pinocchio::LOCAL_WORLD_ALIGNED,
            Jfull
        );

        // Extract position Jacobian of 7 arm joints
        Eigen::Matrix<double, 3, 7> J;

        for (int j = 0; j < 7; ++j) {
            J.col(j) =
                Jfull.block<3, 1>(
                    0, arm_indices[j]
                );
        }

        // Damped least squares
        Eigen::Matrix3d A =
            J * J.transpose();

        A.diagonal().array() +=
            damping * damping;

        Eigen::Vector3d y =
            A.ldlt().solve(error);

        Eigen::Matrix<double, 7, 1> dq =
            J.transpose() * y;

        if (!dq.allFinite()) {
            std::cerr << "Non-finite IK update.\n";
            return false;
        }

        // Limit the largest joint change per iteration
        double max_abs = dq.cwiseAbs().maxCoeff();

        if (max_abs > max_joint_step) {
            dq *= max_joint_step / max_abs;
        }

        // Apply only the arm joints
        for (int j = 0; j < 7; ++j) {

            int idx = arm_indices[j];

            double candidate = q[idx] + dq[j];

            // Bound total deviation from measured posture
            candidate = std::clamp(
                candidate,
                q_start[idx] - max_total_change,
                q_start[idx] + max_total_change
            );

            // URDF joint position limits
            double lower =
                model.lowerPositionLimit[idx];

            double upper =
                model.upperPositionLimit[idx];

            if (std::isfinite(lower) &&
                std::isfinite(upper) &&
                lower < upper) {

                candidate = std::clamp(
                    candidate, lower, upper
                );
            }

            q[idx] = candidate;
        }
    }

    Eigen::Vector3d final_position =
        ComputePosition(model, data, q, frame_id);

    std::cout
        << "IK did not converge. Final error: "
        << (target_world - final_position).norm()
        << " m\n";

    return false;
}

// -----------------------------------------------------
// Test one arm
// -----------------------------------------------------

void TestArm(
    const std::string& name,
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q_initial,
    pinocchio::FrameIndex torso_id,
    pinocchio::FrameIndex frame_id,
    const std::vector<int>& indices)
{
    Eigen::VectorXd q = q_initial;

    // FK of current posture
    ComputePosition(
        model, data, q, frame_id
    );

    const pinocchio::SE3 oMtorso =
        data.oMf[torso_id];

    Eigen::Vector3d current_world =
        data.oMf[frame_id].translation();

    Eigen::Vector3d current_torso =
        oMtorso.actInv(current_world);

    // Move +5 mm along torso X axis
    Eigen::Vector3d target_torso =
        current_torso +
        Eigen::Vector3d(0.005, 0.0, 0.0);

    // Convert target into model world coordinates
    Eigen::Vector3d target_world =
        oMtorso.act(target_torso);

    std::cout
        << "\n========== " << name
        << " IK TEST ==========\n";

    std::cout << std::fixed
              << std::setprecision(6);

    std::cout
        << "Current XYZ (torso): "
        << current_torso.transpose()
        << "\n";

    std::cout
        << "Target  XYZ (torso): "
        << target_torso.transpose()
        << "\n";

    bool success = SolveIK(
        model,
        data,
        q,
        frame_id,
        indices,
        target_world
    );

    Eigen::Vector3d final_world =
        ComputePosition(
            model, data, q, frame_id
        );

    Eigen::Vector3d final_torso =
        oMtorso.actInv(final_world);

    double error =
        (final_torso - target_torso).norm();

    std::cout
        << "Final   XYZ (torso): "
        << final_torso.transpose()
        << "\n";

    std::cout
        << "Position error: "
        << error * 1000.0
        << " mm\n";

    std::cout
        << "Joint angle changes (rad):\n";

    for (int j = 0; j < 7; ++j) {

        int idx = indices[j];

        std::cout
            << "Joint " << j + 1
            << ": " << q_initial[idx]
            << " -> " << q[idx]
            << "\n";
    }

    std::cout
        << "Result: "
        << (success ? "PASS" : "FAIL")
        << "\n";
}

// -----------------------------------------------------
// Main
// -----------------------------------------------------

int main(int argc, char** argv)
{
    if (argc != 3) {

        std::cerr
            << "Usage: " << argv[0]
            << " <network_interface> <urdf_path>\n";

        return 1;
    }

    try {

        pinocchio::Model model;

        pinocchio::urdf::buildModel(
            argv[2], model
        );

        pinocchio::Data data(model);

        // Model frames
        const std::string torso_name =
            "torso_link";

        const std::string left_frame =
            "left_wrist_yaw_link";

        const std::string right_frame =
            "right_wrist_yaw_link";

        for (const auto& name :
             {torso_name, left_frame, right_frame}) {

            if (!model.existFrame(name)) {
                throw std::runtime_error(
                    "Frame not found: " + name
                );
            }
        }

        auto torso_id =
            model.getFrameId(torso_name);

        auto left_id =
            model.getFrameId(left_frame);

        auto right_id =
            model.getFrameId(right_frame);

        auto waist_indices =
            GetJointIndices(model, waist_names);

        auto left_indices =
            GetJointIndices(model, left_names);

        auto right_indices =
            GetJointIndices(model, right_names);

        // DDS initialization
        ChannelFactory::Instance()->Init(
            0, argv[1]
        );

        auto subscriber =
            std::make_shared<ChannelSubscriber<LowState>>(
                "rt/lowstate"
            );

        subscriber->InitChannel(
            StateCallback, 1
        );

        std::cout
            << "Waiting for G1 state...\n";

        for (int i = 0; i < 100; ++i) {

            if (received_state.load()) {
                break;
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(100)
            );
        }

        if (!received_state.load()) {
            throw std::runtime_error(
                "No G1 state received."
            );
        }

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

        auto state_age =
            std::chrono::duration_cast<
                std::chrono::milliseconds
            >(
                std::chrono::steady_clock::now()
                - receive_time
            ).count();

        if (state_age > 1000) {
            throw std::runtime_error(
                "Robot state is stale."
            );
        }

        // Initialize configuration
        Eigen::VectorXd q =
            pinocchio::neutral(model);

        // Fill waist and arm joints
        FillJoints(q, waist_indices, snapshot, 12);
        FillJoints(q, left_indices, snapshot, 15);
        FillJoints(q, right_indices, snapshot, 22);

        // Solve each arm independently
        TestArm(
            "LEFT ARM",
            model, data, q,
            torso_id, left_id, left_indices
        );

        TestArm(
            "RIGHT ARM",
            model, data, q,
            torso_id, right_id, right_indices
        );

        std::cout
            << "\nOffline IK test finished.\n";

        // No motor commands are sent.

    } catch (const std::exception& e) {

        std::cerr
            << "ERROR: " << e.what() << "\n";

        return 1;
    }

    return 0;
}
