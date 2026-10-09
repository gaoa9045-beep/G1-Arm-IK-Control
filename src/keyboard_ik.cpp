#include "pinocchio/fwd.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <algorithm>
#include <cctype>

using namespace unitree::robot;
using LowState = unitree_hg::msg::dds_::LowState_;

std::mutex state_mutex;
LowState latest_state;
std::atomic<bool> received{false};

std::chrono::steady_clock::time_point last_received;

constexpr double STEP = 0.005;
constexpr double TOLERANCE = 0.0005;
constexpr double DAMPING = 0.03;
constexpr double MAX_JOINT_CHANGE = 0.20;

void StateCallback(const void* msg)
{
    if (!msg) return;

    std::lock_guard<std::mutex> lock(state_mutex);

    latest_state = *static_cast<const LowState*>(msg);
    last_received = std::chrono::steady_clock::now();
    received.store(true);
}

struct Arm
{
    std::array<int, 7> qidx;
    std::array<int, 7> vidx;
    pinocchio::FrameIndex frame;
    Eigen::Vector3d start_position;
};

Arm MakeArm(const pinocchio::Model& model,
            const std::string& side)
{
    const std::array<std::string, 7> suffixes = {
        "shoulder_pitch_joint",
        "shoulder_roll_joint",
        "shoulder_yaw_joint",
        "elbow_joint",
        "wrist_roll_joint",
        "wrist_pitch_joint",
        "wrist_yaw_joint"
    };

    Arm arm{};

    for (int i = 0; i < 7; ++i) {
        std::string name = side + "_" + suffixes[i];

        if (!model.existJointName(name))
            throw std::runtime_error("Missing joint: " + name);

        auto id = model.getJointId(name);

        if (model.joints[id].nq() != 1 ||
            model.joints[id].nv() != 1)
            throw std::runtime_error("Invalid joint: " + name);

        arm.qidx[i] = model.joints[id].idx_q();
        arm.vidx[i] = model.joints[id].idx_v();
    }

    std::string frame_name = side + "_wrist_yaw_link";

    if (!model.existFrame(frame_name))
        throw std::runtime_error("Missing frame: " + frame_name);

    arm.frame = model.getFrameId(frame_name);
    return arm;
}

void FillArm(Eigen::VectorXd& q,
             const Arm& arm,
             const LowState& state,
             int start_motor)
{
    for (int i = 0; i < 7; ++i)
        q[arm.qidx[i]] =
            state.motor_state().at(start_motor + i).q();
}

void UpdateFK(const pinocchio::Model& model,
              pinocchio::Data& data,
              const Eigen::VectorXd& q)
{
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
}

Eigen::Vector3d GetPosition(
    const pinocchio::Model& model,
    pinocchio::Data& data,
    const Eigen::VectorXd& q,
    pinocchio::FrameIndex torso,
    pinocchio::FrameIndex wrist)
{
    UpdateFK(model, data, q);

    return (data.oMf[torso].inverse() *
            data.oMf[wrist]).translation();
}

bool SolveIK(const pinocchio::Model& model,
             pinocchio::Data& data,
             Eigen::VectorXd& q,
             const Eigen::VectorXd& q_initial,
             const Arm& arm,
             pinocchio::FrameIndex torso,
             const Eigen::Vector3d& target)
{
    for (int iter = 0; iter < 150; ++iter) {

        UpdateFK(model, data, q);

        Eigen::Vector3d current =
            (data.oMf[torso].inverse() *
             data.oMf[arm.frame]).translation();

        Eigen::Vector3d error_torso = target - current;

        if (error_torso.norm() < TOLERANCE)
            return true;

        // Convert the error to model world coordinates.
        Eigen::Vector3d error_world =
            data.oMf[torso].rotation() * error_torso;

        Eigen::MatrixXd Jfull =
            Eigen::MatrixXd::Zero(6, model.nv);

        pinocchio::computeFrameJacobian(
            model, data, q, arm.frame,
            pinocchio::LOCAL_WORLD_ALIGNED, Jfull
        );

        Eigen::Matrix<double, 3, 7> J;

        for (int j = 0; j < 7; ++j)
            J.col(j) =
                Jfull.block<3, 1>(0, arm.vidx[j]);

        Eigen::Matrix3d A = J * J.transpose();
        A.diagonal().array() += DAMPING * DAMPING;

        Eigen::Matrix<double, 7, 1> dq =
            J.transpose() * A.ldlt().solve(error_world);

        if (!dq.allFinite())
            return false;

        double max_step = dq.cwiseAbs().maxCoeff();

        if (max_step > 0.02)
            dq *= 0.02 / max_step;

        for (int j = 0; j < 7; ++j) {

            int idx = arm.qidx[j];

            double candidate = q[idx] + dq[j];

            candidate = std::clamp(
                candidate,
                q_initial[idx] - MAX_JOINT_CHANGE,
                q_initial[idx] + MAX_JOINT_CHANGE
            );

            double lo = model.lowerPositionLimit[idx];
            double hi = model.upperPositionLimit[idx];

            if (std::isfinite(lo) &&
                std::isfinite(hi) && lo < hi)
                candidate = std::clamp(candidate, lo, hi);

            q[idx] = candidate;
        }
    }

    return false;
}

void PrintPosition(const std::string& name,
                   const Eigen::Vector3d& p)
{
    std::cout << name
              << " X=" << p.x()
              << " Y=" << p.y()
              << " Z=" << p.z()
              << " m\n";
}

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <interface> <urdf>\n";
        return 1;
    }

    try {
        pinocchio::Model model;
        pinocchio::urdf::buildModel(argv[2], model);
        pinocchio::Data data(model);

        if (!model.existFrame("torso_link"))
            throw std::runtime_error("Missing torso_link");

        auto torso = model.getFrameId("torso_link");

        Arm left = MakeArm(model, "left");
        Arm right = MakeArm(model, "right");

        ChannelFactory::Instance()->Init(0, argv[1]);

        auto sub =
            std::make_shared<ChannelSubscriber<LowState>>(
                "rt/lowstate"
            );

        sub->InitChannel(StateCallback, 1);

        std::cout << "Waiting for G1 state...\n";

        for (int i = 0; i < 100 && !received.load(); ++i)
            std::this_thread::sleep_for(
                std::chrono::milliseconds(100));

        if (!received.load())
            throw std::runtime_error("No robot state");

        LowState state;
        std::chrono::steady_clock::time_point timestamp;

        {
            std::lock_guard<std::mutex> lock(state_mutex);
            state = latest_state;
            timestamp = last_received;
        }

        if (std::chrono::steady_clock::now() - timestamp >
            std::chrono::seconds(1))
            throw std::runtime_error("Stale robot state");

        Eigen::VectorXd q = pinocchio::neutral(model);

        const std::array<std::string, 3> waist = {
            "waist_yaw_joint",
            "waist_roll_joint",
            "waist_pitch_joint"
        };

        for (int i = 0; i < 3; ++i) {
            if (!model.existJointName(waist[i]))
                throw std::runtime_error(
                    "Missing joint: " + waist[i]);

            auto id = model.getJointId(waist[i]);

            if (model.joints[id].nq() != 1)
                throw std::runtime_error(
                    "Invalid waist joint");

            q[model.joints[id].idx_q()] =
                state.motor_state().at(12 + i).q();
        }

        FillArm(q, left, state, 15);
        FillArm(q, right, state, 22);

        const Eigen::VectorXd q_initial = q;

        left.start_position =
            GetPosition(model, data, q, torso, left.frame);

        right.start_position =
            GetPosition(model, data, q, torso, right.frame);

        std::cout << "\nOFFLINE KEYBOARD IK\n";
        std::cout << "No motor commands will be sent.\n";

        std::cout << "\nLEFT:  W/S X, A/D Y, Q/E Z\n";
        std::cout << "RIGHT: I/K X, J/L Y, U/O Z\n";
        std::cout << "X: Exit\n";
        std::cout << "Each key requires Enter.\n";

        PrintPosition("Left :", left.start_position);
        PrintPosition("Right:", right.start_position);

        while (true) {

            std::cout << "\nKey > ";

            std::string input;

            if (!std::getline(std::cin, input))
                break;

            if (input.size() != 1) {
                std::cout << "Enter one key.\n";
                continue;
            }

            char key = static_cast<char>(
                std::tolower(
                    static_cast<unsigned char>(input[0])
                )
            );

            if (key == 'x') break;

            Arm* arm = nullptr;
            Eigen::Vector3d delta = Eigen::Vector3d::Zero();

            switch (key) {
                case 'w': arm = &left;  delta.x() =  STEP; break;
                case 's': arm = &left;  delta.x() = -STEP; break;
                case 'a': arm = &left;  delta.y() =  STEP; break;
                case 'd': arm = &left;  delta.y() = -STEP; break;
                case 'q': arm = &left;  delta.z() =  STEP; break;
                case 'e': arm = &left;  delta.z() = -STEP; break;

                case 'i': arm = &right; delta.x() =  STEP; break;
                case 'k': arm = &right; delta.x() = -STEP; break;
                case 'j': arm = &right; delta.y() =  STEP; break;
                case 'l': arm = &right; delta.y() = -STEP; break;
                case 'u': arm = &right; delta.z() =  STEP; break;
                case 'o': arm = &right; delta.z() = -STEP; break;

                default:
                    std::cout << "Unknown key.\n";
                    continue;
            }

            Eigen::Vector3d current =
                GetPosition(model, data, q, torso, arm->frame);

            Eigen::Vector3d target = current + delta;

            // Keep the offline test near the starting pose.
            if ((target - arm->start_position).norm() > 0.05) {
                std::cout << "Workspace test limit reached.\n";
                continue;
            }

            Eigen::VectorXd candidate = q;

            bool success = SolveIK(
                model, data, candidate, q_initial,
                *arm, torso, target
            );

            Eigen::Vector3d achieved =
                GetPosition(model, data, candidate,
                            torso, arm->frame);

            double error = (target - achieved).norm();

            if (!success || error > TOLERANCE) {
                std::cout << "IK failed. Target rejected.\n";
                continue;
            }

            // Update virtual posture only.
            q = candidate;

            std::cout << "IK PASS\n";
            PrintPosition("Target  :", target);
            PrintPosition("Achieved:", achieved);

            std::cout << "Error: "
                      << error * 1000.0
                      << " mm\n";

            std::cout << "Target joint angles (rad): ";

            for (int idx : arm->qidx)
                std::cout << q[idx] << " ";

            std::cout << "\n";

            std::cout << "Offline only: robot did not move.\n";
        }

        std::cout << "\nOffline keyboard IK finished.\n";

    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
