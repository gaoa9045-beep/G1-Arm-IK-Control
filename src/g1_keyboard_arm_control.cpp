#include "pinocchio/fwd.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <unitree/dds_wrapper/common/crc.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cctype>
#include <cstdint>
#include <iostream>
#include <memory>
#include <limits>
#include <mutex>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

using namespace unitree::robot;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
using Clock = std::chrono::steady_clock;

namespace {

constexpr char kStateTopic[] = "rt/lowstate";
constexpr char kArmSdkTopic[] = "rt/arm_sdk";

constexpr int kLeftArmMotorStart = 15;
constexpr int kRightArmMotorStart = 22;
constexpr int kArmJointCount = 14;
constexpr int kWaistJointCount = 3;
constexpr int kArmSdkWeightIndex = 29;
constexpr std::array<int, kWaistJointCount> kWaistMotorIds = {12, 13, 14};

constexpr double kStep = 0.050; // 50 mm per key
constexpr double kIkTolerance = 0.0005;          // 0.5 mm
constexpr double kDamping = 0.03;
constexpr double kMaxJointChangeFromStart = 0.20; // rad
constexpr double kMaxWorkspaceRadius = 0.05;     // 5 cm from start
constexpr double kMaxStateAgeSeconds = 0.25;
constexpr double kMaxFeedbackError = 0.05;        // rad; reject new targets until the arms settle
constexpr double kMaxWaistHoldError = 0.05;        // rad; release ArmSDK if waist drifts from its hold target
constexpr double kControlDt = 0.02;              // 50 Hz
constexpr double kMaxCommandVelocity = 0.20;      // rad/s
constexpr float kWeightStep = 0.004f;             // 0 -> 1 in about 5 s at 50 Hz
constexpr float kKp = 60.0f;
constexpr float kKd = 1.5f;

constexpr std::array<int, kArmJointCount> kArmMotorIds = {
    15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28
};

std::mutex g_state_mutex;
LowState g_latest_state{};
bool g_have_state = false;
Clock::time_point g_last_state_time{};

std::mutex g_target_mutex;
std::array<float, kArmJointCount> g_target_q{};

std::atomic<bool> g_stop_requested{false};
std::atomic<bool> g_control_ready{false};
volatile std::sig_atomic_t g_signal_stop = 0;
uint8_t g_initial_mode_machine = 0;

void SignalHandler(int) {
    g_signal_stop = 1;
}

bool ShouldStop() {
    return g_signal_stop != 0 || g_stop_requested.load();
}

void RequestStop() {
    g_stop_requested.store(true);
}

void StateCallback(const void* message) {
    if (!message) return;

    const auto* state = static_cast<const LowState*>(message);
    std::lock_guard<std::mutex> lock(g_state_mutex);
    g_latest_state = *state;
    g_last_state_time = Clock::now();
    g_have_state = true;
}

bool CopyLatestState(LowState& state, Clock::time_point& stamp) {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    if (!g_have_state) return false;
    state = g_latest_state;
    stamp = g_last_state_time;
    return true;
}

bool GetFreshState(LowState& state) {
    Clock::time_point stamp;
    if (!CopyLatestState(state, stamp)) return false;
    const double age = std::chrono::duration<double>(Clock::now() - stamp).count();
    return age <= kMaxStateAgeSeconds;
}

struct Arm {
    std::array<int, 7> qidx{};
    std::array<int, 7> vidx{};
    pinocchio::FrameIndex frame{};
    Eigen::Vector3d start_position = Eigen::Vector3d::Zero();
    int motor_start = 0;
};

Arm MakeArm(const pinocchio::Model& model,
            const std::string& side,
            int motor_start) {
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
    arm.motor_start = motor_start;

    for (int i = 0; i < 7; ++i) {
        const std::string joint_name = side + "_" + suffixes[i];
        if (!model.existJointName(joint_name)) {
            throw std::runtime_error("Missing URDF joint: " + joint_name);
        }

        const auto joint_id = model.getJointId(joint_name);
        if (model.joints[joint_id].nq() != 1 || model.joints[joint_id].nv() != 1) {
            throw std::runtime_error("Expected a one-DoF joint: " + joint_name);
        }

        arm.qidx[i] = model.joints[joint_id].idx_q();
        arm.vidx[i] = model.joints[joint_id].idx_v();
    }

    const std::string frame_name = side + "_hand_palm_link";
    if (!model.existFrame(frame_name)) {
        throw std::runtime_error("Missing URDF frame: " + frame_name);
    }
    arm.frame = model.getFrameId(frame_name);
    return arm;
}

void FillArmFromState(Eigen::VectorXd& q,
                      const Arm& arm,
                      const LowState& state) {
    for (int i = 0; i < 7; ++i) {
        q[arm.qidx[i]] = state.motor_state().at(arm.motor_start + i).q();
    }
}

bool FillWaistFromState(const pinocchio::Model& model,
                        Eigen::VectorXd& q,
                        const LowState& state,
                        const std::array<std::string, 3>& waist_names) {
    for (int i = 0; i < 3; ++i) {
        if (!model.existJointName(waist_names[i])) {
            throw std::runtime_error("Missing URDF joint: " + waist_names[i]);
        }
        const auto joint_id = model.getJointId(waist_names[i]);
        if (model.joints[joint_id].nq() != 1) {
            throw std::runtime_error("Expected a one-DoF joint: " + waist_names[i]);
        }
        const double measured = state.motor_state().at(kWaistMotorIds[i]).q();
        if (!std::isfinite(measured)) return false;
        q[model.joints[joint_id].idx_q()] = measured;
    }
    return true;
}

bool CopyWaistPositionsFromState(
    const LowState& state,
    std::array<float, kWaistJointCount>& waist_q) {
    for (int i = 0; i < kWaistJointCount; ++i) {
        const float measured = state.motor_state().at(kWaistMotorIds[i]).q();
        if (!std::isfinite(measured)) return false;
        waist_q[i] = measured;
    }
    return true;
}

bool CopyArmPositionsFromState(const LowState& state,
                               std::array<float, kArmJointCount>& arm_q) {
    for (int i = 0; i < kArmJointCount; ++i) {
        const float measured = state.motor_state().at(kArmMotorIds[i]).q();
        if (!std::isfinite(measured)) return false;
        arm_q[i] = measured;
    }
    return true;
}

bool ArmJointSpeedsAcceptable(const LowState& state,
                              double max_allowed_speed,
                              double& max_speed) {
    max_speed = 0.0;
    for (const int motor_id : kArmMotorIds) {
        const double measured_dq = state.motor_state().at(motor_id).dq();
        if (!std::isfinite(measured_dq)) return false;
        max_speed = std::max(max_speed, std::abs(measured_dq));
    }
    return max_speed <= max_allowed_speed;
}

bool WaistJointSpeedsAcceptable(const LowState& state,
                                double max_allowed_speed,
                                double& max_speed) {
    max_speed = 0.0;
    for (const int motor_id : kWaistMotorIds) {
        const double measured_dq = state.motor_state().at(motor_id).dq();
        if (!std::isfinite(measured_dq)) return false;
        max_speed = std::max(max_speed, std::abs(measured_dq));
    }
    return max_speed <= max_allowed_speed;
}

bool WaistHoldErrorAcceptable(
    const LowState& state,
    const std::array<float, kWaistJointCount>& hold_target,
    double& max_error) {
    max_error = 0.0;
    for (int i = 0; i < kWaistJointCount; ++i) {
        const double measured = state.motor_state().at(kWaistMotorIds[i]).q();
        if (!std::isfinite(measured)) {
            max_error = std::numeric_limits<double>::infinity();
            return false;
        }
        max_error = std::max(max_error,
                             std::abs(measured - hold_target[i]));
    }
    return max_error <= kMaxWaistHoldError;
}

void UpdateFK(const pinocchio::Model& model,
              pinocchio::Data& data,
              const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
}

Eigen::Vector3d GetPalmPosition(const pinocchio::Model& model,
                                pinocchio::Data& data,
                                const Eigen::VectorXd& q,
                                pinocchio::FrameIndex torso,
                                pinocchio::FrameIndex palm) {
    UpdateFK(model, data, q);
    return (data.oMf[torso].inverse() * data.oMf[palm]).translation();
}

bool SolvePositionIK(const pinocchio::Model& model,
                     pinocchio::Data& data,
                     Eigen::VectorXd& q,
                     const Eigen::VectorXd& q_initial,
                     const Arm& arm,
                     pinocchio::FrameIndex torso,
                     const Eigen::Vector3d& target) {
    for (int iter = 0; iter < 150; ++iter) {
        UpdateFK(model, data, q);

        const Eigen::Vector3d current =
            (data.oMf[torso].inverse() * data.oMf[arm.frame]).translation();
        const Eigen::Vector3d error_torso = target - current;

        if (error_torso.norm() < kIkTolerance) return true;

        const Eigen::Vector3d error_world =
            data.oMf[torso].rotation() * error_torso;

        Eigen::MatrixXd Jfull = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::computeFrameJacobian(
            model, data, q, arm.frame,
            pinocchio::LOCAL_WORLD_ALIGNED, Jfull);

        Eigen::Matrix<double, 3, 7> J;
        for (int j = 0; j < 7; ++j) {
            J.col(j) = Jfull.block<3, 1>(0, arm.vidx[j]);
        }

        Eigen::Matrix3d A = J * J.transpose();
        A.diagonal().array() += kDamping * kDamping;

        const Eigen::Matrix<double, 7, 1> dq =
            J.transpose() * A.ldlt().solve(error_world);
        if (!dq.allFinite()) return false;

        Eigen::Matrix<double, 7, 1> limited_dq = dq;
        const double max_step = limited_dq.cwiseAbs().maxCoeff();
        if (max_step > 0.02) limited_dq *= 0.02 / max_step;

        for (int j = 0; j < 7; ++j) {
            const int idx = arm.qidx[j];
            double candidate = q[idx] + limited_dq[j];
            candidate = std::clamp(
                candidate,
                q_initial[idx] - kMaxJointChangeFromStart,
                q_initial[idx] + kMaxJointChangeFromStart);

            const double lo = model.lowerPositionLimit[idx];
            const double hi = model.upperPositionLimit[idx];
            if (std::isfinite(lo) && std::isfinite(hi) && lo < hi) {
                candidate = std::clamp(candidate, lo, hi);
            }
            q[idx] = candidate;
        }
    }
    return false;
}

void PrintPosition(const std::string& label, const Eigen::Vector3d& p) {
    std::cout << label
              << " X=" << p.x()
              << " Y=" << p.y()
              << " Z=" << p.z()
              << " m\n";
}

void ArmSdkControlLoop(
    std::shared_ptr<ChannelPublisher<LowCmd>> publisher,
    std::array<float, kArmJointCount> initial_q,
    std::array<float, kWaistJointCount> waist_hold_q) {
    std::array<float, kArmJointCount> command_q = initial_q;
    float weight = 0.0f;
    bool waist_error_reported = false;
    auto next_tick = Clock::now();

    while (true) {
        next_tick += std::chrono::milliseconds(20);

        LowState state{};
        Clock::time_point stamp{};
        const bool have_state = CopyLatestState(state, stamp);
        bool state_fresh = false;
        if (have_state) {
            const double age = std::chrono::duration<double>(Clock::now() - stamp).count();
            state_fresh = age <= kMaxStateAgeSeconds;
        }
        if (!state_fresh ||
            (have_state && state.mode_machine() != g_initial_mode_machine)) {
            // Stop accepting new targets and ramp ArmSDK weight down if feedback
            // becomes stale or the robot's mode_machine changes during control.
            RequestStop();
        }
        if (state_fresh) {
            double max_waist_error = 0.0;
            if (!WaistHoldErrorAcceptable(state, waist_hold_q,
                                          max_waist_error)) {
                if (!waist_error_reported) {
                    std::cerr << "Waist hold error exceeded "
                              << kMaxWaistHoldError
                              << " rad (max error=" << max_waist_error
                              << " rad); releasing ArmSDK.\n";
                    waist_error_reported = true;
                }
                RequestStop();
            }
        }

        const bool stopping = ShouldStop();
        std::array<float, kArmJointCount> desired_q{};
        if (stopping) {
            // Freeze the command at the current ramped position while releasing control.
            desired_q = command_q;
        } else {
            std::lock_guard<std::mutex> lock(g_target_mutex);
            desired_q = g_target_q;
        }

        const float max_step = static_cast<float>(kMaxCommandVelocity * kControlDt);
        for (int i = 0; i < kArmJointCount; ++i) {
            const float delta = desired_q[i] - command_q[i];
            command_q[i] += std::clamp(delta, -max_step, max_step);
        }

        if (stopping) {
            weight = std::max(0.0f, weight - kWeightStep);
        } else {
            weight = std::min(1.0f, weight + kWeightStep);
            if (weight >= 1.0f) g_control_ready.store(true);
        }

        LowCmd command{};
        command.mode_pr(0);
        command.mode_machine(have_state ? state.mode_machine() : g_initial_mode_machine);

        for (int i = 0; i < kArmJointCount; ++i) {
            auto& motor = command.motor_cmd().at(kArmMotorIds[i]);
            motor.mode(1);
            motor.q(command_q[i]);
            motor.dq(0.0f);
            motor.tau(0.0f);
            motor.kp(kKp);
            motor.kd(kKd);
        }

        // Hold all three waist joints at the measured posture captured immediately
        // before ArmSDK enable: yaw=12, roll=13, pitch=14. Do not command zero.
        for (int i = 0; i < kWaistJointCount; ++i) {
            auto& motor = command.motor_cmd().at(kWaistMotorIds[i]);
            motor.mode(1);
            motor.q(waist_hold_q[i]);
            motor.dq(0.0f);
            motor.tau(0.0f);
            motor.kp(kKp);
            motor.kd(kKd);
        }

        // G1 ArmSDK weight: motor_cmd[29].q(), 0=disabled, 1=enabled.
        command.motor_cmd().at(kArmSdkWeightIndex).q(weight);
        command.crc() = crc32_core(
            reinterpret_cast<uint32_t*>(&command),
            (sizeof(LowCmd) >> 2) - 1);
        publisher->Write(command);

        if (stopping && weight <= 0.0f) break;
        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "\nArmSDK command loop stopped; weight ramped to zero.\n";
}

bool TrackingErrorAcceptable(const LowState& state,
                             const std::array<float, kArmJointCount>& target,
                             double& max_error) {
    max_error = 0.0;
    for (int i = 0; i < kArmJointCount; ++i) {
        const double measured = state.motor_state().at(kArmMotorIds[i]).q();
        if (!std::isfinite(measured)) return false;
        max_error = std::max(max_error, std::abs(measured - target[i]));
    }
    return max_error <= kMaxFeedbackError;
}

bool WaitForArmsAndWaistToSettle(
    const std::array<float, kArmJointCount>& arm_target,
    const std::array<float, kWaistJointCount>& waist_target,
    double max_allowed_speed,
    double timeout_seconds) {
    const auto deadline = Clock::now() +
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(timeout_seconds));
    int stable_samples = 0;
    constexpr int kRequiredStableSamples = 10;

    while (!ShouldStop() && Clock::now() < deadline) {
        LowState state{};
        if (!GetFreshState(state)) {
            stable_samples = 0;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        if (state.mode_machine() != g_initial_mode_machine) return false;

        double max_speed = 0.0;
        double max_error = 0.0;
        const bool arm_speed_ok = ArmJointSpeedsAcceptable(
            state, max_allowed_speed, max_speed);
        const bool arm_tracking_ok = TrackingErrorAcceptable(
            state, arm_target, max_error);

        double max_waist_speed = 0.0;
        double max_waist_error = 0.0;
        const bool waist_speed_ok = WaistJointSpeedsAcceptable(
            state, max_allowed_speed, max_waist_speed);
        const bool waist_tracking_ok = WaistHoldErrorAcceptable(
            state, waist_target, max_waist_error);

        if (arm_speed_ok && arm_tracking_ok &&
            waist_speed_ok && waist_tracking_ok) {
            ++stable_samples;
            if (stable_samples >= kRequiredStableSamples) return true;
        } else {
            stable_samples = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <network_interface> <urdf_path>\n";
        return 1;
    }

    std::thread controller;

    try {
        pinocchio::Model model;
        pinocchio::urdf::buildModel(argv[2], model);
        pinocchio::Data data(model);

        if (!model.existFrame("torso_link")) {
            throw std::runtime_error("Missing URDF frame: torso_link");
        }
        const auto torso = model.getFrameId("torso_link");
        Arm left = MakeArm(model, "left", kLeftArmMotorStart);
        Arm right = MakeArm(model, "right", kRightArmMotorStart);

        ChannelFactory::Instance()->Init(0, argv[1]);
        auto subscriber = std::make_shared<ChannelSubscriber<LowState>>(kStateTopic);
        subscriber->InitChannel(StateCallback, 1);

        std::cout << "Waiting for fresh G1 state...\n";
        LowState state{};
        bool fresh_state_received = false;
        for (int i = 0; i < 100 && !ShouldStop(); ++i) {
            if (GetFreshState(state)) {
                fresh_state_received = true;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!fresh_state_received) {
            throw std::runtime_error("No fresh rt/lowstate received; no commands were sent");
        }

        g_initial_mode_machine = state.mode_machine();
        std::cout << "Robot mode_machine: " << static_cast<unsigned>(state.mode_machine()) << "\n";

        Eigen::VectorXd q = pinocchio::neutral(model);
        const std::array<std::string, 3> waist_names = {
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"
        };
        if (!FillWaistFromState(model, q, state, waist_names)) {
            throw std::runtime_error("Non-finite initial waist joint state");
        }
        FillArmFromState(q, left, state);
        FillArmFromState(q, right, state);
        Eigen::VectorXd q_initial = q;

        left.start_position = GetPalmPosition(model, data, q, torso, left.frame);
        right.start_position = GetPalmPosition(model, data, q, torso, right.frame);

        std::array<float, kArmJointCount> initial_arm_q{};
        std::array<float, kWaistJointCount> initial_waist_q{};
        if (!CopyArmPositionsFromState(state, initial_arm_q)) {
            throw std::runtime_error("Non-finite initial arm joint state");
        }

        std::cout << "\nG1 KEYBOARD PALM IK + ArmSDK\n"
                  << "WARNING: after explicit confirmation this program WILL publish real arm commands.\n"
                  << "It starts from measured joint positions; it does not command a home pose.\n"
                  << "Keep the robot upright and stationary, clear the workspace, and have the physical emergency stop ready.\n"
                  << "This program does not switch motion-service modes; verify the required ArmSDK mode separately.\n\n";
        PrintPosition("Left palm (torso frame): ", left.start_position);
        PrintPosition("Right palm (torso frame):", right.start_position);
        std::cout << "\nKeys (each followed by Enter):\n"
                  << " LEFT:  W/S = X +/-; A/D = Y +/-; Q/E = Z +/-\n"
                  << " RIGHT: I/K = X +/-; J/L = Y +/-; U/O = Z +/-\n"
                  << " X = ramp ArmSDK weight to zero and exit\n"
                  << "Limits: 50 mm per key, 5 cm workspace radius, 0.20 rad joint deviation from startup.\n\n"
                  << "No motor commands have been sent yet.\n"
                  << "Type exactly ENABLE_ARMSDK to enable live control: " << std::flush;

        std::string confirmation;
        if (!std::getline(std::cin, confirmation) || confirmation != "ENABLE_ARMSDK") {
            std::cout << "Confirmation not received. Exiting without publishing motor commands.\n";
            return 0;
        }

        // Re-read the robot immediately before enabling. The operator may have
        // taken time to confirm, so the first posture snapshot may no longer be current.
        LowState enable_state{};
        if (!GetFreshState(enable_state)) {
            std::cerr << "No fresh G1 state immediately before enabling; no commands were sent.\n";
            return 1;
        }
        if (enable_state.mode_machine() != g_initial_mode_machine) {
            std::cerr << "Robot mode_machine changed while awaiting confirmation; aborting without publishing.\n";
            return 1;
        }
        if (!FillWaistFromState(model, q, enable_state, waist_names)) {
            std::cerr << "Non-finite waist feedback immediately before enabling; no commands were sent.\n";
            return 1;
        }
        FillArmFromState(q, left, enable_state);
        FillArmFromState(q, right, enable_state);
        q_initial = q;
        left.start_position = GetPalmPosition(model, data, q, torso, left.frame);
        right.start_position = GetPalmPosition(model, data, q, torso, right.frame);
        if (!CopyArmPositionsFromState(enable_state, initial_arm_q)) {
            std::cerr << "Non-finite arm feedback immediately before enabling; no commands were sent.\n";
            return 1;
        }
        if (!CopyWaistPositionsFromState(enable_state, initial_waist_q)) {
            std::cerr << "Non-finite waist feedback immediately before enabling; no commands were sent.\n";
            return 1;
        }
        double max_startup_speed = 0.0;
        if (!ArmJointSpeedsAcceptable(enable_state, 0.15, max_startup_speed)) {
            std::cerr << "Arm joints are moving too quickly to enable safely (max |dq|="
                      << max_startup_speed << " rad/s). Let the arms settle and restart.\n";
            return 1;
        }
        double max_startup_waist_speed = 0.0;
        if (!WaistJointSpeedsAcceptable(enable_state, 0.15,
                                        max_startup_waist_speed)) {
            std::cerr << "Waist joints are moving too quickly to enable safely (max |dq|="
                      << max_startup_waist_speed
                      << " rad/s). Let the waist settle and restart.\n";
            return 1;
        }
        {
            std::lock_guard<std::mutex> lock(g_target_mutex);
            g_target_q = initial_arm_q;
        }

        // Cartesian goals are accumulated from the startup palm positions.
        // Each key changes one torso-frame coordinate by the configured 50 mm,
        // rather than adding the step to a potentially drifting feedback pose.
        Eigen::Vector3d left_goal = left.start_position;
        Eigen::Vector3d right_goal = right.start_position;

        std::cout << "Fresh posture captured immediately before enabling.\n";
        PrintPosition("Enable left palm (torso frame): ", left.start_position);
        PrintPosition("Enable right palm (torso frame):", right.start_position);
        std::cout << "Max arm joint speed at enable: " << max_startup_speed << " rad/s\n";
        std::cout << "Max waist joint speed at enable: "
                  << max_startup_waist_speed << " rad/s\n";
        std::cout << "Waist hold targets (rad): yaw=" << initial_waist_q[0]
                  << ", roll=" << initial_waist_q[1]
                  << ", pitch=" << initial_waist_q[2] << "\n";

        // Create the live command publisher only after confirmation and fresh-state checks.
        auto publisher = std::make_shared<ChannelPublisher<LowCmd>>(kArmSdkTopic);
        publisher->InitChannel();

        std::cout << "ArmSDK publisher initialized. Enabling from the refreshed measured posture with a 5 s weight ramp.\n";
        controller = std::thread(ArmSdkControlLoop, publisher,
                                 initial_arm_q, initial_waist_q);

        std::cout << "Waiting for the 5 s ArmSDK handover ramp to complete; keyboard targets are disabled.\n";
        while (!ShouldStop() && !g_control_ready.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (ShouldStop()) {
            if (controller.joinable()) controller.join();
            std::cerr << "ArmSDK handover did not complete; exiting without accepting targets.\n";
            return 1;
        }

        std::cout << "Weight ramp complete. Waiting for the arms and waist to settle before accepting keyboard input...\n";
        if (!WaitForArmsAndWaistToSettle(
                initial_arm_q, initial_waist_q, 0.05, 8.0)) {
            std::cerr << "Arms/waist did not settle within the allowed time, or state/mode became invalid. Releasing ArmSDK.\n";
            RequestStop();
            if (controller.joinable()) controller.join();
            return 1;
        }
        std::cout << "Arms and waist settled. Waist yaw/roll/pitch are held at the captured startup angles.\n";
        std::cout << "Each accepted key moves the accumulated palm target by 50 mm on one axis.\n";

        while (!ShouldStop()) {
            std::cout << "\nKey > " << std::flush;
            pollfd input_fd{};
            input_fd.fd = STDIN_FILENO;
            input_fd.events = POLLIN;

            bool input_ready = false;
            while (!ShouldStop()) {
                const int result = ::poll(&input_fd, 1, 100);
                if (result < 0) {
                    if (errno == EINTR) continue;
                    std::cerr << "Input polling failed; requesting controlled release.\n";
                    RequestStop();
                    break;
                }
                if (result == 0) continue;
                if (input_fd.revents & (POLLIN | POLLHUP)) {
                    input_ready = true;
                    break;
                }
            }
            if (ShouldStop() || !input_ready) break;

            std::string input;
            if (!std::getline(std::cin, input)) {
                RequestStop();
                break;
            }
            if (input.size() != 1) {
                std::cout << "Enter one key only.\n";
                continue;
            }

            const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(input[0])));
            if (key == 'x') {
                RequestStop();
                break;
            }

            Arm* arm = nullptr;
            Eigen::Vector3d delta = Eigen::Vector3d::Zero();
            switch (key) {
                case 'w': arm = &left;  delta.x() =  kStep; break;
                case 's': arm = &left;  delta.x() = -kStep; break;
                case 'a': arm = &left;  delta.y() =  kStep; break;
                case 'd': arm = &left;  delta.y() = -kStep; break;
                case 'q': arm = &left;  delta.z() =  kStep; break;
                case 'e': arm = &left;  delta.z() = -kStep; break;
                case 'i': arm = &right; delta.x() =  kStep; break;
                case 'k': arm = &right; delta.x() = -kStep; break;
                case 'j': arm = &right; delta.y() =  kStep; break;
                case 'l': arm = &right; delta.y() = -kStep; break;
                case 'u': arm = &right; delta.z() =  kStep; break;
                case 'o': arm = &right; delta.z() = -kStep; break;
                default:
                    std::cout << "Unknown key.\n";
                    continue;
            }

            LowState live_state{};
            if (!GetFreshState(live_state)) {
                std::cerr << "State is stale; refusing new target and releasing ArmSDK.\n";
                RequestStop();
                break;
            }

            if (live_state.mode_machine() != g_initial_mode_machine) {
                std::cerr << "Robot mode_machine changed; refusing the target and releasing ArmSDK.\n";
                RequestStop();
                break;
            }

            std::array<float, kArmJointCount> target_snapshot{};
            {
                std::lock_guard<std::mutex> lock(g_target_mutex);
                target_snapshot = g_target_q;
            }
            double max_feedback_error = 0.0;
            if (!TrackingErrorAcceptable(live_state, target_snapshot, max_feedback_error)) {
                std::cout << "Target rejected: arm tracking error "
                          << max_feedback_error << " rad exceeds "
                          << kMaxFeedbackError << " rad. Wait for the arms to settle.\n";
                continue;
            }

            // Build the IK starting configuration from current feedback, not the
            // last commanded solution. This avoids accumulating target/feedback drift.
            if (!FillWaistFromState(model, q, live_state, waist_names)) {
                std::cerr << "Non-finite waist feedback; refusing the target and releasing ArmSDK.\n";
                RequestStop();
                break;
            }
            FillArmFromState(q, left, live_state);
            FillArmFromState(q, right, live_state);

            const Eigen::Vector3d current_feedback =
                GetPalmPosition(model, data, q, torso, arm->frame);
            Eigen::Vector3d& accumulated_goal = (arm == &left) ? left_goal : right_goal;
            const Eigen::Vector3d target = accumulated_goal + delta;
            if ((target - arm->start_position).norm() > kMaxWorkspaceRadius) {
                std::cout << "Workspace limit reached; target rejected.\n";
                continue;
            }

            Eigen::VectorXd candidate = q;
            const bool success = SolvePositionIK(
                model, data, candidate, q_initial, *arm, torso, target);
            const Eigen::Vector3d achieved =
                GetPalmPosition(model, data, candidate, torso, arm->frame);
            const double error = (target - achieved).norm();

            if (!success || error > kIkTolerance) {
                std::cout << "IK failed; target rejected.\n";
                continue;
            }

            q = candidate;
            accumulated_goal = target;
            const int target_offset = arm->motor_start - kLeftArmMotorStart;
            {
                std::lock_guard<std::mutex> lock(g_target_mutex);
                for (int i = 0; i < 7; ++i) {
                    g_target_q[target_offset + i] = static_cast<float>(q[arm->qidx[i]]);
                }
            }

            std::cout << "IK PASS; joint target accepted. Command loop applies a velocity limit.\n";
            PrintPosition("Palm feedback before key: ", current_feedback);
            PrintPosition("Palm accumulated target : ", target);
            PrintPosition("Palm IK solution         : ", achieved);
            std::cout << "IK position error: " << error * 1000.0 << " mm\n";
            std::cout << "Max measured tracking error before this move: "
                      << max_feedback_error << " rad\n";
        }

        RequestStop();
        if (controller.joinable()) controller.join();
        std::cout << "Live keyboard ArmSDK control finished.\n";
    } catch (const std::exception& e) {
        RequestStop();
        if (controller.joinable()) controller.join();
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
