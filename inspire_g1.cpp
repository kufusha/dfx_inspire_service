#include "inspire.h"
#include "param.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unistd.h>

namespace
{
constexpr int kDofPerHand = 6;
constexpr int kTotalDof = 12;
constexpr uint8_t kSafetyProtocolMode = 1;
constexpr uint32_t kSafetyProtocolMagic = 0x494E5350;  // "INSP"
constexpr double kDefaultForceLimitG = 100.0;
constexpr double kDefaultClosingSpeed = 25.0;
constexpr double kDefaultOpeningSpeed = 1000.0;
constexpr double kBackoff = 0.02;
constexpr double kEmergencyBackoff = 0.05;
constexpr double kDirectionEpsilon = 0.002;

using HandVector = Eigen::Matrix<double, kDofPerHand, 1>;

constexpr std::array<const char*, kDofPerHand> kActuatorNames = {
    "pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotate"};

void printForceRow(const char* side, const HandVector& force_n)
{
  std::cout << "  " << std::left << std::setw(5) << side << std::right;
  for (int i = 0; i < kDofPerHand; ++i)
  {
    const double force_g = force_n(i) * 1000.0 / 9.8;
    std::cout << "  " << kActuatorNames[i] << "="
              << std::fixed << std::setprecision(1) << force_g;
  }
  std::cout << std::defaultfloat << std::endl;
}

void setVelocity(inspire::InspireHand& hand, const HandVector& value)
{
  hand.SetVelocity(
      static_cast<int16_t>(value(0)), static_cast<int16_t>(value(1)),
      static_cast<int16_t>(value(2)), static_cast<int16_t>(value(3)),
      static_cast<int16_t>(value(4)), static_cast<int16_t>(value(5)));
}

void setForce(inspire::InspireHand& hand, const HandVector& value)
{
  hand.SetForce(
      static_cast<uint16_t>(value(0)), static_cast<uint16_t>(value(1)),
      static_cast<uint16_t>(value(2)), static_cast<uint16_t>(value(3)),
      static_cast<uint16_t>(value(4)), static_cast<uint16_t>(value(5)));
}
}  // namespace

class InspireRunner
{
public:
  InspireRunner()
  {
    serial1 = std::make_shared<SerialPort>("/dev/ttyUSB1", B115200);
    serial2 = std::make_shared<SerialPort>("/dev/ttyUSB2", B115200);

    // Swap serial1/serial2 here if the physical hands are reversed.
    righthand = std::make_shared<inspire::InspireHand>(serial1, 1);
    lefthand = std::make_shared<inspire::InspireHand>(serial2, 1);

    if (param::calibrate_force)
    {
      std::cout << "[InspireForce] Starting unloaded right-hand calibration "
                   "(about 10 seconds)..." << std::endl;
      righthand->Calibration();
      std::cout << "[InspireForce] Starting unloaded left-hand calibration "
                   "(about 10 seconds)..." << std::endl;
      lefthand->Calibration();
      std::cout << "[InspireForce] Calibration completed." << std::endl;
    }

    handcmd = std::make_shared<unitree::robot::SubscriptionBase<
        unitree_go::msg::dds_::MotorCmds_>>("rt/" + param::ns + "/cmd");
    handcmd->msg_.cmds().resize(kTotalDof);
    handstate = std::make_unique<unitree::robot::RealTimePublisher<
        unitree_go::msg::dds_::MotorStates_>>("rt/" + param::ns + "/state");
    handstate->msg_.states().resize(kTotalDof);

    qcmd.setOnes();
    qstate.setOnes();
    force_n.setZero();
    right_.last_velocity.setConstant(-1.0);
    left_.last_velocity.setConstant(-1.0);
    right_.last_force_limit_g.setConstant(-1.0);
    left_.last_force_limit_g.setConstant(-1.0);

    thread = std::make_shared<unitree::common::RecurrentThread>(
        10000, std::bind(&InspireRunner::run, this));
  }

private:
  struct HandSafetyState
  {
    std::array<bool, kDofPerHand> contact_latched{};
    HandVector hold_position = HandVector::Ones();
    HandVector last_velocity = HandVector::Zero();
    HandVector last_force_limit_g = HandVector::Zero();
    bool position_valid = false;
    bool force_valid = false;
  };

  void readHand(inspire::InspireHand& hand, HandSafetyState& safety, int offset)
  {
    HandVector sample;
    if (hand.GetPosition(sample) == 0)
    {
      qstate.block<kDofPerHand, 1>(offset, 0) = sample;
      safety.position_valid = true;
    }
    else
    {
      safety.position_valid = false;
      for (int i = 0; i < kDofPerHand; ++i)
        handstate->msg_.states()[offset + i].lost()++;
    }

    if (hand.GetForce(sample) == 0)
    {
      force_n.block<kDofPerHand, 1>(offset, 0) = sample.cwiseMax(0.0);
      safety.force_valid = true;
    }
    else
    {
      safety.force_valid = false;
      for (int i = 0; i < kDofPerHand; ++i)
        handstate->msg_.states()[offset + i].lost()++;
    }
  }

  HandVector safeTarget(
      const HandVector& requested,
      const HandVector& measured,
      const HandVector& measured_force_n,
      const HandVector& force_limit_g,
      HandSafetyState& safety)
  {
    HandVector safe = requested;
    for (int i = 0; i < kDofPerHand; ++i)
    {
      // RH56 position convention: 1=open, 0=closed.
      const bool opening = requested(i) >= measured(i) - kDirectionEpsilon;
      if (opening)
      {
        safety.contact_latched[i] = false;
        continue;
      }

      // Missing feedback must never authorize additional closure.
      if (!safety.position_valid || !safety.force_valid)
      {
        safe(i) = measured(i);
        continue;
      }

      const double limit_n = force_limit_g(i) * 9.8 / 1000.0;
      if (!safety.contact_latched[i] && measured_force_n(i) >= limit_n)
      {
        safety.contact_latched[i] = true;
        const bool severe = measured_force_n(i) >= 1.5 * limit_n;
        safety.hold_position(i) = std::min(
            1.0, measured(i) + (severe ? kEmergencyBackoff : kBackoff));
      }

      if (safety.contact_latched[i])
        safe(i) = safety.hold_position(i);
    }
    return safe.cwiseMax(0.0).cwiseMin(1.0);
  }

  void commandHand(
      inspire::InspireHand& hand,
      HandSafetyState& safety,
      int offset)
  {
    const HandVector requested = qcmd.block<kDofPerHand, 1>(offset, 0);
    const HandVector measured = qstate.block<kDofPerHand, 1>(offset, 0);
    const HandVector measured_force = force_n.block<kDofPerHand, 1>(offset, 0);
    HandVector velocity;
    HandVector force_limit_g;

    for (int i = 0; i < kDofPerHand; ++i)
    {
      const auto& cmd = handcmd->msg_.cmds()[offset + i];
      const bool extended = cmd.mode() == kSafetyProtocolMode;
      const bool opening = requested(i) >= measured(i) - kDirectionEpsilon;
      const double requested_speed = extended ? cmd.dq() : 0.0;
      const double requested_force = extended ? cmd.tau() : 0.0;
      velocity(i) = std::clamp(
          requested_speed > 0.0
              ? requested_speed
              : (opening ? kDefaultOpeningSpeed : kDefaultClosingSpeed),
          1.0, 1000.0);
      force_limit_g(i) = std::clamp(
          requested_force > 0.0 ? requested_force : kDefaultForceLimitG,
          1.0, 1000.0);
    }

    // Program the firmware limits before sending the position target.
    if (!velocity.isApprox(safety.last_velocity, 0.5))
    {
      setVelocity(hand, velocity);
      safety.last_velocity = velocity;
    }
    if (!force_limit_g.isApprox(safety.last_force_limit_g, 0.5))
    {
      setForce(hand, force_limit_g);
      safety.last_force_limit_g = force_limit_g;
    }

    hand.SetPosition(safeTarget(
        requested, measured, measured_force, force_limit_g, safety));
  }

  void run()
  {
    // Read feedback even before the first DDS command so startup is
    // observation-only and never moves a hand unexpectedly.
    readHand(*righthand, right_, 0);
    readHand(*lefthand, left_, kDofPerHand);

    const bool command_fresh = !param::monitor_only &&
        !handcmd->isTimeout() && handcmd->msg_.cmds().size() >= kTotalDof;
    if (command_fresh)
    {
      for (int i = 0; i < kTotalDof; ++i)
      {
        qcmd(i) = std::clamp(
            static_cast<double>(handcmd->msg_.cmds()[i].q()), 0.0, 1.0);
      }
      has_received_command_ = true;
    }
    else if (has_received_command_)
    {
      // A lost DDS stream must not leave a previous closing target active.
      qcmd = qstate;
    }

    // Feedback and protection remain local to avoid PC-to-G1 DDS latency.
    // Do not issue any position command until a real DDS command has arrived.
    if (has_received_command_)
    {
      commandHand(*righthand, right_, 0);
      commandHand(*lefthand, left_, kDofPerHand);
    }

    if (handstate->trylock())
    {
      for (int i = 0; i < kTotalDof; ++i)
      {
        auto& state = handstate->msg_.states()[i];
        const HandSafetyState& safety = i < kDofPerHand ? right_ : left_;
        state.q() = qstate(i);
        state.tau_est() = force_n(i);
        state.mode() = safety.contact_latched[i % kDofPerHand] ? 1 : 0;
        state.reserve()[0] = kSafetyProtocolMagic;
      }
      handstate->unlockAndPublish();
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_diagnostic_ >= std::chrono::seconds(1))
    {
      std::cout << "[InspireForce] force[g]  sensors="
                << (right_.force_valid ? "R" : "-")
                << (left_.force_valid ? "L" : "-")
                << "  mode=" << (param::monitor_only ? "MONITOR" : "CONTROL")
                << std::endl;
      printForceRow(
          "right", force_n.block<kDofPerHand, 1>(0, 0));
      printForceRow(
          "left", force_n.block<kDofPerHand, 1>(kDofPerHand, 0));
      last_diagnostic_ = now;
    }
  }

public:
  unitree::common::ThreadPtr thread;
  SerialPort::SharedPtr serial1;
  SerialPort::SharedPtr serial2;
  std::shared_ptr<inspire::InspireHand> lefthand;
  std::shared_ptr<inspire::InspireHand> righthand;
  Eigen::Matrix<double, kTotalDof, 1> qcmd;
  Eigen::Matrix<double, kTotalDof, 1> qstate;
  Eigen::Matrix<double, kTotalDof, 1> force_n;
  std::unique_ptr<unitree::robot::RealTimePublisher<
      unitree_go::msg::dds_::MotorStates_>> handstate;
  std::shared_ptr<unitree::robot::SubscriptionBase<
      unitree_go::msg::dds_::MotorCmds_>> handcmd;

private:
  HandSafetyState right_;
  HandSafetyState left_;
  bool has_received_command_ = false;
  std::chrono::steady_clock::time_point last_diagnostic_{};
};

int main(int argc, char** argv)
{
  param::helper(argc, argv);
  unitree::robot::ChannelFactory::Instance()->Init(0, param::network);
  std::cout << " --- Unitree Robotics ---\n"
            << " Inspire Hand Force-Safe Controller\n"
            << " Default force limit: " << kDefaultForceLimitG << " g\n"
            << " Closing speed: " << kDefaultClosingSpeed << " (raw)\n"
            << " Monitor only: " << (param::monitor_only ? "yes" : "no") << "\n";
  InspireRunner runner;
  while (true)
    sleep(1);
  return 0;
}
