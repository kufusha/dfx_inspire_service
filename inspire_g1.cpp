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
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>
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
constexpr double kDirectionEpsilon = 0.002;
constexpr int kTareSamples = 31;
constexpr int kTareMaxAttempts = 500;
constexpr int kTareSettleSeconds = 3;
constexpr double kCalibrationOpenPosition = 0.90;
constexpr double kCalibrationOpeningSpeed = 100.0;
constexpr int kOpenMaxAttempts = 200;
constexpr int kContactConfirmSamples = 3;
constexpr double kForceFilterAlpha = 0.35;
constexpr double kBaselineDriftAlpha = 0.001;
constexpr double kTareMaxSpreadG = 50.0;
constexpr int kProtectivePoseMaxAttempts = 450;
constexpr double kProtectivePoseTolerance = 0.03;
constexpr int kProtectiveMaxFeedbackFailures = 3;
constexpr int kPositionReadRetries = 3;
constexpr int kProtectivePreflightStableSamples = 5;
constexpr int kProtectivePreflightMaxAttempts = 100;
constexpr double kProtectivePreflightMaxNetForceG = 50.0;
constexpr int kProtectiveContactConfirmSamples = 3;
constexpr int kProtectiveReachedStableSamples = 10;

using HandVector = Eigen::Matrix<double, kDofPerHand, 1>;

constexpr std::array<const char*, kDofPerHand> kActuatorNames = {
    "pinky", "ring", "middle", "index", "thumb_bend", "thumb_rotate"};
const HandVector kProtectivePose =
    (HandVector() << 0.0, 0.0, 0.0, 0.0, 0.270, 0.978).finished();

void printForceRow(const char* side, const HandVector& force_n)
{
  const auto old_flags = std::cout.flags();
  const auto old_precision = std::cout.precision();
  std::cout << "  " << std::left << std::setw(5) << side << std::right;
  for (int i = 0; i < kDofPerHand; ++i)
  {
    const double force_g = force_n(i) * 1000.0 / 9.8;
    std::cout << "  " << kActuatorNames[i] << "="
              << std::fixed << std::setprecision(1) << force_g;
  }
  std::cout.flags(old_flags);
  std::cout.precision(old_precision);
  std::cout << std::endl;
}

double median(std::vector<double>& values)
{
  const auto middle = values.begin() + values.size() / 2;
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

bool isFullyOpen(const HandVector& position)
{
  return (position.array() >= kCalibrationOpenPosition).all();
}

void printPositionRow(const char* side, const HandVector& position)
{
  const auto old_flags = std::cout.flags();
  const auto old_precision = std::cout.precision();
  std::cout << "  " << std::left << std::setw(5) << side << std::right;
  for (int i = 0; i < kDofPerHand; ++i)
  {
    std::cout << "  " << kActuatorNames[i] << "="
              << std::fixed << std::setprecision(3) << position(i);
  }
  std::cout.flags(old_flags);
  std::cout.precision(old_precision);
  std::cout << std::endl;
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
    serial1 = std::make_shared<SerialPort>("/dev/ttyUSB1", B115200, 10);
    serial2 = std::make_shared<SerialPort>("/dev/ttyUSB2", B115200, 10);

    // Swap serial1/serial2 here if the physical hands are reversed.
    righthand = std::make_shared<inspire::InspireHand>(serial1, 1);
    lefthand = std::make_shared<inspire::InspireHand>(serial2, 1);

    qcmd.setOnes();
    qstate.setOnes();
    raw_force_n.setZero();
    force_n.setZero();
    force_offset_n.setZero();
    right_.last_velocity.setConstant(-1.0);
    left_.last_velocity.setConstant(-1.0);
    right_.last_force_limit_g.setConstant(-1.0);
    left_.last_force_limit_g.setConstant(-1.0);

    if (param::calibrate_force)
    {
      if (param::open_before_calibration)
        openHandsForCalibration();
      else
        requireHandsOpen();

      std::cout << "[InspireForce] Starting unloaded right-hand calibration "
                   "(about 10 seconds)..." << std::endl;
      reportCalibrationResult("right", righthand->Calibration());
      std::cout << "[InspireForce] Starting unloaded left-hand calibration "
                   "(about 10 seconds)..." << std::endl;
      reportCalibrationResult("left", lefthand->Calibration());
      std::cout << "[InspireForce] Calibration completed." << std::endl;
      requireHandsOpen();
      tareForceSensors();
      saveForceBaseline();
    }
    else if (!loadForceBaseline())
    {
      if (!param::monitor_only)
        fail("no valid force baseline; run --calibrate-force "
             "--open-before-calibration --monitor-only first");
      std::cout << "[InspireForce] WARNING: no saved baseline; monitor output "
                   "will show raw force and CONTROL remains unavailable."
                << std::endl;
    }

    if (param::return_to_protective_pose)
    {
      if (!baseline_valid_)
        fail("a valid saved force baseline is required for protective-pose "
             "motion");
      moveToProtectivePose();
    }

    handcmd = std::make_shared<unitree::robot::SubscriptionBase<
        unitree_go::msg::dds_::MotorCmds_>>("rt/" + param::ns + "/cmd");
    handcmd->msg_.cmds().resize(kTotalDof);
    handstate = std::make_unique<unitree::robot::RealTimePublisher<
        unitree_go::msg::dds_::MotorStates_>>("rt/" + param::ns + "/state");
    handstate->msg_.states().resize(kTotalDof);

    thread = std::make_shared<unitree::common::RecurrentThread>(
        10000, std::bind(&InspireRunner::run, this));
  }

private:
  struct HandSafetyState
  {
    std::array<bool, kDofPerHand> contact_latched{};
    std::array<int, kDofPerHand> above_limit_count{};
    HandVector hold_position = HandVector::Ones();
    HandVector last_velocity = HandVector::Zero();
    HandVector last_force_limit_g = HandVector::Zero();
    bool position_valid = false;
    bool force_valid = false;
  };

  [[noreturn]] void fail(const std::string& message)
  {
    std::cerr << "[InspireForce] ERROR: " << message << std::endl;
    std::exit(1);
  }

  void reportCalibrationResult(const char* side, int16_t result)
  {
    if (result == 1)
      fail(std::string(side) + " hand force-calibration command send failed");
    if (result == 2)
    {
      std::cout << "[InspireForce] WARNING: " << side
                << " hand sent no recognized calibration ACK; continuing "
                   "because this is firmware-dependent."
                << std::endl;
    }
  }

  bool readBothPositions(HandVector& right, HandVector& left)
  {
    return righthand->GetPosition(right) == 0 &&
           lefthand->GetPosition(left) == 0;
  }

  int readPositionWithRetries(
      inspire::InspireHand& hand, HandVector& position)
  {
    int result = 1;
    for (int attempt = 0; attempt < kPositionReadRetries; ++attempt)
    {
      result = hand.GetPosition(position);
      if (result == 0)
        return 0;
    }
    return result;
  }

  void requireHandsOpen()
  {
    HandVector right;
    HandVector left;
    for (int attempt = 0; attempt < 10; ++attempt)
    {
      if (readBothPositions(right, left))
      {
        if (!isFullyOpen(right) || !isFullyOpen(left))
          fail("force calibration requires both hands fully open; use "
               "--open-before-calibration after clearing the workspace");
        return;
      }
    }
    fail("could not verify hand positions before force calibration");
  }

  void openHandsForCalibration()
  {
    std::cout << "[InspireForce] WARNING: opening both hands for calibration. "
                 "Keep the workspace clear." << std::endl;
    HandVector speed = HandVector::Constant(kCalibrationOpeningSpeed);
    setVelocity(*righthand, speed);
    setVelocity(*lefthand, speed);
    righthand->SetPosition(HandVector::Ones());
    lefthand->SetPosition(HandVector::Ones());

    HandVector right;
    HandVector left;
    bool received_position = false;
    int right_result = -1;
    int left_result = -1;
    for (int attempt = 0; attempt < kOpenMaxAttempts; ++attempt)
    {
      right_result = righthand->GetPosition(right);
      left_result = lefthand->GetPosition(left);
      if (right_result == 0 && left_result == 0)
      {
        received_position = true;
        if (isFullyOpen(right) && isFullyOpen(left))
        {
          std::cout << "[InspireForce] Both hands verified open." << std::endl;
          return;
        }
      }
      usleep(100000);
    }
    if (received_position)
    {
      std::cerr << "[InspireForce] Last measured open positions "
                   "(required >= 0.900):" << std::endl;
      printPositionRow("right", right);
      printPositionRow("left", left);
    }
    else
    {
      std::cerr << "[InspireForce] No complete position feedback: right="
                << right_result << " left=" << left_result << std::endl;
    }
    fail("hands did not reach the verified open position within 20 seconds");
  }

  bool loadForceBaseline()
  {
    std::ifstream input(param::force_baseline_file);
    if (!input)
      return false;
    for (int i = 0; i < kTotalDof; ++i)
    {
      if (!(input >> force_offset_n(i)) || !std::isfinite(force_offset_n(i)) ||
          force_offset_n(i) < -9.8 || force_offset_n(i) > 9.8)
        return false;
    }
    baseline_valid_ = true;
    std::cout << "[InspireForce] Loaded unloaded baseline from "
              << param::force_baseline_file << std::endl;
    printForceRow("right", force_offset_n.block<kDofPerHand, 1>(0, 0));
    printForceRow(
        "left", force_offset_n.block<kDofPerHand, 1>(kDofPerHand, 0));
    return true;
  }

  void saveForceBaseline()
  {
    const std::filesystem::path path(param::force_baseline_file);
    std::error_code error;
    if (path.has_parent_path())
      std::filesystem::create_directories(path.parent_path(), error);
    if (error)
      fail("could not create baseline directory: " + error.message());

    const auto temporary = path.string() + ".tmp";
    std::ofstream output(temporary, std::ios::trunc);
    if (!output)
      fail("could not write force baseline: " + temporary);
    output << std::setprecision(17);
    for (int i = 0; i < kTotalDof; ++i)
      output << force_offset_n(i) << (i + 1 == kTotalDof ? '\n' : ' ');
    output.close();
    if (!output)
      fail("failed while writing force baseline: " + temporary);
    std::filesystem::rename(temporary, path, error);
    if (error)
      fail("could not install force baseline: " + error.message());
    std::cout << "[InspireForce] Saved unloaded baseline to " << path
              << std::endl;
  }

  void tareForceSensors()
  {
    std::array<std::vector<double>, kTotalDof> samples;
    bool stable = false;

    std::cout << "[InspireForce] Waiting " << kTareSettleSeconds
              << " seconds for force sensors to settle..." << std::endl;
    sleep(kTareSettleSeconds);
    std::cout << "[InspireForce] Measuring unloaded force baseline..."
              << std::endl;
    for (int attempt = 0; attempt < kTareMaxAttempts; ++attempt)
    {
      HandVector right_sample;
      HandVector left_sample;
      if (righthand->GetForce(right_sample) != 0 ||
          lefthand->GetForce(left_sample) != 0)
        continue;

      for (int i = 0; i < kDofPerHand; ++i)
      {
        samples[i].push_back(right_sample(i));
        samples[kDofPerHand + i].push_back(left_sample(i));
      }
      for (auto& window : samples)
      {
        if (window.size() > kTareSamples)
          window.erase(window.begin());
      }
      if (samples[0].size() < kTareSamples)
        continue;

      stable = true;
      for (const auto& window : samples)
      {
        const auto [minimum, maximum] =
            std::minmax_element(window.begin(), window.end());
        const double spread_g = (*maximum - *minimum) * 1000.0 / 9.8;
        if (spread_g > kTareMaxSpreadG)
        {
          stable = false;
          break;
        }
      }
      if (stable)
        break;
    }

    if (!stable)
      fail("force sensors did not produce a stable unloaded window; keep "
           "hands open and untouched, then retry");

    for (int i = 0; i < kTotalDof; ++i)
      force_offset_n(i) = median(samples[i]);
    baseline_valid_ = true;

    std::cout << "[InspireForce] Unloaded baseline captured:" << std::endl;
    printForceRow("right", force_offset_n.block<kDofPerHand, 1>(0, 0));
    printForceRow(
        "left", force_offset_n.block<kDofPerHand, 1>(kDofPerHand, 0));
  }

  void moveToProtectivePose()
  {
    std::cout << "[InspireForce] Returning both hands to the protective pose "
                 "at closing speed " << kDefaultClosingSpeed << "..."
              << std::endl;
    const HandVector speed = HandVector::Constant(kDefaultClosingSpeed);
    HandVector right_target = kProtectivePose;
    HandVector left_target = kProtectivePose;
    HandVector right_limit;
    HandVector left_limit;
    for (int i = 0; i < kDofPerHand; ++i)
    {
      right_limit(i) = std::clamp(
          kDefaultForceLimitG +
              std::max(0.0, force_offset_n(i) * 1000.0 / 9.8),
          1.0, 1000.0);
      left_limit(i) = std::clamp(
          kDefaultForceLimitG +
              std::max(
                  0.0,
                  force_offset_n(kDofPerHand + i) * 1000.0 / 9.8),
          1.0, 1000.0);
    }
    setVelocity(*righthand, speed);
    setVelocity(*lefthand, speed);
    setForce(*righthand, right_limit);
    setForce(*lefthand, left_limit);

    HandVector last_right_position = HandVector::Ones();
    HandVector last_left_position = HandVector::Ones();
    int preflight_stable_samples = 0;
    std::cout << "[InspireForce] Waiting for stable unloaded force feedback "
                 "before closing..." << std::endl;
    for (int attempt = 0; attempt < kProtectivePreflightMaxAttempts; ++attempt)
    {
      HandVector right_force;
      HandVector left_force;
      const bool feedback_valid =
          readPositionWithRetries(*righthand, last_right_position) == 0 &&
          readPositionWithRetries(*lefthand, last_left_position) == 0 &&
          righthand->GetForce(right_force) == 0 &&
          lefthand->GetForce(left_force) == 0;
      bool unloaded = feedback_valid;
      if (feedback_valid)
      {
        for (int i = 0; i < kDofPerHand; ++i)
        {
          const double right_net_g = std::abs(
              right_force(i) - force_offset_n(i)) * 1000.0 / 9.8;
          const double left_net_g = std::abs(
              left_force(i) - force_offset_n(kDofPerHand + i)) *
              1000.0 / 9.8;
          if (right_net_g > kProtectivePreflightMaxNetForceG ||
              left_net_g > kProtectivePreflightMaxNetForceG)
          {
            unloaded = false;
            break;
          }
        }
      }
      preflight_stable_samples = unloaded ? preflight_stable_samples + 1 : 0;
      if (preflight_stable_samples >= kProtectivePreflightStableSamples)
        break;
      usleep(100000);
    }
    if (preflight_stable_samples < kProtectivePreflightStableSamples)
      fail("force feedback did not stabilize before protective-pose motion");

    righthand->SetPosition(right_target);
    lefthand->SetPosition(left_target);

    int consecutive_feedback_failures = 0;
    bool closing_paused = false;
    std::array<int, kDofPerHand> right_contact_count{};
    std::array<int, kDofPerHand> left_contact_count{};
    int reached_stable_samples = 0;
    for (int attempt = 0; attempt < kProtectivePoseMaxAttempts; ++attempt)
    {
      HandVector right_position;
      HandVector left_position;
      HandVector right_force;
      HandVector left_force;
      const int right_position_result =
          readPositionWithRetries(*righthand, right_position);
      const int left_position_result =
          readPositionWithRetries(*lefthand, left_position);
      const int right_force_result = righthand->GetForce(right_force);
      const int left_force_result = lefthand->GetForce(left_force);
      if (right_position_result != 0 || left_position_result != 0 ||
          right_force_result != 0 || left_force_result != 0)
      {
        std::cerr << "[InspireForce] Protective feedback miss: "
                  << "right_pos=" << right_position_result
                  << " left_pos=" << left_position_result
                  << " right_force=" << right_force_result
                  << " left_force=" << left_force_result << std::endl;
        // Do not leave the full closing target active when feedback is lost.
        righthand->SetPosition(last_right_position);
        lefthand->SetPosition(last_left_position);
        closing_paused = true;
        if (++consecutive_feedback_failures >= kProtectiveMaxFeedbackFailures)
          fail("feedback was repeatedly lost while returning to the "
               "protective pose");
        usleep(100000);
        continue;
      }
      consecutive_feedback_failures = 0;
      last_right_position = right_position;
      last_left_position = left_position;
      if (closing_paused)
      {
        righthand->SetPosition(right_target);
        lefthand->SetPosition(left_target);
        closing_paused = false;
      }

      bool target_changed = false;
      bool any_force_over_limit = false;
      for (int i = 0; i < kDofPerHand; ++i)
      {
        const double right_net_g = std::max(
            0.0, right_force(i) - force_offset_n(i)) * 1000.0 / 9.8;
        const double left_net_g = std::max(
            0.0, left_force(i) - force_offset_n(kDofPerHand + i)) *
            1000.0 / 9.8;
        right_contact_count[i] = right_net_g >= kDefaultForceLimitG
            ? right_contact_count[i] + 1 : 0;
        left_contact_count[i] = left_net_g >= kDefaultForceLimitG
            ? left_contact_count[i] + 1 : 0;
        any_force_over_limit = any_force_over_limit ||
            right_net_g >= kDefaultForceLimitG ||
            left_net_g >= kDefaultForceLimitG;
        if (right_contact_count[i] >= kProtectiveContactConfirmSamples &&
            right_target(i) < right_position(i))
        {
          right_target(i) = std::min(1.0, right_position(i) + kBackoff);
          right_contact_count[i] = 0;
          target_changed = true;
        }
        if (left_contact_count[i] >= kProtectiveContactConfirmSamples &&
            left_target(i) < left_position(i))
        {
          left_target(i) = std::min(1.0, left_position(i) + kBackoff);
          left_contact_count[i] = 0;
          target_changed = true;
        }
      }
      if (target_changed)
      {
        righthand->SetPosition(right_target);
        lefthand->SetPosition(left_target);
      }

      const bool position_reached =
          (right_position - right_target).cwiseAbs().maxCoeff() <=
              kProtectivePoseTolerance &&
          (left_position - left_target).cwiseAbs().maxCoeff() <=
              kProtectivePoseTolerance;
      reached_stable_samples = position_reached && !any_force_over_limit
          ? reached_stable_samples + 1 : 0;
      if (reached_stable_samples >= kProtectiveReachedStableSamples)
      {
        std::cout << "[InspireForce] Protective pose reached and force "
                     "feedback remained stable." << std::endl;
        return;
      }
      usleep(100000);
    }
    fail("protective pose was not reached within 45 seconds");
  }

  void printDiagnosticRow(
      const char* side, int offset, const HandSafetyState& safety)
  {
    if (!safety.force_valid)
    {
      std::cout << "  " << side << "  unavailable" << std::endl;
      return;
    }
    const auto old_flags = std::cout.flags();
    const auto old_precision = std::cout.precision();
    std::cout << "  " << std::left << std::setw(5) << side << std::right;
    for (int i = 0; i < kDofPerHand; ++i)
    {
      const int index = offset + i;
      const double raw_g = raw_force_n(index) * 1000.0 / 9.8;
      const double baseline_g = baseline_valid_
          ? force_offset_n(index) * 1000.0 / 9.8 : 0.0;
      const double net_g = force_n(index) * 1000.0 / 9.8;
      std::cout << "  " << kActuatorNames[i] << "="
                << std::fixed << std::setprecision(0)
                << raw_g << "/" << baseline_g << "/" << net_g
                << (safety.contact_latched[i] ? "[C]" : "");
    }
    std::cout.flags(old_flags);
    std::cout.precision(old_precision);
    std::cout << std::endl;
  }

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
      raw_force_n.block<kDofPerHand, 1>(offset, 0) = sample;
      if (baseline_valid_)
      {
        for (int i = 0; i < kDofPerHand; ++i)
        {
          const int index = offset + i;
          const double net_n = std::max(0.0, sample(i) - force_offset_n(index));
          // Adapt only while physically open and far below contact. This
          // tracks slow thermal drift without learning an object as zero.
          if (qstate(index) >= kCalibrationOpenPosition &&
              !safety.contact_latched[i] &&
              net_n < kDefaultForceLimitG * 9.8 / 1000.0 * 0.2)
          {
            force_offset_n(index) +=
                kBaselineDriftAlpha * (sample(i) - force_offset_n(index));
          }
          const double corrected =
              std::max(0.0, sample(i) - force_offset_n(index));
          force_n(index) = kForceFilterAlpha * corrected +
              (1.0 - kForceFilterAlpha) * force_n(index);
        }
      }
      else
      {
        force_n.block<kDofPerHand, 1>(offset, 0) = sample.cwiseMax(0.0);
      }
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
        safety.above_limit_count[i] = 0;
        continue;
      }

      // Missing feedback must never authorize additional closure.
      if (!safety.position_valid || !safety.force_valid)
      {
        safe(i) = measured(i);
        continue;
      }

      const double limit_n = force_limit_g(i) * 9.8 / 1000.0;
      const bool severe = measured_force_n(i) >= 1.5 * limit_n;
      if (measured_force_n(i) >= limit_n)
        ++safety.above_limit_count[i];
      else
        safety.above_limit_count[i] = 0;

      if (!safety.contact_latched[i] &&
          (severe || safety.above_limit_count[i] >= kContactConfirmSamples))
      {
        safety.contact_latched[i] = true;
        // Hold the actuator exactly where contact was detected. Opening it as
        // an automatic backoff loosens multi-finger grasps as each actuator
        // reaches the threshold at a different time. The firmware force limit
        // remains active as the independent overshoot guard.
        safety.hold_position(i) = measured(i);
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
    HandVector device_force_limit_g;

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
      // FORCE_SET uses the sensor's absolute reading. Add the measured
      // no-load offset so its threshold matches our offset-corrected limit.
      device_force_limit_g(i) = std::clamp(
          force_limit_g(i) + std::max(
              0.0, force_offset_n(offset + i) * 1000.0 / 9.8),
          1.0, 1000.0);
    }

    // Program the firmware limits before sending the position target.
    if (!velocity.isApprox(safety.last_velocity, 0.5))
    {
      setVelocity(hand, velocity);
      safety.last_velocity = velocity;
    }
    if (!device_force_limit_g.isApprox(safety.last_force_limit_g, 0.5))
    {
      setForce(hand, device_force_limit_g);
      safety.last_force_limit_g = device_force_limit_g;
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
      std::cout << "[InspireForce] force[g] raw/base/net  sensors="
                << (right_.force_valid ? "R" : "-")
                << (left_.force_valid ? "L" : "-")
                << "  mode=" << (param::monitor_only ? "MONITOR" : "CONTROL")
                << std::endl;
      printDiagnosticRow("right", 0, right_);
      printDiagnosticRow("left", kDofPerHand, left_);
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
  Eigen::Matrix<double, kTotalDof, 1> raw_force_n;
  Eigen::Matrix<double, kTotalDof, 1> force_n;
  Eigen::Matrix<double, kTotalDof, 1> force_offset_n =
      Eigen::Matrix<double, kTotalDof, 1>::Zero();
  std::unique_ptr<unitree::robot::RealTimePublisher<
      unitree_go::msg::dds_::MotorStates_>> handstate;
  std::shared_ptr<unitree::robot::SubscriptionBase<
      unitree_go::msg::dds_::MotorCmds_>> handcmd;

private:
  HandSafetyState right_;
  HandSafetyState left_;
  bool baseline_valid_ = false;
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
