#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <iostream>
#include <chrono>
#include <spdlog/spdlog.h>
#include <boost/program_options.hpp>

namespace param
{

namespace po = boost::program_options;

inline std::string serial_port;
inline std::string network; 
inline std::string ns; 
inline float threhold;
inline bool calibrate_force = false;
inline bool monitor_only = false;
inline bool open_before_calibration = false;
inline bool return_to_protective_pose = false;
inline std::string force_baseline_file;

po::variables_map helper(int argc, char** argv)
{
#ifndef NDEBUG
  spdlog::set_level(spdlog::level::debug);
#else
  spdlog::set_level(spdlog::level::info);
#endif


  po::options_description desc("Unitree H1 Inspire Hand Serial to DDS");
  desc.add_options()
    ("help,h", "produce help message")
    ("serial,s", po::value<std::string>(&serial_port)->default_value("/dev/ttyUSB0"), "serial port")
    ("network", po::value<std::string>(&network)->default_value(""), "DDS network interface")
    ("namespace", po::value<std::string>(&ns)->default_value("inspire"), "DDS topic namespace")
    ("calibrate-force", po::bool_switch(&calibrate_force),
      "calibrate both force sensors before starting (hands must be unloaded)")
    ("monitor-only", po::bool_switch(&monitor_only),
      "publish position/force state but ignore all DDS commands")
    ("open-before-calibration", po::bool_switch(&open_before_calibration),
      "slowly open both hands before force calibration")
    ("return-to-protective-pose", po::bool_switch(&return_to_protective_pose),
      "slowly return both hands to the configured protective pose afterward")
    ("force-baseline-file", po::value<std::string>(&force_baseline_file)->
      default_value("/var/lib/dfx_inspire_service/force_baseline.txt"),
      "path used to save/load the unloaded force baseline")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    exit(0);
  }

  if(ns.empty())
  {
    spdlog::error("Namespace cannot be empty");
    exit(1);
  }

  if (calibrate_force && !monitor_only)
  {
    spdlog::error("--calibrate-force requires --monitor-only so calibration "
                  "cannot be followed by motion commands");
    exit(1);
  }

  if (open_before_calibration && !calibrate_force)
  {
    spdlog::error("--open-before-calibration requires --calibrate-force");
    exit(1);
  }

  if (return_to_protective_pose && !calibrate_force)
  {
    spdlog::error("--return-to-protective-pose requires --calibrate-force");
    exit(1);
  }

  return vm;
}

}

#endif // PARAM_H
