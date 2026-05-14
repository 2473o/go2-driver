#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

namespace
{

struct ResetConfig
{
  std::string network_interface;
  std::string service_name;
  std::string target_mode;
  int64_t restart_delay_ms;
  double timeout_sec;
};

void print_usage()
{
  std::cout
    << "Usage:\n"
    << "  ros2 run go2_driver mcf_reset --ros-args "
    << "-p network_interface:=eth0 -p service_name:=mcf "
    << "-p target_mode:=classic "
    << "-p restart_delay_ms:=1000 -p timeout_sec:=10.0\n\n"
    << "This executable is a Unitree SDK tool packaged in a ROS package. "
    << "It accepts ROS-style -p key:=value arguments, but does not create a ROS node.\n";
}

void log_info(const std::string & message)
{
  std::cout << "[INFO] [mcf_reset]: " << message << std::endl;
}

void log_warn(const std::string & message)
{
  std::cerr << "[WARN] [mcf_reset]: " << message << std::endl;
}

void log_error(const std::string & message)
{
  std::cerr << "[ERROR] [mcf_reset]: " << message << std::endl;
}

template<typename T>
std::string to_string(T value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

class ChannelFactoryGuard
{
public:
  ChannelFactoryGuard(int32_t domain_id, const std::string & network_interface)
  {
    if (network_interface.empty()) {
      unitree::robot::ChannelFactory::Instance()->Init(domain_id);
    } else {
      unitree::robot::ChannelFactory::Instance()->Init(domain_id, network_interface);
    }
    initialized_ = true;
  }

  ~ChannelFactoryGuard()
  {
    if (initialized_) {
      unitree::robot::ChannelFactory::Instance()->Release();
    }
  }

  ChannelFactoryGuard(const ChannelFactoryGuard &) = delete;
  ChannelFactoryGuard & operator=(const ChannelFactoryGuard &) = delete;

private:
  bool initialized_{false};
};

bool parse_param_assignment(const std::string & assignment, ResetConfig & config)
{
  const std::size_t separator = assignment.find(":=");
  if (separator == std::string::npos) {
    return false;
  }

  const std::string name = assignment.substr(0, separator);
  const std::string value = assignment.substr(separator + 2);

  try {
    if (name == "network_interface") {
      config.network_interface = value;
    } else if (name == "service_name") {
      config.service_name = value;
    } else if (name == "target_mode" || name == "mode_after_reset" || name == "motion_mode") {
      config.target_mode = value;
    } else if (name == "restart_delay_ms") {
      config.restart_delay_ms = std::stoll(value);
    } else if (name == "timeout_sec") {
      config.timeout_sec = std::stod(value);
    } else {
      log_warn("Ignoring unknown parameter '" + name + "'");
    }
  } catch (const std::exception & err) {
    log_error("Invalid value for parameter '" + name + "': " + value + " (" + err.what() + ")");
    return false;
  }

  return true;
}

int load_config(int argc, char ** argv, ResetConfig & config)
{
  config.network_interface = "eth0";
  config.service_name = "mcf";
  config.target_mode = "classic";
  config.restart_delay_ms = 1000;
  config.timeout_sec = 10.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];

    if (arg == "-h" || arg == "--help") {
      print_usage();
      return 1;
    }

    if (arg == "--ros-args" || arg == "--") {
      continue;
    }

    if (arg == "-p" || arg == "--param") {
      if (i + 1 >= argc) {
        log_error("Missing value after " + arg);
        return -1;
      }
      if (!parse_param_assignment(argv[++i], config)) {
        return -1;
      }
      continue;
    }

    if (arg.find(":=") != std::string::npos) {
      if (!parse_param_assignment(arg, config)) {
        return -1;
      }
      continue;
    }

    log_warn("Ignoring argument '" + arg + "'");
  }

  config.restart_delay_ms = std::max<int64_t>(0, config.restart_delay_ms);
  config.timeout_sec = std::max(0.1, config.timeout_sec);

  return 0;
}

bool check_motion_mode(
  unitree::robot::b2::MotionSwitcherClient & motion_switcher,
  const std::string & label)
{
  std::string form;
  std::string name;
  const int32_t ret = motion_switcher.CheckMode(form, name);
  if (ret != 0) {
    log_warn(label + " CheckMode failed, ret=" + to_string(ret));
    return false;
  }

  if (name.empty()) {
    log_info(label + " motion mode is inactive");
  } else if (form.empty()) {
    log_info(label + " motion mode: name=" + name);
  } else {
    log_info(label + " motion mode: form=" + form + ", name=" + name);
  }

  return true;
}

bool select_motion_mode(
  unitree::robot::b2::MotionSwitcherClient & motion_switcher,
  const std::string & target_mode)
{
  if (target_mode.empty()) {
    log_info("Skipping motion mode selection because target_mode is empty");
    return true;
  }

  const int32_t ret = motion_switcher.SelectMode(target_mode);
  if (ret != 0) {
    log_error("SelectMode(" + target_mode + ") failed, ret=" + to_string(ret));
    return false;
  }

  log_info("SelectMode(" + target_mode + ") succeeded");
  return true;
}

void log_service_list(
  unitree::robot::go2::RobotStateClient & robot_state)
{
  std::vector<unitree::robot::go2::ServiceState> services;
  const int32_t ret = robot_state.ServiceList(services);
  if (ret != 0) {
    log_warn("ServiceList failed, ret=" + to_string(ret));
    return;
  }

  if (services.empty()) {
    log_warn("ServiceList returned no services");
    return;
  }

  log_info("Available services:");
  for (const auto & service : services) {
    log_info(
      "  name=" + service.name +
      ", status=" + to_string(service.status) +
      ", protect=" + to_string(service.protect));
  }
}

bool switch_service(
  unitree::robot::go2::RobotStateClient & robot_state,
  const std::string & service_name,
  bool enable)
{
  int32_t status = 0;
  const int32_t swit = enable ? 1 : 0;
  const int32_t ret = robot_state.ServiceSwitch(service_name, swit, status);
  if (ret != 0) {
    log_error(
      "ServiceSwitch(" + service_name + ", " + to_string(swit) +
      ") failed, ret=" + to_string(ret) +
      ", status=" + to_string(status));
    log_service_list(robot_state);
    return false;
  }

  log_info(
    "ServiceSwitch(" + service_name + ", " + to_string(swit) +
    ") succeeded, status=" + to_string(status));
  return true;
}

int run_reset(const ResetConfig & config)
{
  if (config.service_name.empty()) {
    log_error("Parameter service_name must not be empty");
    return 2;
  }

  log_warn(
    "Restarting Unitree motion service '" + config.service_name +
    "'. Keep the robot supported and clear of people.");
  log_info(
    "Using network_interface='" + config.network_interface +
    "', target_mode='" + config.target_mode +
    "', restart_delay_ms=" + to_string(config.restart_delay_ms) +
    ", timeout_sec=" + to_string(config.timeout_sec));

  ChannelFactoryGuard channel_guard(0, config.network_interface);

  unitree::robot::b2::MotionSwitcherClient motion_switcher;
  motion_switcher.SetTimeout(static_cast<float>(config.timeout_sec));
  motion_switcher.Init();

  unitree::robot::go2::RobotStateClient robot_state;
  robot_state.SetTimeout(static_cast<float>(config.timeout_sec));
  robot_state.Init();

  check_motion_mode(motion_switcher, "Before reset:");

  if (!switch_service(robot_state, config.service_name, false)) {
    return 3;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(config.restart_delay_ms));

  if (!switch_service(robot_state, config.service_name, true)) {
    return 4;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(config.restart_delay_ms));
  if (!select_motion_mode(motion_switcher, config.target_mode)) {
    return 5;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(config.restart_delay_ms));
  check_motion_mode(motion_switcher, "After reset:");

  log_info("Motion service reset finished");
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  int exit_code = 1;
  try {
    ResetConfig config;
    const int config_status = load_config(argc, argv, config);
    if (config_status > 0) {
      return 0;
    }
    if (config_status < 0) {
      return 2;
    }

    exit_code = run_reset(config);
  } catch (const std::exception & err) {
    log_error(std::string("Reset failed: ") + err.what());
    exit_code = 1;
  }

  return exit_code;
}
