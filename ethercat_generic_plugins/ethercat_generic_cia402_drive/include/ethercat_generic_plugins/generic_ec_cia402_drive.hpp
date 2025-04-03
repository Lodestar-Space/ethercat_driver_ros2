// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#ifndef ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
#define ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_

#include <vector>
#include <string>
#include <unordered_map>
#include <limits>
#include <chrono>

#include "yaml-cpp/yaml.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_pdo_channel_manager.hpp"
#include "ethercat_generic_plugins/generic_ec_slave.hpp"
#include "ethercat_generic_plugins/cia402_common_defs.hpp"

namespace ethercat_generic_plugins
{

class EcCiA402Drive : public GenericEcSlave
{
public:
  EcCiA402Drive();
  virtual ~EcCiA402Drive();
  
  /**
   * @brief Returns true if drive is initalized and homing is complete if enabled.
   * 
   * @return true 
   * @return false 
   */
  bool initialized() override;

  /**
   * @brief Process data from the drive.
   * 
   * @param index 
   * @param domain_address 
   */
  virtual void processData(size_t index, uint8_t * domain_address);

  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface);


protected:
  int8_t mode_of_operation_display_ = 0;
  int8_t mode_of_operation_ = -1;

  uint32_t counter_ = 0;
  uint16_t last_status_word_ = -1;

  uint16_t status_word_ = 0;
  uint16_t control_word_ = 0;

  DeviceState last_state_ = STATE_START;
  DeviceState state_ = STATE_START;

  bool initialized_ = false;

  bool auto_fault_reset_ = false;
  bool auto_state_transitions_ = true;
  bool fault_reset_ = false;
  int fault_reset_command_interface_index_ = -1;
  bool last_fault_reset_command_ = false;

  double last_position_ = std::numeric_limits<double>::quiet_NaN();

  bool default_position_updated_ = false;
  
  // Homing parameters
  std::chrono::steady_clock::time_point homing_start_time_;
  std::chrono::milliseconds homing_timeout_ = std::chrono::milliseconds(30000); //default 30 seconds
  bool homing_started_ = false;
  bool homing_complete_ = false;
  bool auto_homing_ = false;
  int8_t prev_mode_of_operation_ = -1;
  int homing_method_ = -1;
  /**
   * @brief returns 1 if homing complete, 0 if in progress, -1 on error
   * 
   * @param status_word 
   * @return true 
   * @return false 
   */
  int checkHomingStatus(uint16_t status_word);

  enum class HomingState:uint16_t
  {
    HOMING_IN_PROGRESS = 0,
    HOMING_NOT_STARTED = (1<<10),
    HOMING_ATTAINED = (1 << 12),
    HOMING_COMPLETE = HOMING_ATTAINED | HOMING_NOT_STARTED, // from somanet docs??
    HOMING_ERROR_MOTOR_MOVING = (1 << 13),
    HOMING_ERROR = (1 << 13)| (1 << 10),
    HOMING_MASK = HOMING_NOT_STARTED | HOMING_ATTAINED | HOMING_ERROR,
  };

  /** returns device state based upon the status_word */
  DeviceState deviceState(uint16_t status_word);
  /** returns the control word that will take device from state to next desired state */
  uint16_t transition(DeviceState state, uint16_t control_word);
  /** set up of the drive configuration from yaml node*/
  bool setup_from_config(YAML::Node drive_config);
  /** set up of the drive configuration from yaml file*/
  bool setup_from_config_file(std::string config_file);
};
}  // namespace ethercat_generic_plugins

#endif  // ETHERCAT_GENERIC_PLUGINS__GENERIC_EC_CIA402_DRIVE_HPP_
