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

#include <numeric>
#include <iostream>
#include <chrono>
#include <string>

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

namespace ethercat_generic_plugins
{

EcCiA402Drive::EcCiA402Drive()
: GenericEcSlave() {}
EcCiA402Drive::~EcCiA402Drive() {}

bool EcCiA402Drive::initialized() {
  return initialized_ && (homing_complete_ || !auto_homing_);
}

void EcCiA402Drive::processData(size_t index, uint8_t * domain_address)
{
  // Special case: ControlWord
  if (pdo_channels_info_[index].index == CiA402D_RPDO_CONTROLWORD) {
    if (is_operational_) {
      if (fault_reset_command_interface_index_ >= 0) {
        if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0) {
          last_fault_reset_command_ = false;
        }
        if (last_fault_reset_command_ == false &&
          command_interface_ptr_->at(fault_reset_command_interface_index_) != 0 &&
          !std::isnan(command_interface_ptr_->at(fault_reset_command_interface_index_)))
        {
          last_fault_reset_command_ = true;
          fault_reset_ = true;
        }
      }

      if (auto_state_transitions_) {
        pdo_channels_info_[index].default_value = transition(
          state_,
          pdo_channels_info_[index].ec_read(domain_address));
      }
      auto& control_word_pdo =  pdo_channels_info_[index];
      auto& control_word = control_word_pdo.default_value;
      if (auto_homing_ && !homing_complete_) {
        //TODOget homing method
        //!if homing method is 0,35,37 then we skip all this

        //wait to get to state_operation_enabled
        if (state_ == STATE_OPERATION_ENABLED) {
          //change mode of operation to homing
          if (mode_of_operation_ != ModeOfOperation::MODE_HOMING) {
            prev_mode_of_operation_ = mode_of_operation_;
            mode_of_operation_ = ModeOfOperation::MODE_HOMING;        
          }
          
          //cwait for mode_of_operation_display has changed
          if (mode_of_operation_display_ == ModeOfOperation::MODE_HOMING) {
            std::cout<<"moo:" << std::to_string(mode_of_operation_display_) << " cw: " << std::to_string(control_word) << std::endl;
            //set control word to start homing
            if (!homing_started_)
            {
              std::cout<< "Homing started" << std::endl;
              control_word = 0x1F; 
              homing_started_ = true;
              homing_start_time_ = std::chrono::steady_clock::now();
            }
            else if (homing_started_) //explicit for clarity
            {
              //check if homing is complete
              //mointor status word wiht timeout 
              int homingStatus = checkHomingStatus(status_word_);
              bool timeout = std::chrono::steady_clock::now() - homing_start_time_ > homing_timeout_;
              if (homingStatus == 1)
              {
                std::cout<< "Homing complete" << std::endl;
                // set contro word back to operation enabled
                control_word = 0x0F;
                homing_complete_ = true;
                //set mode of operation back to previous mode
                mode_of_operation_ = prev_mode_of_operation_;
                
              }

              if (homingStatus == -1 ||  timeout)
              {
                std::cout<< "Homing error" << std::endl;
                if (timeout)
                {
                  std::cout<< "Homing timeout" << std::endl;
                }
                control_word = 0x0F;
                homing_complete_ = true; //umm??? //TODO figure out mitigation here
                //set mode of operation back to previous mode
                mode_of_operation_ = prev_mode_of_operation_;
              }

            }
          }
          //then have error handling somehow for halting??
        }

      }
    }
  }

  // setup current position as default position
  if (pdo_channels_info_[index].index == CiA402D_RPDO_POSITION) {
    if (mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE) {

      auto& pdo_info = pdo_channels_info_[index];
      auto interface_index = pdo_info.interface_index;
      double new_default = pdo_info.factor * last_position_ + pdo_info.offset;

      if (interface_index >= 0 ){
        bool command_interface_nan = std::isnan(command_interface_ptr_->at(pdo_info.interface_index));
        
        if (!command_interface_nan || !default_position_updated_)
        {
          pdo_info.default_value = new_default;

          if (command_interface_nan) 
          {
            //only update the default position once
            default_position_updated_ = true;
            std::cout<< "default position updated" << std::endl;
          }
          else 
          {
            // in the case command interface is not nan we reset 
            // the default position updated flag so that if command
            // interface becomes nan agian, we can still do the first update of default position
            default_position_updated_ = false;
          }

        }
      }
      else //fall through case to original behaviour, idk the proper error handlign yet
      {
        pdo_info.default_value = new_default;
      }

      // pdo_channels_info_[index].default_value =
      //   pdo_channels_info_[index].factor * last_position_ +
      //   pdo_channels_info_[index].offset;
    }
    pdo_channels_info_[index].override_command =
      (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION) ? true : false;
  }

  // setup mode of operation
  if (pdo_channels_info_[index].index == CiA402D_RPDO_MODE_OF_OPERATION) {
    if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10) {
      pdo_channels_info_[index].default_value = mode_of_operation_;
    }
  }

  pdo_channels_info_[index].ec_update(domain_address);

  // get mode_of_operation_display_
  if (pdo_channels_info_[index].index == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY) {
    mode_of_operation_display_ = pdo_channels_info_[index].last_value;
  }

  if (pdo_channels_info_[index].index == CiA402D_TPDO_POSITION) {
    last_position_ = pdo_channels_info_[index].last_value;
  }

  // Special case: StatusWord
  if (pdo_channels_info_[index].index == CiA402D_TPDO_STATUSWORD) {
    status_word_ = pdo_channels_info_[index].last_value;
  }


  // CHECK FOR STATE CHANGE
  if (index == all_channels_.size() - 1) {  // if last entry  in domain
    if (status_word_ != last_status_word_) {
      state_ = deviceState(status_word_);
      if (state_ != last_state_) {
        std::cout << "STATE: " << DEVICE_STATE_STR.at(state_)
                  << " with status word :" << status_word_ << std::endl;
      }
    }
    initialized_ = ((state_ == STATE_OPERATION_ENABLED) &&
      (last_state_ == STATE_OPERATION_ENABLED)) ? true : false;

    last_status_word_ = status_word_;
    last_state_ = state_;
    counter_++;
  }
}

bool EcCiA402Drive::setupSlave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  if (paramters_.find("slave_config") != paramters_.end()) {
    if (!setup_from_config_file(paramters_["slave_config"])) {
      return false;
    }
  } else {
    std::cerr << "EcCiA402Drive: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  setup_interface_mapping();
  setup_syncs();

  if (paramters_.find("mode_of_operation") != paramters_.end()) {
    mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);
  }

  if (paramters_.find("command_interface/reset_fault") != paramters_.end()) {
    fault_reset_command_interface_index_ = std::stoi(paramters_["command_interface/reset_fault"]);
  }

  return true;
}

bool EcCiA402Drive::setup_from_config(YAML::Node drive_config)
{
  if (!GenericEcSlave::setup_from_config(drive_config)) {return false;}
  // additional configuration parameters for CiA402 Drives
  if (drive_config["auto_fault_reset"]) {
    auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
  }
  if (drive_config["auto_state_transitions"]) {
    auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
  }
  if (drive_config["auto_homing"]) {
    auto_homing_ = drive_config["auto_homing"].as<bool>();
  }
  if (drive_config["homing_timeout"]) { //expect timeout in seconds
    homing_timeout_ = std::chrono::milliseconds(
        static_cast<int64_t>(
            drive_config["homing_timeout"].as<double>() * 1000
        )
    );
  }
  return true;
}

bool EcCiA402Drive::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try {
    slave_config_ = YAML::LoadFile(config_file);
  } catch (const YAML::ParserException & ex) {
    std::cerr << "EcCiA402Drive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  } catch (const YAML::BadFile & ex) {
    std::cerr << "EcCiA402Drive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  if (!setup_from_config(slave_config_)) {
    return false;
  }
  return true;
}

/** returns device state based upon the status_word */
DeviceState EcCiA402Drive::deviceState(uint16_t status_word)
{
  if ((status_word & 0b01001111) == 0b00000000) {
    return STATE_NOT_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return STATE_SWITCH_ON_DISABLED;
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return STATE_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return STATE_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return STATE_OPERATION_ENABLED;
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return STATE_QUICK_STOP_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return STATE_FAULT_REACTION_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return STATE_FAULT;
  }
  return STATE_UNDEFINED;
}

/** returns the control word that will take device from state to next desired state */
uint16_t EcCiA402Drive::transition(DeviceState state, uint16_t control_word)
{
  switch (state) {
    case STATE_START:                     // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:    // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:        // -> STATE_READY_TO_SWITCH_ON
      return (control_word & 0b01111110) | 0b00000110;
    case STATE_READY_TO_SWITCH_ON:        // -> STATE_SWITCH_ON
      return (control_word & 0b01110111) | 0b00000111;
    case STATE_SWITCH_ON:                 // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_OPERATION_ENABLED:         // -> GOOD
      return control_word;
    case STATE_QUICK_STOP_ACTIVE:         // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_FAULT_REACTION_ACTIVE:     // -> STATE_FAULT (automatic)
      return control_word;
    case STATE_FAULT:                     // -> STATE_SWITCH_ON_DISABLED
      if (auto_fault_reset_ || fault_reset_) {
        fault_reset_ = false;
        return (control_word & 0b11111111) | 0b10000000;     // automatic reset
      } else {
        return control_word;
      }
    default:
      break;
  }
  return control_word;
}

int EcCiA402Drive::checkHomingStatus(uint16_t status_word)
{
  uint16_t homing_state = status_word & static_cast<uint16_t>(HomingState::HOMING_MASK);
  //TODO use proper logging here
  std::cout<< "Homing state: " << std::to_string(homing_state) << std::endl;
  switch (homing_state) 
  {
    case static_cast<uint16_t>(HomingState::HOMING_IN_PROGRESS): 
    {
      break;
    }
    case static_cast<uint16_t>(HomingState::HOMING_NOT_STARTED): 
    {
      std::cout << "Homing not started" << std::endl;
      break;
    }
    case static_cast<uint16_t>(HomingState::HOMING_ATTAINED): 
    {
      std::cout << "Homing attained" << std::endl;
      std::cout << "Homing complete" << std::endl;
      return true;
    }
    case static_cast<uint16_t>(HomingState::HOMING_COMPLETE): 
    {
      std::cout << "Homing complete" << std::endl;
      return true;
    }
    case static_cast<uint16_t>(HomingState::HOMING_ERROR_MOTOR_MOVING): 
    {
      std::cout << "Homing error: motor moving" << std::endl;
      return 0;
    }
    case static_cast<uint16_t>(HomingState::HOMING_ERROR): 
    {
      std::cout << "Homing error: motor still" << std::endl;
      return 0;
    }
    default: 
    {
      std::cout << "Homing state not defined (masked status: 0x" 
                << std::hex << homing_state << std::dec << ")" 
                << std::endl;
      return -1;
    }
  }
  return false;
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcCiA402Drive, ethercat_interface::EcSlave)
