
/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus/vehicle/ls/ls_controller.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/ls/ls_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/proto/vehicle_signal.pb.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h" //controller主要是控制,数据是下发所以只需要包含sender.
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ls {
using ::apollo::common::ErrorCode;
//ErrorCode在不同namespace下都有定义,此处的定义在common下error_code.proto
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {
const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;

}  // namespace

ErrorCode LsController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "LsController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;//common下error_code.proto的文件中,CANBUS_ERROR的代号是2000;
    /*范围解析运算符 :: 
    语法  ::identifier
class-name :: identifier
namespace :: identifier
enum class :: identifier
enum struct :: identifier
备注: identifier 可以是变量、函数或枚举值。*/
  }

  vehicle_params_.CopyFrom(
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());
  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  brake_command_102_ = dynamic_cast<Brakecommand102*>(
      message_manager_->GetMutableProtocolDataById(Brakecommand102::ID));
  if (brake_command_102_ == nullptr) {
    AERROR << "Brakecommand102 does not exist in the LsMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  gear_command_104_ = dynamic_cast<Gearcommand104*>(
      message_manager_->GetMutableProtocolDataById(Gearcommand104::ID));
  if (gear_command_104_ == nullptr) {
    AERROR << "Gearcommand104 does not exist in the ChMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  steer_command_103_ = dynamic_cast<Steercommand103*>(
      message_manager_->GetMutableProtocolDataById(Steercommand103::ID));
  if (steer_command_103_ == nullptr) {
    AERROR << "Steercommand103 does not exist in the ChMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  throttle_command_101_ = dynamic_cast<Throttlecommand101*>(
      message_manager_->GetMutableProtocolDataById(Throttlecommand101::ID));
  if (throttle_command_101_ == nullptr) {
    AERROR << "Throttlecommand101 does not exist in the ChMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  turnsignal_command_104_ = dynamic_cast<Turnsignalcommand104*>(
      message_manager_->GetMutableProtocolDataById(Turnsignalcommand104::ID));
  if (turnsignal_command_104_ == nullptr) {
    AERROR << "Turnsignalcommand104 does not exist in the ChMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Brakecommand102::ID, brake_command_102_, false);
  can_sender_->AddMessage(Gearcommand104::ID, gear_command_104_, false);
  can_sender_->AddMessage(Steercommand103::ID, steer_command_103_, false);
  can_sender_->AddMessage(Throttlecommand101::ID, throttle_command_101_, false);
  can_sender_->AddMessage(Turnsignalcommand104::ID, turnsignal_command_104_,
                          false);

  // need sleep to ensure all messages received
  AINFO << "LsController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

LsController::~LsController() {}

bool LsController::Start() {
  if (!is_initialized_) {
    AERROR << "LsController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void LsController::Stop() {
  if (!is_initialized_) {
    AERROR << "LsController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "LsController stopped.";
  }
}

Chassis LsController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);
  // 4 engine rpm ch has no engine rpm
  // chassis_.set_engine_rpm(0);
  // 5 ch has no wheel spd.
  if (chassis_detail.ls().has_ecu_status_200() &&
      chassis_detail.ls().ecu_status_200().has_speed()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.ls().ecu_status_200().speed()));
  } else {
    chassis_.set_speed_mps(0);
  }

  // 6 ch has no odometer
  // chassis_.set_odometer_m(0);
  // 7 ch has no fuel. do not set;
  // chassis_.set_fuel_range_m(0);
  // 8 throttle
  if (chassis_detail.ls().has_throttle_status__201() &&
      chassis_detail.ls().throttle_status__201().has_throttle_pedal_sts()) {
    chassis_.set_throttle_percentage(static_cast<float>(
        chassis_detail.ls().throttle_status__201().throttle_pedal_sts()));
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 9 brake
  if (chassis_detail.ls().has_brake_status__202() &&
      chassis_detail.ls().brake_status__202().has_brake_pedal_sts()) {
    chassis_.set_brake_percentage(static_cast<float>(
        chassis_detail.ls().brake_status__202().brake_pedal_sts()));
  } else {
    chassis_.set_brake_percentage(0);
  }

  // 23, previously 10   gear
  if (chassis_detail.ls().has_gear_status_204() &&
      chassis_detail.ls().gear_status_204().has_gear_sts()) {
    Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

    if (chassis_detail.ls().gear_status_204().gear_sts() ==
        gear_status_204::GEAR_STS_NEUTRAL) {
      gear_pos = Chassis::GEAR_NEUTRAL;
    }
    if (chassis_detail.ls().gear_status_204().gear_sts() ==
        gear_status_204::GEAR_STS_REVERSE) {
      gear_pos = Chassis::GEAR_REVERSE;
    }
    if (chassis_detail.ls().gear_status_204().gear_sts() ==
        gear_status_204::GEAR_STS_DRIVE) {
      gear_pos = Chassis::GEAR_DRIVE;
    }
    if (chassis_detail.ls().gear_status_204().gear_sts() ==
        gear_status_204::GEAR_STS_PARK) {
      gear_pos = Chassis::GEAR_PARKING;
    }

    chassis_.set_gear_location(gear_pos);
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }
  // 11 steering
  if (chassis_detail.ls().has_steer_status__203() &&
      chassis_detail.ls().steer_status__203().has_steer_angle_sts()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.ls().steer_status__203().steer_angle_sts() * 100.0 /
        vehicle_params_.max_steer_angle()));
  } else {
    chassis_.set_steering_percentage(0);
  }

  // 26
  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  if (chassis_detail.has_surround()) {
    chassis_.mutable_surround()->CopyFrom(chassis_detail.surround());
  }
  // give engage_advice based on error_code and canbus feedback
  if (!chassis_error_mask_ && !chassis_.parking_brake() &&
      (chassis_.throttle_percentage() == 0.0)) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::READY_TO_ENGAGE);
  } else {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "CANBUS not ready, firmware error or emergency button pressed!");
  }

  return chassis_;
}

void LsController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode LsController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }

  brake_command_102_->set_brake_pedal_en_ctrl(
      Brake_command_102::BRAKE_PEDAL_EN_CTRL_ENABLE);
  throttle_command_101_->set_throttle_pedal_en_ctrl(
      Throttle_command_101::THROTTLE_PEDAL_EN_CTRL_ENABLE);
  steer_command_103_->set_steer_angle_en_ctrl(
      Steer_command_103::STEER_ANGLE_EN_CTRL_ENABLE);
  AINFO << "\n\n\n set enable \n\n\n";
  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
    AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode LsController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL OK!";
  return ErrorCode::OK;
}

ErrorCode LsController::EnableSteeringOnlyMode() {
  AFATAL << "SteeringOnlyMode Not supported!";
  return ErrorCode::OK;
}

ErrorCode LsController::EnableSpeedOnlyMode() {
  AFATAL << "SpeedOnlyMode Not supported!";
  return ErrorCode::OK;
}

// NEUTRAL, REVERSE, DRIVE
void LsController::Gear(Chassis::GearPosition gear_position) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "this drive mode no need to set gear.";
    return;
  }

  // ADD YOUR OWN CAR CHASSIS OPERATION
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      gear_command_104_->set_gear_cmd(Gear_command_114::GEAR_CMD_NEUTRAL);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      gear_command_104_->set_gear_cmd(Gear_command_114::GEAR_CMD_REVERSE);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      gear_command_104_->set_gear_cmd(Gear_command_114::GEAR_CMD_DRIVE);
      break;
    }
    case Chassis::GEAR_PARKING: {
      gear_command_104_->set_gear_cmd(Gear_command_114::GEAR_CMD_PARK);
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      gear_command_104_->set_gear_cmd(Gear_command_114::GEAR_CMD_NEUTRAL);
      break;
    }
    default: {
      gear_command_104_->set_gear_cmd(Gear_command_114::GEAR_CMD_NEUTRAL);
      break;
    }
  }
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
void LsController::Brake(double pedal) {
  // Update brake value based on mode
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // ADD YOUR OWN CAR CHASSIS OPERATION
  brake_command_102_->set_brake_pedal_cmd(static_cast<int>(pedal));
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void LsController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // ADD YOUR OWN CAR CHASSIS OPERATION
  throttle_command_101_->set_throttle_pedal_cmd(static_cast<int>(pedal));
}

void LsController::Acceleration(double acc) {}

// ch default, -23 ~ 23, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void LsController::Steer(double angle) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = vehicle_params_.max_steer_angle() * angle / 100.0;
  // reverse sign
  // ADD YOUR OWN CAR CHASSIS OPERATION
  steer_command_103_->set_steer_angle_cmd(real_angle);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void LsController::Steer(double angle, double angle_spd) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  // ADD YOUR OWN CAR CHASSIS OPERATION
  const double real_angle = vehicle_params_.max_steer_angle() * angle / 100.0;
  steer_command_103_->set_steer_angle_cmd(real_angle);
}

void LsController::SetEpbBreak(const ControlCommand& command) {}

void LsController::SetBeam(const ControlCommand& command) {}

void LsController::SetHorn(const ControlCommand& command) {}

void LsController::SetTurningSignal(const ControlCommand& command) {}

void LsController::ResetProtocol() { message_manager_->ResetSendMessages(); }

bool LsController::CheckChassisError() { return false; }

void LsController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Fail to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false) == false) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR << "Too much time consumption in LsController looping process:"
             << elapsed.count();
    }
  }
}
bool LsController::CheckResponse(const int32_t flags, bool need_wait) {
  int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_eps_online = false;
  bool is_vcu_online = false;
  bool is_esp_online = false;

  do {
    if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK) {
      AERROR_EVERY(100) << "get chassis detail failed.";
      return false;
    }
    bool check_ok = true;
    if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG) {
      is_eps_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_eps_online() &&
                      chassis_detail.check_response().is_eps_online();
      check_ok = check_ok && is_eps_online;
    }

    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      is_vcu_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_vcu_online() &&
                      chassis_detail.check_response().is_vcu_online();
      is_esp_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_esp_online() &&
                      chassis_detail.check_response().is_esp_online();
      check_ok = check_ok && is_vcu_online && is_esp_online;
    }
    if (check_ok) {
      return true;
    } else {
      AINFO << "Need to check response again.";
    }
    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(20));
    }
  } while (need_wait && retry_num);

  AINFO << "check_response fail: is_eps_online:" << is_eps_online
        << ", is_vcu_online:" << is_vcu_online
        << ", is_esp_online:" << is_esp_online;

  return false;
}

void LsController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t LsController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode LsController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void LsController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace ls
}  // namespace canbus
}  // namespace apollo
