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

#include "modules/canbus/vehicle/ls/ls_message_manager.h"
#include "modules/canbus/vehicle/ls/protocol/brake_command_102.h"
#include "modules/canbus/vehicle/ls/protocol/brake_status__202.h"
#include "modules/canbus/vehicle/ls/protocol/control_command_100.h"
#include "modules/canbus/vehicle/ls/protocol/ecu_status__200.h"
#include "modules/canbus/vehicle/ls/protocol/speed_feedback.h"
//#include "modules/canbus/vehicle/ls/protocol/ecu_status_3_517.h"
#include "modules/canbus/vehicle/ls/protocol/gear_command_104.h"
#include "modules/canbus/vehicle/ls/protocol/gear_status__204.h"
#include "modules/canbus/vehicle/ls/protocol/steer_command_103.h"
#include "modules/canbus/vehicle/ls/protocol/steer_status__203.h"
#include "modules/canbus/vehicle/ls/protocol/throttle_command_101.h"
#include "modules/canbus/vehicle/ls/protocol/throttle_status__201.h"
#include "modules/canbus/vehicle/ls/protocol/turnsignal_command_104.h"
#include "modules/canbus/vehicle/ls/protocol/turnsignal_status__204.h"

namespace apollo {
namespace canbus {
namespace ls {

LsMessageManager::LsMessageManager() {
  // Control Messages
  AddSendProtocolData<Brakecommand102, true>();
  AddSendProtocolData<Controlcommand100, true>();
  AddSendProtocolData<Gearcommand104, true>();
  AddSendProtocolData<Steercommand103, true>();
  AddSendProtocolData<Throttlecommand101, true>();
  AddSendProtocolData<Turnsignalcommand104, true>();

  // Report Messages
  AddRecvProtocolData<Brakestatus202, true>();
  AddRecvProtocolData<Ecustatus200, true>();
  //AddRecvProtocolData<Ecustatus2516, true>();
 // AddRecvProtocolData<Ecustatus3517, true>();
  AddRecvProtocolData<Gearstatus204, true>();
  AddRecvProtocolData<Steerstatus203, true>();
  AddRecvProtocolData<Throttlestatus201, true>();
  AddRecvProtocolData<Turnsignalstatus204, true>();
  AddRecvProtocolData<Speedfeedback206,true>();
}

LsMessageManager::~LsMessageManager() {}

}  // namespace ls
}  // namespace canbus
}  // namespace apollo
