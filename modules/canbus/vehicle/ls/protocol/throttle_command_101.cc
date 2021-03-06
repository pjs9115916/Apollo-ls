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

#include "modules/canbus/vehicle/ls/protocol/throttle_command_101.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ls {

using ::apollo::drivers::canbus::Byte;


const int32_t Throttlecommand101::ID = 0x101;

// public
Throttlecommand101::Throttlecommand101() { Reset(); }

uint32_t Throttlecommand101::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 10 * 1000;
  return PERIOD;
}

void Throttlecommand101::UpdateData(uint8_t* data) {
  set_p_throttle_pedal_en_ctrl(data, throttle_pedal_en_ctrl_);
  set_p_throttle_pedal_cmd(data, throttle_pedal_cmd_);
}

void Throttlecommand101::Reset() {
  // you should check this manually
  throttle_pedal_en_ctrl_ =
      Throttle_command_101::THROTTLE_PEDAL_EN_CTRL_DISABLE;
     // Throttle_command_101::Throttle_pedal_en_ctrlType::
     
  throttle_pedal_cmd_ = 0;
}

Throttlecommand101* Throttlecommand101::set_throttle_pedal_en_ctrl(
    Throttle_command_101::Throttle_pedal_en_ctrlType throttle_pedal_en_ctrl) {
  throttle_pedal_en_ctrl_ = throttle_pedal_en_ctrl;
  return this;
}

// config detail: {'description': 'throttle pedal enable bit(Command)', 'enum':
// {0: 'THROTTLE_PEDAL_EN_CTRL_DISABLE', 1: 'THROTTLE_PEDAL_EN_CTRL_ENABLE'},
// 'precision': 1.0, 'len': 8, 'name': 'THROTTLE_PEDAL_EN_CTRL',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
void Throttlecommand101::set_p_throttle_pedal_en_ctrl(
    uint8_t* data,
    Throttle_command_101::Throttle_pedal_en_ctrlType throttle_pedal_en_ctrl) {
  int x = throttle_pedal_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);//第7位开始长度1
}

Throttlecommand101* Throttlecommand101::set_throttle_pedal_cmd(
    int throttle_pedal_cmd) {
  throttle_pedal_cmd_ = throttle_pedal_cmd;
  return this;
}

// config detail: {'description': 'Percentage of throttle pedal(Command)',
// 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'THROTTLE_PEDAL_CMD',
// 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int',
// 'order': 'intel', 'physical_unit': '%'}
void Throttlecommand101::set_p_throttle_pedal_cmd(uint8_t* data,
                                                  int throttle_pedal_cmd) {
  throttle_pedal_cmd = ProtocolData::BoundedValue(0, 100, throttle_pedal_cmd);
  int x = throttle_pedal_cmd;
  //此处油门值是从第1位开始长度为10,此处应该可以取近似值即舍弃掉第0个字节的第1位和第0位;
   uint8_t t = 0;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set0(data + 0);
  to_set0.set_value(t, 0, 2);
  x >>= 8;

  t = static_cast<uint8_t>(x & 0xFF);
  Byte to_set1(data + 1);
  to_set1.set_value(t, 0, 8);
}

}  // namespace ls
}  // namespace canbus
}  // namespace apollo
