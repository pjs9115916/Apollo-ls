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

#include "modules/canbus/vehicle/ls/protocol/brake_command_102.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace ls {

using ::apollo::drivers::canbus::Byte;

const int32_t Brakecommand102::ID = 0x102;

// public
Brakecommand102::Brakecommand102() { Reset(); }

uint32_t Brakecommand102::GetPeriod() const {
  // modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000; 
  //此处需要根据具体的线控协议进行修改，领世当前的版本是10ms，林肯也是10ms；
  return PERIOD;
}

void Brakecommand102::UpdateData(uint8_t* data) {
  set_p_brake_pedal_en_ctrl(data, brake_pedal_en_ctrl_);
  set_p_brake_pedal_cmd(data, brake_pedal_cmd_);
}

void Brakecommand102::Reset() {
  // you should check this manually
  brake_pedal_en_ctrl_ = Brake_command_102::BRAKE_PEDAL_EN_CTRL_DISABLE;
  brake_pedal_cmd_ = 0;
}

Brakecommand102* Brakecommand102::set_brake_pedal_en_ctrl(
    Brake_command_102::Brake_pedal_en_ctrlType brake_pedal_en_ctrl) {
  brake_pedal_en_ctrl_ = brake_pedal_en_ctrl;
  return this;
}

// config detail: {'description': 'brake pedal enable bit(Command)', 'enum': {0:
// 'BRAKE_PEDAL_EN_CTRL_DISABLE', 1: 'BRAKE_PEDAL_EN_CTRL_ENABLE'},
// 'precision': 1.0, 'len': 8, 'name': 'BRAKE_PEDAL_EN_CTRL', 'is_signed_var':
// False, 'offset': 0.0, 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum',
// 'order': 'intel', 'physical_unit': ''}
void Brakecommand102::set_p_brake_pedal_en_ctrl(
    uint8_t* data,
    Brake_command_102::Brake_pedal_en_ctrlType brake_pedal_en_ctrl) {
  int x = brake_pedal_en_ctrl;

  Byte to_set(data + 0);
  to_set.set_value(static_cast<uint8_t>(x), 7, 1);
}

Brakecommand102* Brakecommand102::set_brake_pedal_cmd(int brake_pedal_cmd) {
  brake_pedal_cmd_ = brake_pedal_cmd;
  return this;
}

// config detail: {'description': 'Percentage of brake pedal(Command)',
// 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'BRAKE_PEDAL_CMD',
// 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int',
// 'order': 'intel', 'physical_unit': '%'}
void Brakecommand102::set_p_brake_pedal_cmd(uint8_t* data,
                                            int brake_pedal_cmd) {
  brake_pedal_cmd = ProtocolData::BoundedValue(0, 100, brake_pedal_cmd);
  int x = brake_pedal_cmd;

   //此处油门值是从第1位开始长度为10,此处应该可以取近似值即舍弃掉第0个字节的第1位和第0位;
  //Byte to_set(data + 1);
  //to_set.set_value(static_cast<uint8_t>(x), 0, 8);
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
