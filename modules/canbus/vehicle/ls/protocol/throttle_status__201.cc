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

#include "modules/canbus/vehicle/ls/protocol/throttle_status__201.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ls {

using ::apollo::drivers::canbus::Byte;

Throttlestatus201::Throttlestatus201() {}
const int32_t Throttlestatus201::ID = 0x201;

void Throttlestatus201::Parse(const std::uint8_t* bytes, int32_t length,
                              ChassisDetail* chassis) const {
  chassis->mutable_ls()
      ->mutable_throttle_status__201()
      ->set_throttle_pedal_en_sts(throttle_pedal_en_sts(bytes, length));
  chassis->mutable_ls()->mutable_throttle_status__201()->set_throttle_pedal_sts(
      throttle_pedal_sts(bytes, length));
  chassis->mutable_check_response()->set_is_vcu_online(
      throttle_pedal_en_sts(bytes, length) == 1);
}

// config detail: {'description': 'throttle pedal enable bit(Status)', 'enum':
// {0: 'THROTTLE_PEDAL_EN_STS_DISABLE', 1: 'THROTTLE_PEDAL_EN_STS_ENABLE', 2:
// 'THROTTLE_PEDAL_EN_STS_TAKEOVER'}, 'precision': 1.0, 'len': 8, 'name':
// 'throttle_pedal_en_sts', 'is_signed_var': False, 'offset': 0.0,
// 'physical_range': '[0|1]', 'bit': 0, 'type': 'enum', 'order': 'intel',
// 'physical_unit': ''}
Throttle_status__201::Throttle_pedal_en_stsType
Throttlestatus201::throttle_pedal_en_sts(const std::uint8_t* bytes,
                                         int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  Throttle_status__201::Throttle_pedal_en_stsType ret =
      static_cast<Throttle_status__201::Throttle_pedal_en_stsType>(x);
  return ret;
}

// config detail: {'description': 'Percentage of throttle pedal(Status)',
// 'offset': 0.0, 'precision': 1.0, 'len': 8, 'name': 'throttle_pedal_sts',
// 'is_signed_var': False, 'physical_range': '[0|100]', 'bit': 8, 'type': 'int',
// 'order': 'intel', 'physical_unit': '%'}
int Throttlestatus201::throttle_pedal_sts(const std::uint8_t* bytes,
                                          int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace ls
}  // namespace canbus
}  // namespace apollo
