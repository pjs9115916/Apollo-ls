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

#include "modules/canbus/vehicle/ls/protocol/gear_status__204.h"
#include "glog/logging.h"
#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace ls {

using ::apollo::drivers::canbus::Byte;

Gearstatus204::Gearstatus204() {}
const int32_t Gearstatus204::ID = 0x204;

void Gearstatus204::Parse(const std::uint8_t* bytes, int32_t length,
                          ChassisDetail* chassis) const {
  chassis->mutable_ls()->mutable_gear_status__204()->set_gear_sts(
      gear_sts(bytes, length));
}

// config detail: {'description': 'PRND control(Status)', 'enum': {1:
// 'GEAR_STS_PARK', 2: 'GEAR_STS_REVERSE', 3: 'GEAR_STS_NEUTRAL', 4:
// 'GEAR_STS_DRIVE'}, 'precision': 1.0, 'len': 8, 'name': 'gear_sts',
// 'is_signed_var': False, 'offset': 0.0, 'physical_range': '[1|4]', 'bit': 0,
// 'type': 'enum', 'order': 'intel', 'physical_unit': ''}
Gear_status__204::Gear_stsType Gearstatus204::gear_sts(const std::uint8_t* bytes,
                                                      int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 4);

  Gear_status__204::Gear_stsType ret =
      static_cast<Gear_status__204::Gear_stsType>(x);
  return ret;
}
}  // namespace ls
}  // namespace canbus
}  // namespace apollo
