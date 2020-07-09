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
#include "gtest/gtest.h"
#include "modules/canbus/vehicle/ls/protocol/brake_command_102.h"
#include "modules/canbus/vehicle/ls/protocol/brake_status__202.h"
#include "modules/canbus/vehicle/ls/protocol/control_command_100.h"
#include "modules/canbus/vehicle/ls/protocol/ecu_status__200.h"
//#include "modules/canbus/vehicle/ls/protocol/ecu_status_2_516.h"
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
using ::apollo::canbus::ChassisDetail;
using ::apollo::drivers::canbus::ProtocolData;

class LsMessageManagerTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
};

TEST_F(LsMessageManagerTest, Brakecommand102) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Brakecommand102::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Brakecommand102 *>(pd)->ID, Brakecommand102::ID);
}

TEST_F(LsMessageManagerTest, Brakestatus202) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Brakestatus202::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Brakestatus202 *>(pd)->ID, Brakestatus202::ID);
}

TEST_F(LsMessageManagerTest, Controlcommand100) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Controlcommand100::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Controlcommand100 *>(pd)->ID, Controlcommand100::ID);
}

TEST_F(LsMessageManagerTest, Ecustatus200) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Ecustatus200::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Ecustatus200 *>(pd)->ID, Ecustatus200::ID);
}

/*TEST_F(LsMessageManagerTest, Ecustatus2516) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Ecustatus2516::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Ecustatus2516 *>(pd)->ID, Ecustatus2516::ID);
}

TEST_F(LsMessageManagerTest, Ecustatus3517) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Ecustatus3517::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Ecustatus3517 *>(pd)->ID, Ecustatus3517::ID);
}*/

TEST_F(LsMessageManagerTest, Gearcommand104) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Gearcommand104::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gearcommand104 *>(pd)->ID, Gearcommand104::ID);
}

TEST_F(LsMessageManagerTest, Gearstatus204) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Gearstatus204::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Gearstatus204 *>(pd)->ID, Gearstatus204::ID);
}

TEST_F(LsMessageManagerTest, Steercommand103) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Steercommand103::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Steercommand103 *>(pd)->ID, Steercommand103::ID);
}

TEST_F(LsMessageManagerTest, Steerstatus203) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Steerstatus203::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Steerstatus203 *>(pd)->ID, Steerstatus203::ID);
}

TEST_F(LsMessageManagerTest, Throttlecommand101) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Throttlecommand101::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Throttlecommand101 *>(pd)->ID, Throttlecommand101::ID);
}

TEST_F(LsMessageManagerTest, Throttlestatus201) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Throttlestatus201::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Throttlestatus201 *>(pd)->ID, Throttlestatus201::ID);
}

TEST_F(LsMessageManagerTest, Turnsignalcommand104) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Turnsignalcommand104::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Turnsignalcommand104 *>(pd)->ID,
            Turnsignalcommand104::ID);
}

TEST_F(LsMessageManagerTest, Turnsignalstatus204) {
  LsMessageManager manager;
  ProtocolData<ChassisDetail> *pd =
      manager.GetMutableProtocolDataById(Turnsignalstatus204::ID);
  EXPECT_NE(pd, nullptr);
  EXPECT_EQ(static_cast<Turnsignalstatus204 *>(pd)->ID,
            Turnsignalstatus204::ID);
}

}  // namespace ls
}  // namespace canbus
}  // namespace apollo
