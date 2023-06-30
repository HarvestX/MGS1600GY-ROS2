// Copyright 2022 HarvestX Inc.
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

#include <gmock/gmock.h>
#include <mgs1600gy_interface/commander/maintenance_commander.hpp>
#include <h6x_serial_interface/gmock_port_handler.hpp>

using namespace std::chrono_literals;  // NOLINT
using ::testing::_;
using ::testing::StrEq;
using ::testing::Return;
using ::testing::DoAll;

using namespace mgs1600gy_interface;  // NOLINT
using RS = RESPONSE_STATE;

class TestMaintenanceCommander : public ::testing::Test
{
protected:
  PacketHandler::SharedPtr packet_handler;
  MaintenanceCommander::UniquePtr commander;

  MockPortHandler mock_port_handler;

  virtual void SetUp()
  {
    this->packet_handler = std::make_shared<PacketHandler>(&mock_port_handler);
    this->commander = std::make_unique<MaintenanceCommander>(this->packet_handler, 1ms);
  }
};


TEST_F(TestMaintenanceCommander, writeZeroOK)
{
  const char sending[] = "%ZERO\r";

  EXPECT_CALL(mock_port_handler, write(StrEq(sending), strlen(sending))).Times(1);

  ASSERT_EQ(this->commander->writeZERO(), RS::OK);
}

TEST_F(TestMaintenanceCommander, writeGzerOK)
{
  const char sending[] = "%GZER\r";

  EXPECT_CALL(mock_port_handler, write(StrEq(sending), strlen(sending))).Times(1);

  ASSERT_EQ(this->commander->writeGZER(), RS::OK);
}

TEST_F(TestMaintenanceCommander, writeClsavOK)
{
  const char sending[] = "%CLSAV\r";

  EXPECT_CALL(mock_port_handler, write(StrEq(sending), strlen(sending))).Times(1);

  ASSERT_EQ(this->commander->writeCLSAV(), RS::OK);
}
