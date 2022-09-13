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
#include <mgs1600gy_interface/commander/realtime_commander.hpp>

using namespace std::chrono_literals;
using ::testing::_;
using ::testing::StrEq;
using ::testing::Return;
using ::testing::DoAll;
using MD = mgs1600gy_interface::RealtimeCommander::MODE;
using RS = mgs1600gy_interface::RESPONSE_STATE;

ACTION_P(StrCpyToArg0, str) {
  strcpy(arg0, str);
}

class MockPortHandler : public mgs1600gy_interface::PortHandlerBase
{
public:
  MockPortHandler()
  : mgs1600gy_interface::PortHandlerBase()
  {
  }

  MOCK_METHOD(size_t, getBytesAvailable, (), (const override));
  MOCK_METHOD(size_t, readPort, (char * const, const size_t), (const override));
  MOCK_METHOD(
    size_t, writePort,
    (const char * const, const size_t), (const override));
};


class TestRealtimeCommander : public ::testing::Test
{
protected:
  std::shared_ptr<mgs1600gy_interface::PacketHandler> packet_handler;
  std::unique_ptr<mgs1600gy_interface::RealtimeCommander> commander;

  MockPortHandler mock_port_handler;
  virtual void SetUp()
  {
    this->packet_handler =
      std::make_shared<mgs1600gy_interface::PacketHandler>(
      &mock_port_handler);

    this->commander =
      std::make_unique<mgs1600gy_interface::RealtimeCommander>(
      this->packet_handler, 1ms);
  }
};

TEST_F(TestRealtimeCommander, readMzOK)
{
  const std::string sending = "?MZ\r";
  const std::string response =
    sending +
    "MZ=-56:-45:-39:-45:-53:-53:-42:-35:-45:-40:-53:-36:-37:-29:-24:-28\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sending.size()))
  .Times(1);

  EXPECT_CALL(mock_port_handler, getBytesAvailable())
  .WillRepeatedly(Return(response.size()));

  EXPECT_CALL(mock_port_handler, readPort(_, _))
  .WillRepeatedly(
    DoAll(
      StrCpyToArg0(response.c_str()),
      Return(response.size())));

  std::array<float, 16> expected = {
    -56, -45, -39, -45,
    -53, -53, -42, -35,
    -45, -40, -53, -36,
    -37, -29, -24, -28
  };

  std::array<float, 16> actual;
  ASSERT_EQ(this->commander->readMZ(actual, MD::NORMAL), RS::OK);

  for (size_t i = 0; i < expected.size(); ++i) {
    ASSERT_FLOAT_EQ(expected.at(i), actual.at(i));
  }
}

TEST_F(TestRealtimeCommander, readMzQueryModeOK)
{
  const std::string sending = "?MZ\r";
  const std::string response =
    "MZ=-56:-45:-39:-45:-53:-53:-42:-35:-45:-40:-53:-36:-37:-29:-24:-28\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sending.size()))
  .Times(0);

  EXPECT_CALL(mock_port_handler, getBytesAvailable())
  .WillRepeatedly(Return(response.size()));

  EXPECT_CALL(mock_port_handler, readPort(_, _))
  .WillRepeatedly(
    DoAll(
      StrCpyToArg0(response.c_str()),
      Return(response.size())));

  std::array<float, 16> expected = {
    -56, -45, -39, -45,
    -53, -53, -42, -35,
    -45, -40, -53, -36,
    -37, -29, -24, -28
  };

  std::array<float, 16> actual;
  ASSERT_EQ(this->commander->readMZ(actual, MD::QUERY), RS::OK);

  for (size_t i = 0; i < expected.size(); ++i) {
    ASSERT_FLOAT_EQ(expected.at(i), actual.at(i));
  }
}

TEST_F(TestRealtimeCommander, readMzNG)
{
  const std::string sending = "?MZ\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sending.size()))
  .Times(1);
  EXPECT_CALL(mock_port_handler, getBytesAvailable())
  .WillRepeatedly(Return(0));

  std::array<float, 16> actual;
  ASSERT_EQ(
    this->commander->readMZ(actual, MD::NORMAL),
    RS::ERROR_NO_RESPONSE);
}

TEST_F(TestRealtimeCommander, readAngOK)
{
  const std::string sending = "?ANG\r";
  const std::string response =
    sending +
    "ANG=17:5:-9\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sending.size()))
  .Times(1);

  EXPECT_CALL(mock_port_handler, getBytesAvailable())
  .WillRepeatedly(Return(response.size()));

  EXPECT_CALL(mock_port_handler, readPort(_, _))
  .WillRepeatedly(
    DoAll(
      StrCpyToArg0(response.c_str()),
      Return(response.size())));

  std::array<float, 3> expected = {
    17, 5, -9};

  std::array<float, 3> actual;
  ASSERT_EQ(this->commander->readANG(actual, MD::NORMAL), RS::OK);

  for (size_t i = 0; i < expected.size(); ++i) {
    ASSERT_FLOAT_EQ(expected.at(i), actual.at(i));
  }
}

TEST_F(TestRealtimeCommander, readAngQueryModeOK)
{
  const std::string sending = "?ANG\r";
  const std::string response = "ANG=17:5:-9\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sending.size()))
  .Times(0);

  EXPECT_CALL(mock_port_handler, getBytesAvailable())
  .WillRepeatedly(Return(response.size()));

  EXPECT_CALL(mock_port_handler, readPort(_, _))
  .WillRepeatedly(
    DoAll(
      StrCpyToArg0(response.c_str()),
      Return(response.size())));

  std::array<float, 3> expected = {
    17, 5, -9};

  std::array<float, 3> actual;
  ASSERT_EQ(this->commander->readANG(actual, MD::QUERY), RS::OK);

  for (size_t i = 0; i < expected.size(); ++i) {
    ASSERT_FLOAT_EQ(expected.at(i), actual.at(i));
  }
}

TEST_F(TestRealtimeCommander, readAngNG)
{
  const std::string sending = "?ANG\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sending.size()))
  .Times(1);

  EXPECT_CALL(mock_port_handler, getBytesAvailable())
  .WillRepeatedly(Return(0));

  std::array<float, 3> actual;
  ASSERT_EQ(
    this->commander->readANG(actual, MD::NORMAL),
    RS::ERROR_NO_RESPONSE);
}
