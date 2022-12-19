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

using namespace std::chrono_literals;  // NOLINT
using ::testing::_;
using ::testing::StrEq;
using ::testing::Return;
using ::testing::DoAll;
using MD = mgs1600gy_interface::RealtimeCommander::MODE;
using RS = mgs1600gy_interface::RESPONSE_STATE;

ACTION_P(StrCpyToArg0, str) {
  strcpy(arg0, str);  // NOLINT
}

class Logger : public rclcpp::node_interfaces::NodeLoggingInterface
{
public:
  rclcpp::Logger get_logger() const override
  {
    return rclcpp::get_logger(this->get_logger_name());
  }

  const char * get_logger_name() const override
  {
    return "TestRealtimeCommander";
  }
};


class MockPortHandler : public h6x_serial_interface::PortHandlerBase
{
public:
  MockPortHandler()
  : h6x_serial_interface::PortHandlerBase()
  {
  }

  MOCK_METHOD(ssize_t, getBytesAvailable, (), (const override));
  MOCK_METHOD(ssize_t, readPort, (char * const, const size_t), (const override));
  MOCK_METHOD(
    ssize_t, writePort,
    (const char * const, const size_t), (const override));
};


class TestRealtimeCommander : public ::testing::Test
{
protected:
  mgs1600gy_interface::PacketHandler::SharedPtr packet_handler;
  mgs1600gy_interface::RealtimeCommander::UniquePtr commander;

  MockPortHandler mock_port_handler;
  virtual void SetUp()
  {
    const auto logger = std::make_shared<Logger>();
    this->packet_handler =
      std::make_shared<mgs1600gy_interface::PacketHandler>(
      &mock_port_handler, logger);

    this->commander =
      std::make_unique<mgs1600gy_interface::RealtimeCommander>(
      this->packet_handler, logger, 1ms);
  }
};

TEST_F(TestRealtimeCommander, readMzOK)
{
  const char sending[] = "?MZ\r";
  const std::string response =
    std::string(sending) +
    "MZ=-56:-45:-39:-45:-53:-53:-42:-35:-45:-40:-53:-36:-37:-29:-24:-28\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sizeof(sending)))
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
  const char sending[] = "?MZ\r";
  const std::string response =
    "MZ=-56:-45:-39:-45:-53:-53:-42:-35:-45:-40:-53:-36:-37:-29:-24:-28\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sizeof(sending)))
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
  const char sending[] = "?MZ\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sizeof(sending)))
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
  const char sending[] = "?ANG\r";
  const std::string response =
    std::string(sending) +
    "ANG=17:5:-9\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sizeof(sending)))
  .Times(1);

  EXPECT_CALL(mock_port_handler, getBytesAvailable())
  .WillOnce(Return(response.size()));

  EXPECT_CALL(mock_port_handler, readPort(_, _))
  .WillOnce(
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
  const char sending[] = "?ANG\r";
  const std::string response = "ANG=17:5:-9\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sizeof(sending)))
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
  const char sending[] = "?ANG\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sizeof(sending)))
  .Times(1);

  EXPECT_CALL(mock_port_handler, getBytesAvailable())
  .WillRepeatedly(Return(0));

  std::array<float, 3> actual;
  ASSERT_EQ(
    this->commander->readANG(actual, MD::NORMAL),
    RS::ERROR_NO_RESPONSE);
}

TEST_F(TestRealtimeCommander, writeAngOK)
{
  std::string sending = "!ANG 1 300\r";

  EXPECT_CALL(mock_port_handler, writePort(StrEq(sending), sending.size()))
  .Times(1);

  ASSERT_EQ(
    this->commander->writeANG(1, 300),
    RS::OK);
}
