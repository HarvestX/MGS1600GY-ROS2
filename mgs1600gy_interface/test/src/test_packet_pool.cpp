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


#include <gtest/gtest.h>
#include <mgs1600gy_interface/packet_pool.hpp>

class TestPacketPool : public ::testing::Test
{
protected:
  std::unique_ptr<mgs1600gy_interface::PacketPool> pool;
  virtual void SetUp()
  {
    this->pool =
      std::make_unique<mgs1600gy_interface::PacketPool>();
  }
};

TEST_F(TestPacketPool, enqueueFineCommands) {
  const std::string MZ_command =
    "MZ=-56:-45:-39:-45:-53:-53:-42:-35:-45:-40:-53:-36:-37:-29:-24:-28\r";
  const std::string ANG_command =
    "ANG=17:5:-9\r";

  this->pool->enqueue(
    MZ_command +
    ANG_command);

  std::string packet = "";
  using PT = mgs1600gy_interface::PacketPool::PACKET_TYPE;
  ASSERT_TRUE(
    this->pool->takePacket(
      PT::MZ, packet));
  EXPECT_EQ(packet, MZ_command);

  ASSERT_TRUE(
    this->pool->takePacket(
      PT::ANG, packet));
  EXPECT_EQ(packet, ANG_command);

  ASSERT_FALSE(
    this->pool->takePacket(
      PT::MZ, packet));
  ASSERT_FALSE(
    this->pool->takePacket(
      PT::ANG, packet));
}

TEST(TestPacketPoolParser, parseGoodResponse1) {
  const std::string MZ_command =
    "MZ=-56:-45:-39:-45:-53:-53:-42:-35:-45:-40:-53:-36:-37:-29:-24:-28\r";
  std::vector<float> expected = {
    -56, -45, -39, -45,
    -53, -53, -42, -35,
    -45, -40, -53, -36,
    -37, -29, -24, -28
  };
  std::vector<float> actual;

  ASSERT_TRUE(
    mgs1600gy_interface::PacketPool::parseResponse(MZ_command, actual));


  ASSERT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    ASSERT_FLOAT_EQ(expected.at(i), actual.at(i));
  }
}

TEST(TestPacketPoolParser, parseGoodResponse2) {
  const std::string ANG_command =
    "ANG=17:5:-9\r";
  std::vector<float> expected = {
    17, 5, -9};
  std::vector<float> actual;

  ASSERT_TRUE(
    mgs1600gy_interface::PacketPool::parseResponse(ANG_command, actual));

  ASSERT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    ASSERT_FLOAT_EQ(expected.at(i), actual.at(i));
  }
}

TEST(TestPacketPoolParser, parseBADResponse) {
  const std::string MZ_command =
    "MZ=-56:-45:foo:-45:-53:-53:-42:-35:-45:-40:-53:-36:-37:-29:-24:-28\r";
  std::vector<float> actual;

  ASSERT_FALSE(
    mgs1600gy_interface::PacketPool::parseResponse(MZ_command, actual));
}
