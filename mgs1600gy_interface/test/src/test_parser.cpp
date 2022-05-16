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

#include <mgs1600gy_interface/command_handler/parser.hpp>


class TestMagnetParser : public ::testing::Test
{
protected:
  static const int NUM_SENSOR = 16;
  std::unique_ptr<mgs1600gy_interface::Parser<int, NUM_SENSOR>> parser;
  virtual void SetUp()
  {
    this->parser =
      std::make_unique<
      mgs1600gy_interface::Parser<int, NUM_SENSOR>>("MGParser");
  }

  virtual void TearDown() {}
};

class TestGyroParser : public ::testing::Test
{
protected:
  static const int NUM_SENSOR = 3;
  std::unique_ptr<mgs1600gy_interface::Parser<int, NUM_SENSOR>> parser;
  virtual void SetUp()
  {
    this->parser =
      std::make_unique<
      mgs1600gy_interface::Parser<int, NUM_SENSOR>>("GyroParser");
  }

  virtual void TearDown() {}
};

TEST_F(TestMagnetParser, parseBadData1) {
  // Rack of '='
  const std::string bad_data =
    "MZ-56:-48:-39:-45:-53:-53:-42:-35:-42:-40:-53:-36:-37:-29:-24:-31\r";
  std::array<int, NUM_SENSOR> actual = {
    777, 777, 777, 777,
    777, 777, 777, 777,
    777, 777, 777, 777,
    777, 777, 777, 777};

  bool res = this->parser->parse(bad_data, actual);
  ASSERT_FALSE(res);

  // assert value is not changed
  for (auto data : actual) {
    ASSERT_EQ(777, data);
  }
}

TEST_F(TestMagnetParser, parseBadData2) {
  // Rack of ':'
  const std::string bad_data =
    "MZ=-56-48:-39:-45:-53:-53:-42:-35:-42:-40:-53:-36:-37:-29:-24:-31\r";
  std::array<int, NUM_SENSOR> actual = {
    777, 777, 777, 777,
    777, 777, 777, 777,
    777, 777, 777, 777,
    777, 777, 777, 777};

  bool res = this->parser->parse(bad_data, actual);
  ASSERT_FALSE(res);

  // assert value is not changed
  for (auto data : actual) {
    ASSERT_EQ(777, data);
  }
}

TEST_F(TestMagnetParser, parseBadData3) {
  // Too many ':'
  const std::string bad_data =
    "MZ=-56:-48:-39:-45:-53:-53:-42:-35:-42:-40:-53:-36:-37:-29:-24:-31:6000\r";
  std::array<int, NUM_SENSOR> actual = {
    777, 777, 777, 777,
    777, 777, 777, 777,
    777, 777, 777, 777,
    777, 777, 777, 777};

  bool res = this->parser->parse(bad_data, actual);
  ASSERT_FALSE(res);

  // Check value is not changeed
  for (auto data : actual) {
    ASSERT_EQ(777, data);
  }
}

TEST_F(TestMagnetParser, parseGoodData1) {
  const std::string input_data =
    "MZ=-56:-48:-39:-45:-53:-53:-42:-35:-42:-40:-53:-36:-37:-29:-24:-31\r";
  const std::array<int, NUM_SENSOR> expected = {
    -56, -48, -39, -45,
    -53, -53, -42, -35,
    -42, -40, -53, -36,
    -37, -29, -24, -31};
  std::array<int, NUM_SENSOR> actual;

  bool res = this->parser->parse(input_data, actual);
  ASSERT_TRUE(res);
  for (int i = 0; i < this->NUM_SENSOR; ++i) {
    ASSERT_EQ(expected.at(i), actual.at(i));
  }
}

TEST_F(TestMagnetParser, parseGoodData2) {
  const std::string input_data =
    "MZ=-56:-45:-39:-45:-53:-53:-42:-35:-42:-40:-53:-36:-37:-26:-24:-28\r";
  const std::array<int, NUM_SENSOR> expected = {
    -56, -45, -39, -45,
    -53, -53, -42, -35,
    -42, -40, -53, -36,
    -37, -26, -24, -28};
  std::array<int, NUM_SENSOR> actual;

  bool res = this->parser->parse(input_data, actual);
  ASSERT_TRUE(res);
  for (int i = 0; i < this->NUM_SENSOR; ++i) {
    ASSERT_EQ(expected.at(i), actual.at(i));
  }
}

TEST_F(TestGyroParser, parseGoodData1)
{
  const std::string input_data =
    "ANG=16:4:-9\r";
  const std::array<int, NUM_SENSOR> expected = {
    16, 4, -9};
  std::array<int, NUM_SENSOR> actual;

  bool res = this->parser->parse(input_data, actual);
  ASSERT_TRUE(res);
  for (int i = 0; i < this->NUM_SENSOR; ++i) {
    ASSERT_EQ(expected.at(i), actual.at(i));
  }
}

TEST_F(TestGyroParser, parseGoodData2)
{
  const std::string input_data =
    "ANG=22:5:-12\r";
  const std::array<int, NUM_SENSOR> expected = {
    22, 5, -12};
  std::array<int, NUM_SENSOR> actual;

  bool res = this->parser->parse(input_data, actual);
  ASSERT_TRUE(res);
  for (int i = 0; i < this->NUM_SENSOR; ++i) {
    ASSERT_EQ(expected.at(i), actual.at(i));
  }
}
