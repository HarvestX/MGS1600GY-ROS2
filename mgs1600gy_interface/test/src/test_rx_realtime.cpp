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

#include <mgs1600gy_interface/command_handler/rx_realtime.hpp>


class TestRxRealtime : public ::testing::Test
{
protected:
  std::unique_ptr<const mgs1600gy_interface::RxRealtime> generator;
  virtual void SetUp()
  {
    this->generator =
      std::make_unique<const mgs1600gy_interface::RxRealtime>();
  }

  virtual void TearDown() {}
};


TEST_F(TestRxRealtime, commandGenerationTest)
{
  ASSERT_EQ(
    "?B 1\r", this->generator->yieldB(1));
  ASSERT_EQ(
    "?MGD\r", this->generator->yieldMGD());
  ASSERT_EQ(
    "?MGM\r", this->generator->yieldMGM());
  ASSERT_EQ(
    "?MGM 1\r", this->generator->yieldMGM(1));
  ASSERT_EQ(
    "?MZ\r", this->generator->yieldMZ());
  ASSERT_EQ(
    "?MZ 1\r", this->generator->yieldMZ(1));
  ASSERT_EQ(
    "?T\r", this->generator->yieldT());
  ASSERT_EQ(
    "?MGT\r", this->generator->yieldMGT());
  ASSERT_EQ(
    "?MGT 1\r", this->generator->yieldMGT(1));
  ASSERT_EQ(
    "?VAR 1\r", this->generator->yieldVAR(1));
  ASSERT_EQ(
    "?GY\r", this->generator->yieldGY());
  ASSERT_EQ(
    "?GY 1\r", this->generator->yieldGY(1));
  ASSERT_EQ(
    "?MGS\r", this->generator->yieldMGS());
  ASSERT_EQ(
    "?MGX\r", this->generator->yieldMGX());
  ASSERT_EQ(
    "?ANG\r", this->generator->yieldANG());
  ASSERT_EQ(
    "?ANG 1\r", this->generator->yieldANG(1));
}
