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

#include <mgs1600gy_interface/command_handler/rx_configuration.hpp>


class TestRxConfiguration : public ::testing::Test
{
protected:
  std::unique_ptr<const mgs1600gy_interface::RxConfiguration> generator;
  virtual void SetUp()
  {
    this->generator =
      std::make_unique<const mgs1600gy_interface::RxConfiguration>();
  }

  virtual void TearDown() {}
};


TEST_F(TestRxConfiguration, commandGenerationTest)
{
  ASSERT_EQ(
    "~ANAM\r", this->generator->yieldANAM());
  ASSERT_EQ(
    "~BADJ\r", this->generator->yieldBADJ());
  ASSERT_EQ(
    "~BRUN\r", this->generator->yieldBRUN());
  ASSERT_EQ(
    "~DIM\r", this->generator->yieldDIM());
  ASSERT_EQ(
    "~FCAL\r", this->generator->yieldFCAL());
  ASSERT_EQ(
    "~GRNG\r", this->generator->yieldGRNG());
  ASSERT_EQ(
    "~MMOD\r", this->generator->yieldMMOD());
  ASSERT_EQ(
    "~PWMM\r", this->generator->yieldPWMM());
  ASSERT_EQ(
    "~RSBR\r", this->generator->yieldRSBR());
  ASSERT_EQ(
    "~SCRO\r", this->generator->yieldSCRO());
  ASSERT_EQ(
    "~TINV\r", this->generator->yieldTINV());
  ASSERT_EQ(
    "~TMS\r", this->generator->yieldTMS());
  ASSERT_EQ(
    "~TPOL\r", this->generator->yieldTPOL());
  ASSERT_EQ(
    "~TWDT\r", this->generator->yieldTWDT());
  ASSERT_EQ(
    "~TXOF\r", this->generator->yieldTXOF());
  ASSERT_EQ(
    "~ZADJ\r", this->generator->yieldZADJ());
}
