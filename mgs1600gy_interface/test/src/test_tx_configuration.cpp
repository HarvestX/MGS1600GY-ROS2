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

#include <mgs1600gy_interface/command_handler/tx_configuration.hpp>


class TestTxConfiguration : public ::testing::Test
{
protected:
  std::unique_ptr<const mgs1600gy_interface::TxConfiguration> generator;
  virtual void SetUp()
  {
    this->generator =
      std::make_unique<const mgs1600gy_interface::TxConfiguration>();
  }

  virtual void TearDown() {}
};


TEST_F(TestTxConfiguration, commandGenerationTest)
{
  ASSERT_EQ(
    "^ANAM 1\r", this->generator->yieldANAM(1));
  ASSERT_EQ(
    "^BADJ 1\r", this->generator->yieldBADJ(1));
  ASSERT_EQ(
    "^BRUN 1\r", this->generator->yieldBRUN(1));
  ASSERT_EQ(
    "^DIM 1\r", this->generator->yieldDIM(1));
  ASSERT_EQ(
    "^FCAL 1\r", this->generator->yieldFCAL(1));
  ASSERT_EQ(
    "^GRNG 1\r", this->generator->yieldGRNG(1));
  ASSERT_EQ(
    "^MMOD 1\r", this->generator->yieldMMOD(1));
  ASSERT_EQ(
    "^PWMM 1\r", this->generator->yieldPWMM(1));
  ASSERT_EQ(
    "^RSBR 1\r", this->generator->yieldRSBR(1));
  ASSERT_EQ(
    "^SCRO 1\r", this->generator->yieldSCRO(1));
  ASSERT_EQ(
    "^TINV 1\r", this->generator->yieldTINV(1));
  ASSERT_EQ(
    "^TMS 1\r", this->generator->yieldTMS(1));
  ASSERT_EQ(
    "^TPOL 1\r", this->generator->yieldTPOL(1));
  ASSERT_EQ(
    "^TWDT 1\r", this->generator->yieldTWDT(1));
  ASSERT_EQ(
    "^TXOF 1\r", this->generator->yieldTXOF(1));
  ASSERT_EQ(
    "^ZADJ 1\r", this->generator->yieldZADJ(1));
}
