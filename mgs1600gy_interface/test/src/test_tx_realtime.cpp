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

#include <mgs1600gy_interface/command_handler/tx_realtime.hpp>


class TestTxRealtime : public ::testing::Test
{
protected:
  std::unique_ptr<const mgs1600gy_interface::TxRealtime> generator;
  virtual void SetUp()
  {
    this->generator =
      std::make_unique<const mgs1600gy_interface::TxRealtime>();
  }

  virtual void TearDown() {}
};


TEST_F(TestTxRealtime, commandGenerationTest)
{
  ASSERT_EQ(
    "!B 1 2\r", this->generator->yieldB(1, 2));
  ASSERT_EQ(
    "!R 0\r", this->generator->yieldR(0));
  ASSERT_EQ(
    "!TV\r", this->generator->yieldTV());
  ASSERT_EQ(
    "!VAR 1 2\r", this->generator->yieldVAR(1, 2));
  ASSERT_EQ(
    "!TX\r", this->generator->yieldTX());
  ASSERT_EQ(
    "!ANG 1 200\r", this->generator->yieldANG(1, 200));
  ASSERT_EQ(
    "!ZER\r", this->generator->yieldZER());

}
