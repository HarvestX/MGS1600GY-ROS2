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

#include <mgs1600gy_interface/command_handler/tx_buffer.hpp>


class TestTxBuffer : public ::testing::Test
{
protected:
  std::unique_ptr<const mgs1600gy_interface::TxBuffer> generator;
  virtual void SetUp()
  {
    this->generator =
      std::make_unique<const mgs1600gy_interface::TxBuffer>();
  }

  virtual void TearDown() {}
};

TEST_F(TestTxBuffer, commandGenerationTest) {
  ASSERT_EQ(
    "# \r", this->generator->yieldRepeat());
  ASSERT_EQ(
    "# 100\r", this->generator->yieldRepeatEvery(100));
  ASSERT_EQ(
    "# C\r", this->generator->yieldClear());
}
