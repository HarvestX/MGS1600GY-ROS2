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

#include <mgs1600gy_interface/converter.hpp>


class TestConverter : public ::testing::Test
{
protected:
  static const size_t NUM_SENSOR = 16;
  std::unique_ptr<mgs1600gy_interface::Converter<int, NUM_SENSOR>> converter;
  virtual void SetUp()
  {
    this->converter =
      std::make_unique<mgs1600gy_interface::Converter<int, NUM_SENSOR>>(
      0, 2000);
  }

  virtual void TearDown() {}
};

class TestFlippedConverter : public ::testing::Test
{
protected:
  static const size_t NUM_SENSOR = 16;
  std::unique_ptr<mgs1600gy_interface::Converter<int, NUM_SENSOR>> converter;
  virtual void SetUp()
  {
    this->converter =
      std::make_unique<mgs1600gy_interface::Converter<int, NUM_SENSOR>>(
      0, 2000, true);
  }

  virtual void TearDown() {}
};

TEST_F(TestConverter, yieldBase)
{
  auto out_data = this->converter->yieldBaseCvMat();
  ASSERT_EQ(16, out_data.cols);
  ASSERT_EQ(1, out_data.rows);
  ASSERT_EQ(1, out_data.channels());
  ASSERT_EQ(CV_8U, out_data.type());
}

TEST_F(TestConverter, yieldBlurBase)
{
  auto out_data = this->converter->yieldBaseBlurCvMat();
  ASSERT_EQ(160, out_data.cols);
  ASSERT_EQ(1, out_data.rows);
  ASSERT_EQ(1, out_data.channels());
  ASSERT_EQ(CV_8U, out_data.type());
}

TEST_F(TestConverter, convertGoodData1) {
  const std::array<int, NUM_SENSOR> input_data = {
    0, 0, 0, 0,
    2000, 2000, 2000, 2000,
    -1000, -1000, 1000, 1000,
    3000, 3000, 3000, 3000};

  const std::array<uint8_t, NUM_SENSOR> expected_data = {
    255, 255, 255, 255, 0, 0, 0, 0,
    255, 255, 128, 128, 0, 0, 0, 0};

  auto actual = this->converter->yieldBaseCvMat();
  this->converter->convert(input_data, actual);
  for (int y = 0; y < actual.rows; ++y) {
    for (int x = 0; x < actual.cols; ++x) {
      const int idx = y * actual.cols + x;
      ASSERT_EQ(
        expected_data.at(idx),
        actual.at<uint8_t>(0, idx));
    }
  }
}

TEST_F(TestFlippedConverter, convertGoodData1) {
  const std::array<int, NUM_SENSOR> input_data = {
    0, 0, 0, 0,
    2000, 2000, 2000, 2000,
    -1000, -1000, 1000, 1000,
    3000, 3000, 3000, 3000};

  const std::array<uint8_t, NUM_SENSOR> expected_data = {
    0, 0, 0, 0, 128, 128, 255, 255,
    0, 0, 0, 0, 255, 255, 255, 255};

  auto actual = this->converter->yieldBaseCvMat();
  this->converter->convert(input_data, actual);
  for (int y = 0; y < actual.rows; ++y) {
    for (int x = 0; x < actual.cols; ++x) {
      const int idx = y * actual.cols + x;
      ASSERT_EQ(
        expected_data.at(idx),
        actual.at<uint8_t>(0, idx));
    }
  }
}
