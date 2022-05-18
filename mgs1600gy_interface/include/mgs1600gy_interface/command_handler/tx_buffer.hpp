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


#pragma once

#include <string>
#include <memory>
#include "mgs1600gy_interface/command_handler/prettier.hpp"

namespace mgs1600gy_interface
{
class TxBuffer
{
private:
  const std::string prefix_ = "#";
  const std::unique_ptr<const Prettier> prettier_;

public:
  TxBuffer();

  std::string yieldRepeat() const noexcept;
  std::string yieldRepeatEvery(const int) const noexcept;
  std::string yieldClear() const noexcept;
};
}  // namespace mgs1600gy_interface
