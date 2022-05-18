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

#include "mgs1600gy_interface/command_handler/tx_buffer.hpp"

namespace mgs1600gy_interface
{
TxBuffer::TxBuffer()
:  prettier_(std::make_unique<Prettier>(this->prefix_))
{
}

std::string TxBuffer::yieldRepeat() const noexcept
{
  return this->prettier_->exec(" ");
}

std::string TxBuffer::yieldRepeatEvery(const int ms) const noexcept
{
  return this->prettier_->exec("", ms);
}

std::string TxBuffer::yieldClear() const noexcept
{
  return this->prettier_->exec(" C");
}


}  // namespace mgs1600gy_interface
