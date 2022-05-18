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

#include "mgs1600gy_interface/command_handler/tx_realtime.hpp"

namespace mgs1600gy_interface
{
TxRealtime::TxRealtime()
: prettier_(std::make_unique<Prettier>(this->prefix_))
{
}

std::string TxRealtime::yieldB(const int idx, const int val) const noexcept
{
  return this->prettier_->exec("B", idx, val);
}

std::string TxRealtime::yieldR(const int arg) const noexcept
{
  return this->prettier_->exec("R", arg);
}

std::string TxRealtime::yieldTV() const noexcept
{
  return this->prettier_->exec("TV");
}

std::string TxRealtime::yieldVAR(const int idx, const int val) const noexcept
{
  return this->prettier_->exec("VAR", idx, val);
}

std::string TxRealtime::yieldTX() const noexcept
{
  return this->prettier_->exec("TX");
}

std::string TxRealtime::yieldANG(const int idx, const int val) const noexcept
{
  return this->prettier_->exec("ANG", idx, val);
}

std::string TxRealtime::yieldZER() const noexcept
{
  return this->prettier_->exec("ZER");
}
}
