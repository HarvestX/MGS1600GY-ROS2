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

#include "mgs1600gy_interface/command_handler/tx_maintenance.hpp"

namespace mgs1600gy_interface
{

TxMaintenance::TxMaintenance()
: prettier_(std::make_unique<Prettier>(this->prefix_))
{
}

std::string TxMaintenance::yieldCLSAV() const noexcept
{
  return this->prettier_->exec("CLSAV");
}

std::string TxMaintenance::yieldCLRST(const int arg) const noexcept
{
  return this->prettier_->exec("CLRST", arg);
}

std::string TxMaintenance::yieldEELD() const noexcept
{
  return this->prettier_->exec("EELD");
}

std::string TxMaintenance::yieldEERST(const int arg) const noexcept
{
  return this->prettier_->exec("EERST", arg);
}

std::string TxMaintenance::yieldEESAV() const noexcept
{
  return this->prettier_->exec("EESAV");
}

std::string TxMaintenance::yieldGREF() const noexcept
{
  return this->prettier_->exec("GREF");
}

std::string TxMaintenance::yieldGZER() const noexcept
{
  return this->prettier_->exec("GZER");
}

std::string TxMaintenance::yieldZERO() const noexcept
{
  return this->prettier_->exec("ZERO");
}

}  // namespace mgs1600gy_interface
