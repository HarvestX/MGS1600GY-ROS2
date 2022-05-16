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

#include "mgs1600gy_interface/command_handler/rx_realtime.hpp"

namespace mgs1600gy_interface
{
RxRealtime::RxRealtime()
:  prettier_(std::make_unique<Prettier>(this->prefix_))
{
}

std::string RxRealtime::yieldB(const int arg) const noexcept
{
  return this->prettier_->exec("B", arg);
}

std::string RxRealtime::yieldMGD() const noexcept
{
  return this->prettier_->exec("MGD");
}

std::string RxRealtime::yieldMGM() const noexcept
{
  return this->prettier_->exec("MGM");
}

std::string RxRealtime::yieldMGM(const int arg) const noexcept
{
  return this->prettier_->exec("MGM", arg);
}

std::string RxRealtime::yieldMZ() const noexcept
{
  return this->prettier_->exec("MZ");
}

std::string RxRealtime::yieldMZ(const int arg) const noexcept
{
  return this->prettier_->exec("MZ", arg);
}

std::string RxRealtime::yieldT() const noexcept
{
  return this->prettier_->exec("T");
}

std::string RxRealtime::yieldMGT() const noexcept
{
  return this->prettier_->exec("MGT");
}

std::string RxRealtime::yieldMGT(const int arg) const noexcept
{
  return this->prettier_->exec("MGT", arg);
}

std::string RxRealtime::yieldVAR(const int arg) const noexcept
{
  return this->prettier_->exec("VAR", arg);
}

std::string RxRealtime::yieldGY() const noexcept
{
  return this->prettier_->exec("GY");
}

std::string RxRealtime::yieldGY(const int arg) const noexcept
{
  return this->prettier_->exec("GY", arg);
}

std::string RxRealtime::yieldMGS() const noexcept
{
  return this->prettier_->exec("MGS");
}

std::string RxRealtime::yieldMGX() const noexcept
{
  return this->prettier_->exec("MGX");
}

std::string RxRealtime::yieldANG() const noexcept
{
  return this->prettier_->exec("ANG");
}

std::string RxRealtime::yieldANG(const int arg) const noexcept
{
  return this->prettier_->exec("ANG", arg);
}

}  // namespace mgs1600gy_interface
