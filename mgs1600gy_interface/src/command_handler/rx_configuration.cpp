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

#include "mgs1600gy_interface/command_handler/rx_configuration.hpp"

namespace mgs1600gy_interface
{

RxConfiguration::RxConfiguration()
: prettier_(std::make_unique<Prettier>(this->prefix_))
{
}

std::string RxConfiguration::yieldANAM() const noexcept
{
  return this->prettier_->exec("ANAM");
}

std::string RxConfiguration::yieldBADJ() const noexcept
{
  return this->prettier_->exec("BADJ");
}

std::string RxConfiguration::yieldBRUN() const noexcept
{
  return this->prettier_->exec("BRUN");
}

std::string RxConfiguration::yieldDIM() const noexcept
{
  return this->prettier_->exec("DIM");
}

std::string RxConfiguration::yieldFCAL() const noexcept
{
  return this->prettier_->exec("FCAL");
}

std::string RxConfiguration::yieldGRNG() const noexcept
{
  return this->prettier_->exec("GRNG");
}

std::string RxConfiguration::yieldMMOD() const noexcept
{
  return this->prettier_->exec("MMOD");
}

std::string RxConfiguration::yieldPWMM() const noexcept
{
  return this->prettier_->exec("PWMM");
}

std::string RxConfiguration::yieldRSBR() const noexcept
{
  return this->prettier_->exec("RSBR");
}

std::string RxConfiguration::yieldSCRO() const noexcept
{
  return this->prettier_->exec("SCRO");
}

std::string RxConfiguration::yieldTINV() const noexcept
{
  return this->prettier_->exec("TINV");
}

std::string RxConfiguration::yieldTMS() const noexcept
{
  return this->prettier_->exec("TMS");
}

std::string RxConfiguration::yieldTPOL() const noexcept
{
  return this->prettier_->exec("TPOL");
}

std::string RxConfiguration::yieldTWDT() const noexcept
{
  return this->prettier_->exec("TWDT");
}

std::string RxConfiguration::yieldTXOF() const noexcept
{
  return this->prettier_->exec("TXOF");
}

std::string RxConfiguration::yieldZADJ() const noexcept
{
  return this->prettier_->exec("ZADJ");
}

}  // namespace mgs1600gy_interface
