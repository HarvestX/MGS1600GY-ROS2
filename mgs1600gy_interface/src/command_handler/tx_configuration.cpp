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

#include "mgs1600gy_interface/command_handler/tx_configuration.hpp"

namespace mgs1600gy_interface
{

TxConfiguration::TxConfiguration()
: prettier_(std::make_unique<Prettier>(this->prefix_))
{
}

std::string TxConfiguration::yieldANAM(const int arg) const noexcept
{
  return this->prettier_->exec("ANAM", arg);
}

std::string TxConfiguration::yieldBADJ(const int arg) const noexcept
{
  return this->prettier_->exec("BADJ", arg);
}

std::string TxConfiguration::yieldBRUN(const int arg) const noexcept
{
  return this->prettier_->exec("BRUN", arg);
}

std::string TxConfiguration::yieldDIM(const int arg) const noexcept
{
  return this->prettier_->exec("DIM", arg);
}

std::string TxConfiguration::yieldFCAL(const int arg) const noexcept
{
  return this->prettier_->exec("FCAL", arg);
}

std::string TxConfiguration::yieldGRNG(const int arg) const noexcept
{
  return this->prettier_->exec("GRNG", arg);
}

std::string TxConfiguration::yieldMMOD(const int arg) const noexcept
{
  return this->prettier_->exec("MMOD", arg);
}

std::string TxConfiguration::yieldPWMM(const int arg) const noexcept
{
  return this->prettier_->exec("PWMM", arg);
}

std::string TxConfiguration::yieldRSBR(const int arg) const noexcept
{
  return this->prettier_->exec("RSBR", arg);
}

std::string TxConfiguration::yieldSCRO(const int arg) const noexcept
{
  return this->prettier_->exec("SCRO", arg);
}

std::string TxConfiguration::yieldTINV(const int arg) const noexcept
{
  return this->prettier_->exec("TINV", arg);
}

std::string TxConfiguration::yieldTMS(const int arg) const noexcept
{
  return this->prettier_->exec("TMS", arg);
}

std::string TxConfiguration::yieldTPOL(const int arg) const noexcept
{
  return this->prettier_->exec("TPOL", arg);
}

std::string TxConfiguration::yieldTWDT(const int arg) const noexcept
{
  return this->prettier_->exec("TWDT", arg);
}

std::string TxConfiguration::yieldTXOF(const int arg) const noexcept
{
  return this->prettier_->exec("TXOF", arg);
}

std::string TxConfiguration::yieldZADJ(const int arg) const noexcept
{
  return this->prettier_->exec("ZADJ", arg);
}

}  // namespace mgs1600gy_interface
