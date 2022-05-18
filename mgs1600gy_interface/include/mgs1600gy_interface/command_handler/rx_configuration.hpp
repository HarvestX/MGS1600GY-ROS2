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
class RxConfiguration
{
private:
  const std::string prefix_ = "~";
  const std::unique_ptr<const Prettier> prettier_;

public:
  RxConfiguration();

  std::string yieldANAM() const noexcept;
  std::string yieldBADJ() const noexcept;
  std::string yieldBRUN() const noexcept;
  std::string yieldDIM() const noexcept;
  std::string yieldFCAL() const noexcept;
  std::string yieldGRNG() const noexcept;
  std::string yieldMMOD() const noexcept;
  std::string yieldPWMM() const noexcept;
  std::string yieldRSBR() const noexcept;
  std::string yieldSCRO() const noexcept;
  std::string yieldTINV() const noexcept;
  std::string yieldTMS() const noexcept;
  std::string yieldTPOL() const noexcept;
  std::string yieldTWDT() const noexcept;
  std::string yieldTXOF() const noexcept;
  std::string yieldZADJ() const noexcept;
};
}  // namespace mgs1600gy_interface
