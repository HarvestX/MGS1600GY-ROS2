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

#include "mgs1600gy_interface/command_handler/prettier.hpp"

namespace mgs1600gy_interface
{
Prettier::Prettier(
  const std::string & prefix
)
: prefix_(prefix),
  suffix_("\r")
{
}

std::string Prettier::exec(
  const std::string && command)
const noexcept
{
  return this->prefix_ + command + this->suffix_;
}

std::string Prettier::exec(
  const std::string && command,
  const int arg) const noexcept
{
  return this->exec(command + " " + std::to_string(arg));
}

std::string Prettier::exec(
  const std::string && command,
  const int idx,
  const int arg
) const noexcept
{
  return this->exec(command + " " + std::to_string(idx), arg);
}
}  // namespace mgs1600gy_interface
