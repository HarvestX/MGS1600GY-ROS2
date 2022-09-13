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

namespace mgs1600gy_interface
{

class PortHandlerBase
{
public:
  PortHandlerBase() {}

  virtual size_t getBytesAvailable() const = 0;
  virtual size_t readPort(char * const, const size_t) const = 0;
  virtual size_t writePort(const char * const, const size_t) const = 0;
};
}  // namespace mgs1600gy_interface
