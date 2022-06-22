// Copyright 2015-2022 Tier IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__TYPE_TRAITS__REQUIRES_HPP_
#define OPENSCENARIO_INTERPRETER__TYPE_TRAITS__REQUIRES_HPP_

#include <type_traits>

#define REQUIRES(...) typename = typename std::enable_if<std::conjunction<__VA_ARGS__>::value>::type

#endif  // OPENSCENARIO_INTERPRETER__TYPE_TRAITS__REQUIRES_HPP_
