// Copyright 2015 TIER IV, Inc. All rights reserved.
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

#ifndef OPENSCENARIO_INTERPRETER__TYPE_TRAITS__IS_OPTIONAL_HPP_
#define OPENSCENARIO_INTERPRETER__TYPE_TRAITS__IS_OPTIONAL_HPP_

#include <optional>
#include <type_traits>

namespace openscenario_interpreter
{
inline namespace type_traits
{
template <typename T, typename = void>
struct is_optional : std::false_type
{
};

template <typename U>
struct is_optional<std::optional<U>> : std::true_type
{
};

template <typename T>
static constexpr auto is_optional_v = is_optional<T>::value;
}  // namespace type_traits
}  // namespace openscenario_interpreter

#endif
