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

#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/stochastic_distribution.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
StochasticDistribution::StochasticDistribution(const pugi::xml_node & node, Scope & scope)
: StochasticDistributionType(node, scope),
  parameter_name(readAttribute<String>("parameterName", node, scope))
{
}

auto StochasticDistribution::derive() -> Object
{
  return apply<Object>(
    [](auto & unnamed_distribution) { return unnamed_distribution.derive(); }, *this);
}

auto StochasticDistribution::derive(
  std::size_t local_index, std::size_t local_size, std::size_t global_index,
  std::size_t global_size) -> ParameterList
{
  return {
    {parameter_name, make(apply<ParameterList>(
                            [&](auto & unnamed_distribution) {
                              return unnamed_distribution.derive(
                                local_index, local_size, global_index, global_size);
                            },
                            (StochasticDistributionType &)*this)
                            .begin()
                            ->second)}};
}
}  // namespace syntax
}  // namespace openscenario_interpreter
