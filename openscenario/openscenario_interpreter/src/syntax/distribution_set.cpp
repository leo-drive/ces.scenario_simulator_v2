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
#include <openscenario_interpreter/syntax/distribution_set.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
DistributionSet::DistributionSet(const pugi::xml_node & node, Scope & scope)
: Scope(scope), elements(readElements<DistributionSetElement, 1>("Element", node, local()))
{
}

auto DistributionSet::derive() -> SingleUnnamedParameterDistribution
{
  SingleUnnamedParameterDistribution distribution;
  for (const auto & element : elements) {
    distribution.emplace_back(make<String>(element.value));
  }
  return distribution;
}

auto DistributionSet::derive(
  std::size_t local_index, std::size_t local_size, std::size_t global_index,
  std::size_t global_size) -> ParameterList
{
  return ParameterList({{"", make<String>(std::next(elements.begin(), local_index)->value)}});
}

auto DistributionSet::getNumberOfDeriveScenarios() const -> std::size_t
{
  return std::size(elements);
}
}  // namespace syntax
}  // namespace openscenario_interpreter
