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

#include <openscenario_interpreter/syntax/entity_ref.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <openscenario_interpreter/syntax/by_entity_condition.hpp>
#include <openscenario_interpreter/syntax/entity_condition.hpp>
#include <openscenario_interpreter/reader/element.hpp>
#include <openscenario_interpreter/syntax/trigger.hpp>
#include <iterator>
#include <list>
#include <vector>

namespace openscenario_interpreter
{
inline namespace syntax
{
Trigger::Trigger(const pugi::xml_node & node, Scope & scope)
{
  traverse<0, unbounded>(node, "ConditionGroup", [&](auto && node) { emplace_back(node, scope); });
}

auto Trigger::evaluate() -> Object
{
  /* -------------------------------------------------------------------------
   *
   *  A trigger is then defined as an association of condition groups. A
   *  trigger evaluates to true if at least one of the associated condition
   *  groups evaluates to true, otherwise it evaluates to false (OR
   *  operation).
   *
   * ---------------------------------------------------------------------- */
  // NOTE: Don't use std::any_of; Intentionally does not short-circuit evaluation.
  return asBoolean(
    current_value = std::accumulate(
      std::begin(*this), std::end(*this), false,
      [&](auto && lhs, ConditionGroup & condition_group) {
        const auto rhs = condition_group.evaluate();
        return lhs or rhs.as<Boolean>();
      }));
}

auto Trigger::triggererEntities() const -> std::list<EntityRef>
{
  auto triggerer_entities = std::list<EntityRef>{};
  for (const auto & condition_group : *this) {
    if (condition_group.current_value) {
      for (const auto & condition : condition_group) {
        if (condition.current_value and condition.is<ByEntityCondition>()) {
          apply<void>([&](const auto &entity_condition) {
            const auto &entity_refs = entity_condition.triggering_entities.entity_refs;
            triggerer_entities.insert(std::end(triggerer_entities), std::begin(entity_refs), std::end(entity_refs));
          }, condition.as<EntityCondition>());
        }
      }
    }
  }
  triggerer_entities.sort();
  triggerer_entities.unique();
  return triggerer_entities;
}

auto Trigger::activeConditionGroupIndex() const -> iterator::difference_type
{
  return std::distance(begin(), std::find_if(begin(), end(), [](const auto & group) {
                         return std::all_of(group.begin(), group.end(), [](const auto & condition) {
                           return condition.current_value;
                         });
                       }));
}

auto Trigger::activeConditionGroupDescription() const
  -> std::vector<std::pair<std::string, std::string>>
{
  const auto & group = *std::next(begin(), activeConditionGroupIndex());
  std::vector<std::pair<std::string, std::string>> name_description_vec;
  for (const auto & condition : group)
    name_description_vec.emplace_back(condition.name, condition.description());
  return name_description_vec;
}

auto operator<<(nlohmann::json & json, const Trigger & datum) -> nlohmann::json &
{
  json["currentValue"] = boost::lexical_cast<std::string>(Boolean(datum.current_value));

  json["ConditionGroup"] = nlohmann::json::array();

  for (const auto & each : datum) {
    nlohmann::json condition_group;
    condition_group << each;
    json["ConditionGroup"].push_back(condition_group);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
