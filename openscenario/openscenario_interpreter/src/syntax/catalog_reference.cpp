// Copyright 2015-2021 Tier IV, Inc. All rights reserved.
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
#include <openscenario_interpreter/syntax/catalog_reference.hpp>
#include <openscenario_interpreter/syntax/controller.hpp>
#include <openscenario_interpreter/syntax/maneuver.hpp>
#include <openscenario_interpreter/syntax/misc_object.hpp>
#include <openscenario_interpreter/syntax/pedestrian.hpp>
#include <openscenario_interpreter/syntax/vehicle.hpp>
#include "openscenario_interpreter/error.hpp"
#include "openscenario_interpreter/object.hpp"

namespace openscenario_interpreter
{
inline namespace syntax
{
#undef UNSUPPORTED_ELEMENT_SPECIFIED
#define UNSUPPORTED_ELEMENT_SPECIFIED(ELEMENT)                 \
  SyntaxError(                                                 \
    "Given class ", ELEMENT,                                   \
    " is valid OpenSCENARIO element of class CatalogRefenrece" \
    ", but is not supported yet")

Element CatalogReference::make(const pugi::xml_node & node, Scope & scope)
{
  auto catalog_name = readAttribute<std::string>("catalogName", node, scope);
  auto entry_name = readAttribute<std::string>("entryName", node, scope);

  // TODO: ParameterAssignment

  auto catalog_locations = scope.global().catalog_locations;
  if (catalog_locations) {
    for (auto & [type, catalog_location] : *catalog_locations) {
      auto found_catalog = catalog_location.find(catalog_name);
      if (found_catalog != catalog_location.end()) {
        using ::openscenario_interpreter::make;
        const auto & xml_node = found_catalog->second;
        // clang-format off
        return choice(
          xml_node,  //
          std::make_pair("Vehicle",     [&](auto && node) -> Element { return make<Vehicle>         (node, scope); }),
          std::make_pair("Controller",  [&](auto && node) -> Element { return make<Controller>      (node, scope); }),
          std::make_pair("Pedestrian",  [&](auto && node) -> Element { return make<Pedestrian>      (node, scope); }),
          std::make_pair("MiscObject",  [&](auto && node) -> Element { return make<MiscObject>      (node, scope); }),
          std::make_pair("Environment", [&](auto && node) -> Element { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified;}),
          std::make_pair("Maneuver",    [&](auto && node) -> Element { return make<Maneuver>        (node, scope); }),
          std::make_pair("Trajectory",  [&](auto && node) -> Element { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); }),
          std::make_pair("Route",       [&](auto && node) -> Element { throw UNSUPPORTED_ELEMENT_SPECIFIED(node.name()); return unspecified;})
        );
        // clang-format on
      }
    }
  }

  return Element{};
}

}  // namespace syntax
}  // namespace openscenario_interpreter
