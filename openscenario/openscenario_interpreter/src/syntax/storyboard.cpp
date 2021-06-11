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

#include <iomanip>
#include <openscenario_interpreter/syntax/storyboard.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::ostream & operator<<(std::ostream & os, const Storyboard & datum)
{
  nlohmann::json json;

  return os << (json << datum).dump(2);
}

nlohmann::json & operator<<(nlohmann::json & json, const Storyboard & datum)
{
  json["state"] = boost::lexical_cast<std::string>(datum.state());

  json["Story"] = nlohmann::json::array();

  for (const auto & each : datum) {
    nlohmann::json story;
    json["Story"].push_back(story);
  }

  return json;
}
}  // namespace syntax
}  // namespace openscenario_interpreter
