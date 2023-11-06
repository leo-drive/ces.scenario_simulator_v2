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

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_perception_msgs/msg/traffic_signal_array.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <openscenario_interpreter/openscenario_interpreter.hpp>
#include <openscenario_interpreter/syntax/open_scenario.hpp>
#include <openscenario_interpreter/syntax/scenario_definition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

const std::string scenario = R"###(
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="0" date="2023-10-30T08:31:04.600Z" description="scenario for testing PseudoTrafficSignalDetectorConfidenceSetAction@v1 in openscenario_interpreter." author="Kotaro Yoshimoto" />
  <CatalogLocations />
  <RoadNetwork>
    <LogicFile filepath="$(find-pkg-share kashiwanoha_map)/map" />
    <TrafficSignals>
      <TrafficSignalController name="controller">
        <Phase name="phase-1" duration="5">
          <TrafficSignalState trafficSignalId="34802" state="green" />
        </Phase>
      </TrafficSignalController>
    </TrafficSignals>
  </RoadNetwork>
  <Entities />
  <Storyboard>
    <Init>
      <Actions>
        <UserDefinedAction>
          <CustomCommandAction type="PseudoTrafficSignalDetectorConfidenceSetAction@v1(34802, 0.5)" />
        </UserDefinedAction>
      </Actions>
    </Init>
    <StopTrigger />
  </Storyboard>
</OpenSCENARIO>
)###";

TEST(TrafficLightConfidenceChecker, test_traffic_light_confidence_checker)
{
  using namespace openscenario_interpreter;

  OpenScenario open_scenario{
    ament_index_cpp::get_package_share_directory("openscenario_interpreter") +
    "/test/CustomCommandAction.PseudoTrafficSignalDetectorConfidenceSetAction@v1.test1.xosc"};

  std::string osc_path =
    ament_index_cpp::get_package_share_directory("openscenario_interpreter") +
    "/test/CustomCommandAction.PseudoTrafficSignalDetectorConfidenceSetAction@v1.test1.xosc";
  Interpreter interpreter{rclcpp::NodeOptions{}};
  interpreter.set_parameter({"osc_path", osc_path});
  interpreter.configure();
  interpreter.activate();
  SimulatorCore::NonStandardOperation::getConventionalTrafficLights() EXPECT_TRUE(true);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
