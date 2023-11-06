#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import rclpy
import time

import pytest
import launch_pytest

from argparse import ArgumentParser
from glob import glob
from scenario_test_runner.lifecycle_controller import LifecycleController
from pathlib import Path
from rclpy.executors import ExternalShutdownException
from shutil import rmtree
from sys import exit
import ament_index_python.packages
import launch
import launch_ros.actions
from autoware_perception_msgs.msg import TrafficSignalArray


@launch_pytest.fixture
def generate_test_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.LifecycleNode(
                package="openscenario_interpreter",
                executable="openscenario_interpreter_node",
                name="openscenario_interpreter",
                output="screen",
                namespace='simulation',
                parameters=[
                    {
                        "architecture_type": "awf/universe/20230906",
                        "record": False,
                        "local_frame_rate": 30.0,
                        "local_real_time_factor": 1.0,
                        "output_directory": "/tmp",
                    }
                ],
            ),
            launch_ros.actions.Node(
                package="simple_sensor_simulator",
                executable="simple_sensor_simulator_node",
                namespace="simulation",
                name="simple_sensor_simulator",
                output="screen",
            ),
        ]
    )


class TestRunner(LifecycleController):
    SLEEP_RATE = 1

    def __init__(self):
        super().__init__()
        self.executor = rclpy.executors.MultiThreadedExecutor(context=self.context)

        self.output_directory = "/tmp"

        self.latest_traffic_signal_array_msg = None

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.traffic_light_subscriber = self.create_subscription(
            TrafficSignalArray,
            '/perception/traffic_light_recognition/traffic_signals',
            self.traffic_light_callback,
            1,
            callback_group=self.callback_group
        )

        # self.executor = rclpy.executors.MultiThreadedExecutor()

    def traffic_light_callback(self, traffic_signal_array_msg):
        self.latest_traffic_signal_array_msg = traffic_signal_array_msg

    # def receive_traffic_signal(self):
    #     time.sleep(self.SLEEP_RATE)
    #     while self.activate_node():
    #         start = time.time()
    #         while rclpy.ok():
    #             if self.get_lifecycle_state() == "inactive":
    #                 break
    #             elif ((time.time() - start) > self.global_timeout
    #             if self.global_timeout is not None else False):
    #                 self.get_logger().error("The simulation has timed out. Forcibly inactivate.")
    #                 self.deactivate_node()
    #                 break
    #             else:
    #                 time.sleep(self.SLEEP_RATE)
    def receive_traffic_signal(self, traffic_signal_id, timeout_sec=10.0):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.latest_traffic_signal_array_msg is not None:
                for signal in self.latest_traffic_signal_array_msg.signals:
                    if signal.traffic_signal_id == traffic_signal_id:
                        return signal
            print('spin!')
            self.executor.spin_once(timeout_sec=0.1)

    def run_test_scenario(self, scenario_path):
        self.configure_node(
            frame_rate=30.0,
            output_directory='/tmp',
            real_time_factor=1.0,
            scenario=scenario_path,
        )

        assert self.get_lifecycle_state() == 'inactive'

        assert self.activate_node()

        assert self.get_lifecycle_state() == 'active'

        # way_id: 34802 -> regulatory_element_id: 34806
        traffic_signal = self.receive_traffic_signal(34806)
        assert traffic_signal is not None
        for element in traffic_signal.elements:
            assert element.confidence == 0.5
        self.cleanup_node()
        self.shutdown()
        self.destroy_node()

    def print_debug(self, message: str):
        self.get_logger().info(message)


@pytest.mark.launch(fixture=generate_test_description)
def test(args=None):
    print('test_confidence')
    rclpy.init(args=args)

    test_runner = TestRunner()

    test_runner.run_test_scenario(
        "/home/hans/workspace/rtc_xx1/src/simulator/scenario_simulator/openscenario/openscenario_interpreter/test/CustomCommandAction.PseudoTrafficSignalDetectorConfidenceSetAction@v1.test1.xosc")

# if __name__ == "__main__":
#     """Entrypoint."""
#     main()
