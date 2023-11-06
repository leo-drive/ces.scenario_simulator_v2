# Copyright 2015 TIER IV, Inc. All rights reserved.
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
import time
from pathlib import Path

import pytest
import launch_pytest

import rcl_interfaces
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

import ament_index_python.packages
import rclpy
from rclpy.node import Node
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
                namespace='test',
                parameters=[
                    {
                        "verbose": "true",
                        "scenario": os.path.join(
                            ament_index_python.packages.get_package_share_directory(
                                "openscenario_interpreter"
                            ),
                            "test/CustomCommandAction.PseudoTrafficSignalDetectorConfidenceSetAction@v1.test1.xosc",
                        ),
                    }
                ],
            )
        ]
    )


class LifecycleController(Node):
    def __init__(self):
        super().__init__('lifecycle_controller')
        # self.client = LifecycleServiceClient(node=self, lifecycle_node_name="your_lifecycle_node_name")
        self.client_change_state = self.create_client(
            ChangeState, "openscenario_interpreter/change_state"
        )

        self.latest_traffic_signal_array_msg = None
        self.traffic_light_subscriber = self.create_subscription(
            TrafficSignalArray,
            '/perception/traffic_light_recognition/traffic_signals',
            self.traffic_light_callback,
            1
        )

    def change_state(self, transition_id):

        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.client_change_state.call_async(request)

        # future = self.client_change_state.call_async(ChangeState.Request(transition_id=transition_id))
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().success
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())
            return False

    def activate(self):
        return self.change_state(Transition.TRANSITION_ACTIVATE)

    def deactivate(self):
        return self.change_state(Transition.TRANSITION_DEACTIVATE)

    def shutdown(self):
        return self.change_state(Transition.TRANSITION_CLEANUP)

    def configure(self):
        return self.change_state(Transition.TRANSITION_CONFIGURE)

    def receive_traffic_signal(self, traffic_signal_id, timeout_sec=90.0):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.latest_traffic_signal_array_msg is not None:
                for signal in self.latest_traffic_signal_array_msg.signals:
                    if signal.traffic_signal_id == traffic_signal_id:
                        return signal
            print('spin!')
            self.executor.spin_once(timeout_sec=0.1)

    def traffic_light_callback(self, msg):
        print('traffic_light_callback')
        self.latest_traffic_signal_array_msg = msg

# def main(args=None):

class DummyTestNode(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('dummy_test_node')

        self.latest_traffic_signal_array_msg = None
        self.traffic_light_subscriber = self.create_subscription(
            TrafficSignalArray,
            '/perception/traffic_light_recognition/traffic_signals',
            self.traffic_light_callback,
            1
        )

        self.client_get_state = self.create_client(
            GetState, "openscenario_interpreter/get_state"
        )

        self.client_change_state = self.create_client(
            ChangeState, "openscenario_interpreter/change_state"
        )

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self)
        print('set up executor')

        while not self.client_get_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                self.client_get_state.srv_name + " service unavailable"
            )

        while not self.client_change_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                self.client_change_state.srv_name + " service unavailable"
            )

        # configure
        assert self.transition(Transition.TRANSITION_CONFIGURE)
        print('configured')
        assert False
        # activate
        assert self.transition(Transition.TRANSITION_ACTIVATE)
        print('activated')
        # deactivate
        # self.transition(Transition.TRANSITION_DEACTIVATE)
        # # cleanup
        # self.transition(Transition.TRANSITION_CLEANUP)
        # # shutdown
        # self.transition(Transition.TRANSITION_SHUTDOWN)

    def transition(self, transition_id):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.client_change_state.call_async(request)
        rclpy.spin_until_future_complete(self, future, executor=self.executor, timeout_sec=2.0)
        if future.result() is None:
            raise RuntimeError("Interpreter tried to set current lifecycle state, but failed.")
        return future.result().success

    def traffic_light_callback(self, msg):
        print('traffic_light_callback')
        self.latest_traffic_signal_array_msg = msg

    def receive_traffic_signal(self, traffic_signal_id, timeout_sec=90.0):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.latest_traffic_signal_array_msg is not None:
                for signal in self.latest_traffic_signal_array_msg.signals:
                    if signal.traffic_signal_id == traffic_signal_id:
                        return signal
            print('spin!')
            self.executor.spin_once(timeout_sec=0.1)


@pytest.mark.launch(fixture=generate_test_description)
def test_confidence():

    print('test_confidence')
    # node = DummyTestNode()
    # node = LifecycleController()

    rclpy.init()
    node = LifecycleController()

    print('set up node')

    # ここで状態遷移を実行
    if node.configure():
        node.get_logger().info("Configured successfully")

    if node.activate():
        node.get_logger().info("Activated successfully")

    # 他の状態遷移も同様に行うことができます

    # rclpy.spin(controller)
    # way_id: 34802 -> regulatory_element_id: 34806
    traffic_signal = node.receive_traffic_signal(34806, 30.0)
    assert traffic_signal is not None
    for element in traffic_signal.elements:
        assert element.confidence == 0.5
    rclpy.shutdown()

