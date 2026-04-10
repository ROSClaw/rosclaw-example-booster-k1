#!/usr/bin/env python3

import json
import sys
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from booster_interface.srv import RpcService


@dataclass(frozen=True)
class RpcTest:
    service_name: str
    label: str
    api_id: int
    body: str


TESTS = [
    RpcTest(
        service_name="/booster_rpc_service",
        label="loco.get_mode",
        api_id=2017,
        body="",
    ),
    RpcTest(
        service_name="/booster_rpc_service",
        label="loco.get_status",
        api_id=2018,
        body="",
    ),
    RpcTest(
        service_name="/booster_rpc_service",
        label="loco.get_robot_info",
        api_id=2022,
        body="",
    ),
    RpcTest(
        service_name="/booster_vision_service",
        label="vision.get_detection_object",
        api_id=3002,
        body=json.dumps({"focus_ratio": 0.33}),
    ),
    RpcTest(
        service_name="/booster_x5_camera_service",
        label="x5.get_status",
        api_id=5002,
        body="",
    ),
    # Low-impact stop requests for services without clear getter APIs.
    RpcTest(
        service_name="/booster_rtc_service",
        label="rtc.stop_ai_chat",
        api_id=2001,
        body="",
    ),
    RpcTest(
        service_name="/booster_lui_service",
        label="lui.stop_tts",
        api_id=1051,
        body="",
    ),
    RpcTest(
        service_name="/booster_light_service",
        label="light.stop_led",
        api_id=2001,
        body="",
    ),
]


class RpcSmokeTester(Node):
    def __init__(self) -> None:
        super().__init__("rpc_service_smoke_tester")
        self._rpc_clients = {
            test.service_name: self.create_client(RpcService, test.service_name)
            for test in TESTS
        }

    def wait_for_service(self, service_name: str, timeout_sec: float = 10.0) -> bool:
        client = self._rpc_clients[service_name]
        return client.wait_for_service(timeout_sec=timeout_sec)

    def call(self, test: RpcTest, timeout_sec: float = 10.0) -> Optional[RpcService.Response]:
        request = RpcService.Request()
        request.msg.api_id = test.api_id
        request.msg.body = test.body
        future: Future = self._rpc_clients[test.service_name].call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            return None
        return future.result()


def main() -> int:
    rclpy.init()
    node = RpcSmokeTester()
    exit_code = 0
    try:
        for test in TESTS:
            print(f"=== {test.label} ({test.service_name}) ===")
            if not node.wait_for_service(test.service_name):
                print("service not available")
                exit_code = 1
                continue

            response = node.call(test)
            if response is None:
                print("timeout waiting for response")
                exit_code = 1
                continue

            print(f"status: {response.msg.status}")
            if response.msg.body:
                print(f"body: {response.msg.body}")
            else:
                print("body: <empty>")
            print()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
