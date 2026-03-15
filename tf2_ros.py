from __future__ import annotations

from typing import Iterable

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class _BaseTransformBroadcaster:
    def __init__(self, node, topic_name: str, qos_profile):
        self._publisher = node.create_publisher(TFMessage, topic_name, qos_profile)

    def sendTransform(self, transform):
        if transform is None:
            return
        transforms = transform if isinstance(transform, Iterable) and not hasattr(transform, "header") else [transform]
        msg = TFMessage()
        msg.transforms = list(transforms)
        self._publisher.publish(msg)


class TransformBroadcaster(_BaseTransformBroadcaster):
    def __init__(self, node):
        super().__init__(node, "/tf", 100)


class StaticTransformBroadcaster(_BaseTransformBroadcaster):
    def __init__(self, node):
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        super().__init__(node, "/tf_static", qos_profile)
