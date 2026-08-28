"""Read messages out of a recorded `ros2 bag`.

Uses `rosbag2_py` + `rclpy.serialization` directly - the standard, official
way to read a bag from Python - rather than a third-party bag-reading
library. Shared by `scripts/render_bag_to_video.py` and
`scripts/extract_metrics.py` so both read bags the same way.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


@dataclass(frozen=True)
class BagMessage:
    topic: str
    timestamp_ns: int
    msg: Any


def read_messages(bag_path: str, topics: list[str] | None = None) -> Iterator[BagMessage]:
    """Yield every message in `bag_path` in storage (recorded) order.

    Each message is deserialized into its real message class - looked up
    per-topic from the bag's own recorded metadata, so this works for any
    topic without the caller having to know message types up front.
    """
    # storage_id="" lets rosbag2 read the format from the bag's own
    # metadata.yaml instead of assuming one - Jazzy's `ros2 bag record`
    # defaults to mcap, not sqlite3, and hardcoding the wrong one here
    # fails with a cryptic "file is not a database" error.
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    if topics:
        reader.set_filter(rosbag2_py.StorageFilter(topics=topics))

    msg_type_by_topic = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        msg_type = msg_type_by_topic.get(topic)
        if msg_type is None:
            continue
        yield BagMessage(
            topic=topic, timestamp_ns=timestamp_ns, msg=deserialize_message(data, msg_type)
        )
