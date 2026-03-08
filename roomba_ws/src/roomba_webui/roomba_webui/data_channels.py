"""DataChannel — Per-channel real/mock fallback logic.

Each data channel manages one stream of data with automatic fallback:
emit mock data by default; the moment a real ROS2 topic produces data,
switch to it transparently. If the topic goes silent, fall back to mock.

No mode flags, no environment variables — data source is determined
solely by whether a ROS2 topic has been heard within the channel timeout.
"""

import threading
import time
from typing import Any, Callable, Optional


class DataChannel:
    """Manages one stream of data with automatic real/mock fallback.

    Args:
        topic: ROS2 topic name this channel subscribes to.
        timeout_s: Seconds of silence before falling back to mock.
        mock_fn: Callable that returns the next mock value for this channel.
    """

    def __init__(
        self,
        topic: str,
        timeout_s: float,
        mock_fn: Callable[[], Any],
    ) -> None:
        self._topic = topic
        self._timeout_s = timeout_s
        self._mock_fn = mock_fn
        self._lock = threading.Lock()
        self._last_real_value: Optional[Any] = None
        self._last_real_timestamp: float = 0.0

    @property
    def topic(self) -> str:
        """Return the ROS2 topic name for this channel."""
        return self._topic

    def get(self) -> Any:
        """Return the most recent real value, or mock value if topic is silent.

        Returns:
            The latest real data if within timeout, otherwise mock data.
        """
        with self._lock:
            if self.is_live():
                return self._last_real_value
            return self._mock_fn()

    def is_live(self) -> bool:
        """Return True if real topic data has been received within timeout_s.

        Returns:
            True if receiving real data, False if using mock fallback.
        """
        if self._last_real_timestamp == 0.0:
            return False
        return (time.time() - self._last_real_timestamp) < self._timeout_s

    def last_seen_seconds(self) -> float:
        """Return seconds since last real message, or 999 if never received.

        Returns:
            Elapsed seconds since last real data.
        """
        if self._last_real_timestamp == 0.0:
            return 999.0
        return time.time() - self._last_real_timestamp

    def on_ros_message(self, msg: Any) -> None:
        """Called by the ROS2 subscriber callback. Updates internal state.

        Args:
            msg: The incoming ROS2 message (already converted to dict/value).
        """
        with self._lock:
            self._last_real_value = msg
            self._last_real_timestamp = time.time()
