"""Unit tests for recon_webui — data channels and mock data."""

import time

from recon_webui.data_channels import DataChannel
from recon_webui import mock_data


class TestDataChannel:
    """Tests for the DataChannel fallback logic."""

    def test_starts_in_mock_mode(self):
        """New channel defaults to mock data."""
        ch = DataChannel(
            topic="/test",
            timeout_s=2.0,
            mock_fn=lambda: "mock_value",
        )
        assert not ch.is_live()
        assert ch.get() == "mock_value"

    def test_promotes_to_live_on_message(self):
        """Channel switches to real data on ROS message."""
        ch = DataChannel(
            topic="/test",
            timeout_s=2.0,
            mock_fn=lambda: "mock_value",
        )
        ch.on_ros_message("real_value")
        assert ch.is_live()
        assert ch.get() == "real_value"

    def test_falls_back_to_mock_on_timeout(self):
        """Channel falls back to mock after timeout."""
        ch = DataChannel(
            topic="/test",
            timeout_s=0.1,  # Very short timeout for testing
            mock_fn=lambda: "mock_value",
        )
        ch.on_ros_message("real_value")
        assert ch.is_live()

        # Wait for timeout
        time.sleep(0.2)
        assert not ch.is_live()
        assert ch.get() == "mock_value"

    def test_last_seen_seconds(self):
        """last_seen_seconds returns 999 when never received."""
        ch = DataChannel(
            topic="/test",
            timeout_s=2.0,
            mock_fn=lambda: None,
        )
        assert ch.last_seen_seconds() == 999.0

        ch.on_ros_message("data")
        assert ch.last_seen_seconds() < 1.0


class TestMockData:
    """Tests for mock data generators."""

    def test_robot_pose_structure(self):
        """Robot pose has x, y, theta."""
        data = mock_data.mock_robot_pose()
        assert "x" in data
        assert "y" in data
        assert "theta" in data

    def test_occupancy_grid_structure(self):
        """Occupancy grid has correct dimensions."""
        data = mock_data.mock_occupancy_grid()
        assert data["width"] == 20
        assert data["height"] == 20
        assert len(data["data"]) == 400
