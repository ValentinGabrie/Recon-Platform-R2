"""Unit tests for roomba_webui — data channels, mock data, bluetooth, and routes."""

import time
import pytest
from unittest.mock import patch, MagicMock

from roomba_webui.data_channels import DataChannel
from roomba_webui import mock_data
from roomba_webui.bluetooth_manager import BluetoothManager, _validate_mac


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

    def test_sensor_distances_range(self):
        """Sensor mock values are within physical range."""
        data = mock_data.mock_sensor_distances()
        assert "front" in data
        assert "left" in data
        assert "right" in data
        # Values should be positive (may go slightly outside due to sine)
        for key in ["front", "left", "right"]:
            assert -0.5 < data[key] < 5.0

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

    def test_controller_connected_is_false(self):
        """Mock controller always reports disconnected."""
        data = mock_data.mock_controller_state()
        assert data["connected"] is False

    def test_bluetooth_is_mock(self):
        """Mock Bluetooth shows 'No device (mock)'."""
        data = mock_data.mock_bluetooth_status()
        assert "mock" in data["name"].lower()
        assert data["connected"] is False


class TestBluetoothValidation:
    """Tests for MAC validation and BluetoothManager result parsing."""

    def test_valid_mac_accepted(self):
        assert _validate_mac("AA:BB:CC:DD:EE:FF") == "AA:BB:CC:DD:EE:FF"

    def test_valid_mac_lowercase(self):
        assert _validate_mac("aa:bb:cc:dd:ee:ff") == "aa:bb:cc:dd:ee:ff"

    def test_invalid_mac_rejected(self):
        with pytest.raises(ValueError):
            _validate_mac("not-a-mac")

    def test_mac_injection_rejected(self):
        """Command injection via MAC is blocked."""
        with pytest.raises(ValueError):
            _validate_mac("AA:BB:CC:DD:EE:FF; rm -rf /")

    def test_pair_invalid_mac(self):
        bm = BluetoothManager()
        with pytest.raises(ValueError):
            bm.pair("bad")

    def test_connect_invalid_mac(self):
        bm = BluetoothManager()
        with pytest.raises(ValueError):
            bm.connect("bad")

    @patch.object(BluetoothManager, "_bt_wait")
    @patch.object(BluetoothManager, "_bt_quick")
    def test_pair_success(self, mock_quick, mock_wait):
        """pair() returns success when polling detects Paired: yes."""
        mock_quick.return_value = ""  # info check — not yet paired
        mock_wait.return_value = (True, "Paired: yes")

        bm = BluetoothManager(timeout=5)
        result = bm.pair("AA:BB:CC:DD:EE:FF")
        assert result["success"]
        assert "Paired" in result["message"]

    @patch.object(BluetoothManager, "_bt_wait")
    @patch.object(BluetoothManager, "_bt_quick")
    def test_connect_failure(self, mock_quick, mock_wait):
        """connect() returns failure when polling times out."""
        mock_quick.return_value = ""  # info check — not connected
        mock_wait.return_value = (False, "")

        bm = BluetoothManager(timeout=5)
        result = bm.connect("AA:BB:CC:DD:EE:FF")
        assert not result["success"]
        assert "timed out" in result["message"]

    @patch.object(BluetoothManager, "connect")
    @patch.object(BluetoothManager, "trust")
    @patch.object(BluetoothManager, "pair")
    def test_setup_controller_sequential(self, mock_pair, mock_trust, mock_connect):
        """setup_controller calls pair → trust → connect in order."""
        mock_pair.return_value = {"success": True, "message": "Paired"}
        mock_trust.return_value = {"success": True, "message": "Trusted"}
        mock_connect.return_value = {"success": True, "message": "Connected"}

        bm = BluetoothManager()
        result = bm.setup_controller("AA:BB:CC:DD:EE:FF")
        assert result["success"]
        mock_pair.assert_called_once()
        mock_trust.assert_called_once()
        mock_connect.assert_called_once()

    @patch.object(BluetoothManager, "connect")
    @patch.object(BluetoothManager, "trust")
    @patch.object(BluetoothManager, "pair")
    def test_setup_stops_on_pair_failure(self, mock_pair, mock_trust, mock_connect):
        """setup_controller aborts if pair step fails."""
        mock_pair.return_value = {"success": False, "message": "Failed"}

        bm = BluetoothManager()
        result = bm.setup_controller("AA:BB:CC:DD:EE:FF")
        assert not result["success"]
        assert "Pair failed" in result["message"]
        mock_trust.assert_not_called()
        mock_connect.assert_not_called()

    @patch.object(BluetoothManager, "connect")
    @patch.object(BluetoothManager, "trust")
    @patch.object(BluetoothManager, "pair")
    def test_setup_retries_connect(self, mock_pair, mock_trust, mock_connect):
        """setup_controller retries connect once on first failure."""
        mock_pair.return_value = {"success": True, "message": "Paired"}
        mock_trust.return_value = {"success": True, "message": "Trusted"}
        mock_connect.side_effect = [
            {"success": False, "message": "Failed"},
            {"success": True, "message": "Connected"},
        ]

        bm = BluetoothManager()
        result = bm.setup_controller("AA:BB:CC:DD:EE:FF")
        assert result["success"]
        assert mock_connect.call_count == 2

    def test_parse_devices(self):
        """_parse_devices extracts MAC and name from bluetoothctl output."""
        output = (
            "Device AA:BB:CC:DD:EE:FF Xbox Controller\n"
            "Device 11:22:33:44:55:66 DualSense\n"
        )
        devices = BluetoothManager._parse_devices(output)
        assert len(devices) == 2
        assert devices[0]["mac"] == "AA:BB:CC:DD:EE:FF"
        assert devices[1]["name"] == "DualSense"
