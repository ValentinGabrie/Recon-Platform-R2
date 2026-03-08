"""BluetoothManager — wraps bluetoothctl via subprocess for the web UI.

Provides scan, pair, connect, disconnect, and trust operations.
All subprocess calls are contained here — no subprocess calls in route handlers.
"""

import logging
import subprocess
from typing import Optional

logger = logging.getLogger(__name__)


class BluetoothManager:
    """Wraps bluetoothctl commands for Bluetooth device management.

    This class is designed to be mockable for testing.
    """

    def __init__(self, timeout: int = 15) -> None:
        """Initialise BluetoothManager.

        Args:
            timeout: Default subprocess timeout in seconds.
        """
        self._timeout = timeout

    def _run_bluetoothctl(
        self, commands: list[str], timeout: Optional[int] = None
    ) -> str:
        """Execute a sequence of bluetoothctl commands.

        Args:
            commands: List of bluetoothctl commands to execute.
            timeout: Override timeout in seconds.

        Returns:
            Combined stdout from bluetoothctl.

        Raises:
            RuntimeError: If bluetoothctl fails or times out.
        """
        t = timeout or self._timeout
        input_str = "\n".join(commands) + "\nexit\n"

        try:
            result = subprocess.run(
                ["bluetoothctl"],
                input=input_str,
                capture_output=True,
                text=True,
                timeout=t,
            )
            return result.stdout
        except subprocess.TimeoutExpired:
            raise RuntimeError(
                f"bluetoothctl timed out after {t}s"
            )
        except FileNotFoundError:
            raise RuntimeError(
                "bluetoothctl not found — is bluez installed?"
            )

    def scan(self, duration_seconds: int = 10) -> list[dict]:
        """Start a Bluetooth scan and return discovered devices.

        Args:
            duration_seconds: How long to scan.

        Returns:
            List of dicts with 'name' and 'mac' keys.
        """
        import time

        try:
            # Start scan
            self._run_bluetoothctl(
                ["power on", "scan on"],
                timeout=5,
            )

            time.sleep(duration_seconds)

            # Stop scan and get devices
            output = self._run_bluetoothctl(
                ["scan off", "devices"],
                timeout=5,
            )

            return self._parse_devices(output)
        except RuntimeError as e:
            logger.error(f"Scan failed: {e}")
            return []

    def pair(self, mac: str) -> bool:
        """Pair with a Bluetooth device.

        Args:
            mac: MAC address of the device.

        Returns:
            True if pairing succeeded.
        """
        try:
            output = self._run_bluetoothctl(
                [f"pair {mac}"], timeout=30
            )
            return "Pairing successful" in output
        except RuntimeError as e:
            logger.error(f"Pair failed: {e}")
            return False

    def connect(self, mac: str) -> bool:
        """Connect to a paired Bluetooth device.

        Args:
            mac: MAC address of the device.

        Returns:
            True if connection succeeded.
        """
        try:
            output = self._run_bluetoothctl(
                [f"connect {mac}"], timeout=15
            )
            return "Connection successful" in output
        except RuntimeError as e:
            logger.error(f"Connect failed: {e}")
            return False

    def disconnect(self, mac: str) -> bool:
        """Disconnect a Bluetooth device.

        Args:
            mac: MAC address of the device.

        Returns:
            True if disconnection succeeded.
        """
        try:
            output = self._run_bluetoothctl(
                [f"disconnect {mac}"], timeout=10
            )
            return "Successful disconnected" in output
        except RuntimeError as e:
            logger.error(f"Disconnect failed: {e}")
            return False

    def trust(self, mac: str) -> bool:
        """Trust a Bluetooth device for auto-reconnection.

        Args:
            mac: MAC address of the device.

        Returns:
            True if trust succeeded.
        """
        try:
            output = self._run_bluetoothctl(
                [f"trust {mac}"], timeout=10
            )
            return "trust succeeded" in output.lower()
        except RuntimeError as e:
            logger.error(f"Trust failed: {e}")
            return False

    def list_devices(self) -> list[dict]:
        """List known/paired Bluetooth devices.

        Returns:
            List of dicts with 'name' and 'mac' keys.
        """
        try:
            output = self._run_bluetoothctl(
                ["devices"], timeout=5
            )
            return self._parse_devices(output)
        except RuntimeError as e:
            logger.error(f"List devices failed: {e}")
            return []

    def get_connection_status(self, mac: str) -> dict:
        """Get connection status for a specific device.

        Args:
            mac: MAC address of the device.

        Returns:
            Dict with 'connected', 'name', 'mac', 'battery_pct' keys.
        """
        try:
            output = self._run_bluetoothctl(
                [f"info {mac}"], timeout=5
            )
            connected = "Connected: yes" in output

            # Try to extract battery percentage
            battery_pct = None
            for line in output.splitlines():
                if "Battery Percentage" in line:
                    try:
                        battery_pct = int(
                            line.split("(")[-1].split(")")[0]
                        )
                    except (ValueError, IndexError):
                        pass

            # Extract name
            name = mac
            for line in output.splitlines():
                if "Name:" in line:
                    name = line.split("Name:")[-1].strip()
                    break

            return {
                "connected": connected,
                "name": name,
                "mac": mac,
                "battery_pct": battery_pct,
            }
        except RuntimeError as e:
            logger.error(f"Get status failed: {e}")
            return {
                "connected": False,
                "name": mac,
                "mac": mac,
                "battery_pct": None,
            }

    @staticmethod
    def _parse_devices(output: str) -> list[dict]:
        """Parse bluetoothctl 'devices' output into structured data.

        Args:
            output: Raw bluetoothctl output.

        Returns:
            List of dicts with 'name' and 'mac' keys.
        """
        devices = []
        for line in output.splitlines():
            line = line.strip()
            if line.startswith("Device "):
                parts = line.split(" ", 2)
                if len(parts) >= 3:
                    devices.append({
                        "mac": parts[1],
                        "name": parts[2],
                    })
        return devices
