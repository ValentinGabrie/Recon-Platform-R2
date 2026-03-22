"""BluetoothManager — wraps bluetoothctl via subprocess for the web UI.

Provides scan, pair, connect, disconnect, trust, and remove operations.
All subprocess calls are contained here — no subprocess calls in route handlers.
"""

import logging
import re
import subprocess
import time
from typing import Optional

logger = logging.getLogger(__name__)

# Strict MAC address pattern — prevents command injection via stdin
_MAC_RE = re.compile(r"^([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}$")


def _validate_mac(mac: str) -> str:
    """Validate and return a MAC address, or raise ValueError."""
    mac = mac.strip()
    if not _MAC_RE.match(mac):
        raise ValueError(f"Invalid MAC address: {mac!r}")
    return mac


class BluetoothManager:
    """Wraps bluetoothctl commands for Bluetooth device management.

    This class is designed to be mockable for testing.
    """

    def __init__(self, timeout: int = 15) -> None:
        self._timeout = timeout

    def _run_bluetoothctl(
        self, commands: list[str], timeout: Optional[int] = None
    ) -> str:
        """Execute a sequence of bluetoothctl commands in a single session.

        Returns combined stdout from bluetoothctl.
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
            logger.debug("bluetoothctl commands=%s output_lines=%d", commands, len(result.stdout.splitlines()))
            return result.stdout
        except subprocess.TimeoutExpired:
            raise RuntimeError(f"bluetoothctl timed out after {t}s")
        except FileNotFoundError:
            raise RuntimeError("bluetoothctl not found — is bluez installed?")

    def scan(self, duration_seconds: int = 10) -> list[dict]:
        """Start a Bluetooth scan and return discovered devices.

        Runs the entire scan lifecycle in a single bluetoothctl session so
        discovered (unpaired) devices are visible in the output.
        """
        logger.info("BT scan starting — duration=%ds", duration_seconds)
        try:
            # Single process: power on → agent on → scan on → wait → scan off → devices
            # 'devices' in the same session includes newly discovered devices.
            # We use a generous timeout: startup + scan duration + teardown.
            timeout = duration_seconds + 10
            input_commands = [
                "power on",
                "agent on",
                "default-agent",
                "scan on",
            ]
            input_str = "\n".join(input_commands) + "\n"

            proc = subprocess.Popen(
                ["bluetoothctl"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )

            # Send initial commands
            proc.stdin.write(input_str)
            proc.stdin.flush()

            # Wait for scan duration
            time.sleep(duration_seconds)

            # Stop scan and list all discovered devices, then exit
            proc.stdin.write("scan off\ndevices\nexit\n")
            proc.stdin.flush()

            stdout, _ = proc.communicate(timeout=10)
            devices = self._parse_devices(stdout)
            logger.info("BT scan complete — found %d devices", len(devices))
            return devices
        except subprocess.TimeoutExpired:
            proc.kill()
            logger.error("Scan timed out")
            return []
        except (RuntimeError, FileNotFoundError) as e:
            logger.error("Scan failed: %s", e)
            return []

    def pair(self, mac: str) -> dict:
        """Pair with a Bluetooth device.

        Returns dict with 'success' bool and 'message' string.
        """
        mac = _validate_mac(mac)
        logger.info("BT pair — mac=%s", mac)
        try:
            output = self._run_bluetoothctl(
                ["power on", "agent on", "default-agent", f"pair {mac}"],
                timeout=30,
            )
            if "Pairing successful" in output or "Already Paired" in output:
                return {"success": True, "message": "Paired"}
            # Xbox controllers may connect without formal pairing
            if "Connected: yes" in output:
                return {"success": True, "message": "Already connected"}
            for line in output.splitlines():
                if "Failed to pair" in line or "Error" in line:
                    return {"success": False, "message": line.strip()}
            return {"success": False, "message": "Pairing did not succeed"}
        except ValueError as e:
            return {"success": False, "message": str(e)}
        except RuntimeError as e:
            logger.error("Pair failed: %s", e)
            return {"success": False, "message": str(e)}

    def connect(self, mac: str) -> dict:
        """Connect to a paired Bluetooth device.

        Returns dict with 'success' bool and 'message' string.
        """
        mac = _validate_mac(mac)
        logger.info("BT connect — mac=%s", mac)
        try:
            output = self._run_bluetoothctl(
                ["power on", f"connect {mac}"], timeout=15
            )
            if "Connection successful" in output:
                return {"success": True, "message": "Connected"}
            # Already connected is a success
            if "Connected: yes" in output:
                return {"success": True, "message": "Already connected"}
            for line in output.splitlines():
                if "Failed to connect" in line or "Error" in line:
                    return {"success": False, "message": line.strip()}
            return {"success": False, "message": "Connection did not succeed"}
        except ValueError as e:
            return {"success": False, "message": str(e)}
        except RuntimeError as e:
            logger.error("Connect failed: %s", e)
            return {"success": False, "message": str(e)}

    def disconnect(self, mac: str) -> dict:
        """Disconnect a Bluetooth device.

        Returns dict with 'success' bool and 'message' string.
        """
        mac = _validate_mac(mac)
        logger.info("BT disconnect — mac=%s", mac)
        try:
            output = self._run_bluetoothctl(
                [f"disconnect {mac}"], timeout=10
            )
            if "Successful disconnected" in output:
                return {"success": True, "message": "Disconnected"}
            return {"success": False, "message": "Disconnect did not succeed"}
        except ValueError as e:
            return {"success": False, "message": str(e)}
        except RuntimeError as e:
            logger.error("Disconnect failed: %s", e)
            return {"success": False, "message": str(e)}

    def trust(self, mac: str) -> dict:
        """Trust a Bluetooth device for auto-reconnection.

        Returns dict with 'success' bool and 'message' string.
        """
        mac = _validate_mac(mac)
        logger.info("BT trust — mac=%s", mac)
        try:
            output = self._run_bluetoothctl(
                ["power on", f"trust {mac}"], timeout=10
            )
            if "trust succeeded" in output.lower():
                return {"success": True, "message": "Trusted"}
            return {"success": False, "message": "Trust did not succeed"}
        except ValueError as e:
            return {"success": False, "message": str(e)}
        except RuntimeError as e:
            logger.error("Trust failed: %s", e)
            return {"success": False, "message": str(e)}

    def remove(self, mac: str) -> dict:
        """Remove a Bluetooth device (unpair and forget).

        Returns dict with 'success' bool and 'message' string.
        """
        mac = _validate_mac(mac)
        logger.info("BT remove — mac=%s", mac)
        try:
            output = self._run_bluetoothctl(
                [f"remove {mac}"], timeout=10
            )
            if "Device has been removed" in output:
                return {"success": True, "message": "Removed"}
            return {"success": False, "message": "Remove did not succeed"}
        except ValueError as e:
            return {"success": False, "message": str(e)}
        except RuntimeError as e:
            logger.error("Remove failed: %s", e)
            return {"success": False, "message": str(e)}

    def setup_controller(self, mac: str) -> dict:
        """One-shot pair + trust + connect for a new controller.

        Returns dict with 'success' bool and 'message' string describing
        the step that failed (if any).
        """
        mac = _validate_mac(mac)
        logger.info("BT setup_controller — mac=%s", mac)
        try:
            output = self._run_bluetoothctl(
                [
                    "power on",
                    "agent on",
                    "default-agent",
                    f"pair {mac}",
                    f"trust {mac}",
                    f"connect {mac}",
                ],
                timeout=45,
            )
            connected = "Connection successful" in output or "Connected: yes" in output
            paired = "Pairing successful" in output or "Already Paired" in output
            trusted = "trust succeeded" in output.lower()

            if connected:
                return {"success": True, "message": "Paired, trusted, and connected"}
            if not paired:
                for line in output.splitlines():
                    if "Failed to pair" in line:
                        return {"success": False, "message": line.strip()}
                return {"success": False, "message": "Pairing did not succeed"}
            if not trusted:
                return {"success": False, "message": "Trust failed after pairing"}
            # Paired and trusted but connect failed
            for line in output.splitlines():
                if "Failed to connect" in line:
                    return {"success": False, "message": line.strip()}
            return {"success": False, "message": "Connection failed after pair+trust"}
        except ValueError as e:
            return {"success": False, "message": str(e)}
        except RuntimeError as e:
            logger.error("Setup controller failed: %s", e)
            return {"success": False, "message": str(e)}

    def list_devices(self) -> list[dict]:
        """List Bluetooth devices that are paired or connected.

        Uses 'devices' then filters via 'info' to check Paired or Connected,
        for compatibility with bluez < 5.77 (which lacks 'devices Paired').
        Xbox controllers may be Connected without being formally Paired.
        """
        try:
            output = self._run_bluetoothctl(["devices"], timeout=5)
            all_devices = self._parse_devices(output)
            if not all_devices:
                return []

            # Batch info queries in one bluetoothctl session
            cmds = [f"info {d['mac']}" for d in all_devices]
            info_output = self._run_bluetoothctl(cmds, timeout=10)

            # Parse which MACs have Paired: yes OR Connected: yes
            relevant_macs: set[str] = set()
            current_mac = None
            for line in info_output.splitlines():
                stripped = line.strip()
                if stripped.startswith("Device "):
                    parts = stripped.split(None, 2)
                    if len(parts) >= 2:
                        current_mac = parts[1]
                elif current_mac and (
                    "Paired: yes" in stripped or "Connected: yes" in stripped
                ):
                    relevant_macs.add(current_mac)

            return [d for d in all_devices if d["mac"] in relevant_macs]
        except RuntimeError as e:
            logger.error("List devices failed: %s", e)
            return []

    def get_connection_status(self, mac: str) -> dict:
        """Get connection status for a specific device."""
        mac = _validate_mac(mac)
        try:
            output = self._run_bluetoothctl(
                [f"info {mac}"], timeout=5
            )
            connected = "Connected: yes" in output
            logger.debug("BT status — mac=%s connected=%s", mac, connected)

            battery_pct = None
            for line in output.splitlines():
                if "Battery Percentage" in line:
                    try:
                        battery_pct = int(line.split("(")[-1].split(")")[0])
                    except (ValueError, IndexError):
                        pass

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
        except (ValueError, RuntimeError) as e:
            logger.error("Get status failed: %s", e)
            return {
                "connected": False,
                "name": mac,
                "mac": mac,
                "battery_pct": None,
            }

    @staticmethod
    def _parse_devices(output: str) -> list[dict]:
        """Parse bluetoothctl 'devices' output into structured data."""
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
