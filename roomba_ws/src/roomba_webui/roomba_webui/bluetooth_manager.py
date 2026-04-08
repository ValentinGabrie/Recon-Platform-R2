"""BluetoothManager — wraps bluetoothctl via subprocess for the web UI.

Provides scan, pair, connect, disconnect, trust, remove, and setup operations.
All subprocess calls are contained here — no subprocess calls in route handlers.

Design
------
- Quick operations (info, disconnect, remove, list) use ``_bt_quick()`` — a
  batch ``subprocess.run()`` call that sends commands + ``exit`` and waits.
- Async operations (pair, trust, connect) use ``_bt_wait()`` — starts a
  long-lived ``Popen`` session so the BT handshake can complete, while
  polling ``info {mac}`` in separate short-lived sessions until the
  expected state appears or timeout is reached.
"""

import glob
import logging
import re
import subprocess
import time
from typing import Callable, Optional

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
    """Wraps bluetoothctl for Bluetooth device management."""

    def __init__(self, timeout: int = 15) -> None:
        self._timeout = timeout

    # -----------------------------------------------------------------
    # Low-level helpers
    # -----------------------------------------------------------------

    def _bt_quick(
        self, commands: list[str], timeout: Optional[int] = None
    ) -> str:
        """Run bluetoothctl commands in a batch session (appends ``exit``).

        Use for fast / synchronous operations: info, devices, disconnect,
        remove.  Returns combined stdout.
        """
        t = timeout or self._timeout
        stdin = "\n".join(commands) + "\nexit\n"
        try:
            result = subprocess.run(
                ["bluetoothctl"],
                input=stdin,
                capture_output=True,
                text=True,
                timeout=t,
            )
            logger.debug(
                "_bt_quick commands=%s rc=%d lines=%d",
                commands, result.returncode, len(result.stdout.splitlines()),
            )
            return result.stdout
        except subprocess.TimeoutExpired:
            raise RuntimeError(f"bluetoothctl timed out after {t}s")
        except FileNotFoundError:
            raise RuntimeError("bluetoothctl not found — is bluez installed?")

    def _bt_wait(
        self,
        commands: list[str],
        mac: str,
        check_fn: Callable[[str], bool],
        timeout: Optional[int] = None,
    ) -> tuple[bool, str]:
        """Start a long-lived bluetoothctl session and poll until *check_fn* passes.

        Keeps the main process alive (so the BT handshake can finish) while
        polling ``info {mac}`` in separate short-lived sessions every second.

        Returns ``(success, last_info_output)``.
        """
        t = timeout or self._timeout
        try:
            proc = subprocess.Popen(
                ["bluetoothctl"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except FileNotFoundError:
            raise RuntimeError("bluetoothctl not found — is bluez installed?")

        try:
            proc.stdin.write("\n".join(commands) + "\n")
            proc.stdin.flush()
        except (OSError, BrokenPipeError):
            proc.kill()
            proc.wait()
            return False, ""

        last_info = ""
        try:
            deadline = time.monotonic() + t
            while time.monotonic() < deadline:
                time.sleep(1)
                try:
                    last_info = self._bt_quick([f"info {mac}"], timeout=5)
                    if check_fn(last_info):
                        return True, last_info
                except RuntimeError:
                    continue
            return False, last_info
        finally:
            self._cleanup_proc(proc)

    @staticmethod
    def _cleanup_proc(proc: subprocess.Popen) -> None:
        """Cleanly shut down a bluetoothctl process."""
        try:
            proc.stdin.write("exit\n")
            proc.stdin.flush()
        except (OSError, BrokenPipeError):
            pass
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    def _get_info(self, mac: str) -> str:
        """Return ``info`` output for *mac*, or empty string on error."""
        try:
            return self._bt_quick([f"info {mac}"], timeout=5)
        except RuntimeError:
            return ""

    # -----------------------------------------------------------------
    # Public operations
    # -----------------------------------------------------------------

    def scan(self, duration_seconds: int = 10) -> list[dict]:
        """Run a Bluetooth scan and return discovered devices."""
        logger.info("BT scan starting — duration=%ds", duration_seconds)
        try:
            proc = subprocess.Popen(
                ["bluetoothctl"],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            proc.stdin.write("power on\nagent on\ndefault-agent\nscan on\n")
            proc.stdin.flush()

            time.sleep(duration_seconds)

            proc.stdin.write("scan off\ndevices\nexit\n")
            proc.stdin.flush()

            stdout, _ = proc.communicate(timeout=10)
            devices = self._parse_devices(stdout)
            logger.info("BT scan complete — found %d devices", len(devices))
            return devices
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            logger.error("Scan timed out")
            return []
        except FileNotFoundError:
            logger.error("bluetoothctl not found")
            return []

    def pair(self, mac: str) -> dict:
        """Pair with a Bluetooth device.

        Only succeeds when ``Paired: yes`` appears in device info — a mere
        ``Connected: yes`` without pairing is *not* accepted (an Xbox
        controller in that state has no HID input device).
        """
        mac = _validate_mac(mac)
        logger.info("BT pair — mac=%s", mac)

        info = self._get_info(mac)
        if "Paired: yes" in info:
            logger.info("BT pair — already paired")
            return {"success": True, "message": "Already paired"}

        ok, _ = self._bt_wait(
            ["power on", "agent on", "default-agent", f"pair {mac}"],
            mac,
            lambda i: "Paired: yes" in i,
            timeout=30,
        )
        if ok:
            logger.info("BT pair succeeded — mac=%s", mac)
            return {"success": True, "message": "Paired"}
        logger.warning("BT pair timed out — mac=%s", mac)
        return {"success": False, "message": "Pairing timed out"}

    def trust(self, mac: str) -> dict:
        """Trust a Bluetooth device for auto-reconnection."""
        mac = _validate_mac(mac)
        logger.info("BT trust — mac=%s", mac)

        info = self._get_info(mac)
        if "Trusted: yes" in info:
            logger.info("BT trust — already trusted")
            return {"success": True, "message": "Already trusted"}

        ok, _ = self._bt_wait(
            [f"trust {mac}"],
            mac,
            lambda i: "Trusted: yes" in i,
            timeout=10,
        )
        if ok:
            logger.info("BT trust succeeded — mac=%s", mac)
            return {"success": True, "message": "Trusted"}
        logger.warning("BT trust failed — mac=%s", mac)
        return {"success": False, "message": "Trust did not succeed"}

    def connect(self, mac: str) -> dict:
        """Connect to a paired Bluetooth device."""
        mac = _validate_mac(mac)
        logger.info("BT connect — mac=%s", mac)

        info = self._get_info(mac)
        if "Connected: yes" in info:
            logger.info("BT connect — already connected")
            return {"success": True, "message": "Already connected"}

        ok, _ = self._bt_wait(
            ["power on", f"connect {mac}"],
            mac,
            lambda i: "Connected: yes" in i,
            timeout=15,
        )
        if ok:
            logger.info("BT connect succeeded — mac=%s", mac)
            return {"success": True, "message": "Connected"}
        logger.warning("BT connect timed out — mac=%s", mac)
        return {"success": False, "message": "Connection timed out"}

    def disconnect(self, mac: str) -> dict:
        """Disconnect a Bluetooth device."""
        mac = _validate_mac(mac)
        logger.info("BT disconnect — mac=%s", mac)
        try:
            output = self._bt_quick([f"disconnect {mac}"], timeout=10)
            if "Successful disconnected" in output or "not connected" in output.lower():
                logger.info("BT disconnect succeeded — mac=%s", mac)
                return {"success": True, "message": "Disconnected"}
            logger.warning("BT disconnect — unexpected output for mac=%s", mac)
            return {"success": False, "message": "Disconnect did not succeed"}
        except RuntimeError as e:
            logger.error("BT disconnect failed — mac=%s: %s", mac, e)
            return {"success": False, "message": str(e)}

    def remove(self, mac: str) -> dict:
        """Remove (forget) a Bluetooth device."""
        mac = _validate_mac(mac)
        logger.info("BT remove — mac=%s", mac)
        try:
            output = self._bt_quick([f"remove {mac}"], timeout=10)
            if "Device has been removed" in output:
                logger.info("BT remove succeeded — mac=%s", mac)
                return {"success": True, "message": "Removed"}
            logger.warning("BT remove — unexpected output for mac=%s", mac)
            return {"success": False, "message": "Remove did not succeed"}
        except RuntimeError as e:
            logger.error("BT remove failed — mac=%s: %s", mac, e)
            return {"success": False, "message": str(e)}

    def setup_controller(self, mac: str) -> dict:
        """One-shot pair + trust + connect for a new controller.

        Executes each step sequentially, verifying completion before
        proceeding.  Retries connection once on failure.  Checks ERTM
        status upfront (Xbox controllers require ERTM disabled).
        """
        mac = _validate_mac(mac)
        logger.info("BT setup_controller — mac=%s", mac)

        # Pre-check: ERTM must be disabled for Xbox controllers
        try:
            with open("/sys/module/bluetooth/parameters/disable_ertm") as f:
                if f.read().strip() != "Y":
                    logger.error("ERTM is enabled — Xbox controllers will not create input devices")
                    return {"success": False, "message": "ERTM not disabled — run environment.sh"}
        except FileNotFoundError:
            logger.warning("Cannot check ERTM — /sys path not found")

        # Step 1: Pair
        result = self.pair(mac)
        if not result["success"]:
            return {"success": False, "message": f"Pair failed: {result['message']}"}
        logger.info("BT setup 1/3 — paired")

        # Step 2: Trust
        result = self.trust(mac)
        if not result["success"]:
            return {"success": False, "message": f"Trust failed: {result['message']}"}
        logger.info("BT setup 2/3 — trusted")

        # Step 3: Connect (retry once)
        result = self.connect(mac)
        if not result["success"]:
            logger.warning("BT setup — first connect attempt failed, retrying")
            time.sleep(2)
            result = self.connect(mac)
        if not result["success"]:
            return {"success": False, "message": f"Connect failed: {result['message']}"}
        logger.info("BT setup 3/3 — connected")

        # Step 4: Wait for joystick input device
        for _ in range(5):
            if glob.glob("/dev/input/js*"):
                logger.info("BT setup complete — /dev/input/js* found")
                return {"success": True, "message": "Controller ready"}
            time.sleep(1)

        logger.warning("BT setup — connected but no /dev/input/js*")
        return {"success": True, "message": "Connected but no input device — check ERTM"}

    def list_devices(self) -> list[dict]:
        """List Bluetooth devices that are paired or connected.

        Uses ``devices`` then filters via ``info`` to find devices with
        ``Paired: yes`` or ``Connected: yes``.  Compatible with bluez < 5.77
        (which lacks ``devices Paired``).
        """
        try:
            output = self._bt_quick(["devices"], timeout=5)
            all_devices = self._parse_devices(output)
            if not all_devices:
                return []

            info_cmds = [f"info {d['mac']}" for d in all_devices]
            info_output = self._bt_quick(info_cmds, timeout=10)

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
        """Get full connection status for a specific device."""
        mac = _validate_mac(mac)
        try:
            output = self._bt_quick([f"info {mac}"], timeout=5)
            connected = "Connected: yes" in output
            paired = "Paired: yes" in output
            trusted = "Trusted: yes" in output

            battery_pct = None
            name = mac
            for line in output.splitlines():
                if "Battery Percentage" in line:
                    try:
                        battery_pct = int(line.split("(")[-1].split(")")[0])
                    except (ValueError, IndexError):
                        pass
                elif "Name:" in line:
                    name = line.split("Name:")[-1].strip()

            logger.debug(
                "BT status — mac=%s connected=%s paired=%s trusted=%s",
                mac, connected, paired, trusted,
            )
            return {
                "connected": connected,
                "paired": paired,
                "trusted": trusted,
                "name": name,
                "mac": mac,
                "battery_pct": battery_pct,
            }
        except (ValueError, RuntimeError) as e:
            logger.error("Get status failed: %s", e)
            return {
                "connected": False,
                "paired": False,
                "trusted": False,
                "name": mac,
                "mac": mac,
                "battery_pct": None,
            }

    @staticmethod
    def _parse_devices(output: str) -> list[dict]:
        """Parse bluetoothctl ``devices`` output into structured data."""
        devices = []
        for line in output.splitlines():
            line = line.strip()
            if line.startswith("Device "):
                parts = line.split(" ", 2)
                if len(parts) >= 3:
                    devices.append({"mac": parts[1], "name": parts[2]})
        return devices
