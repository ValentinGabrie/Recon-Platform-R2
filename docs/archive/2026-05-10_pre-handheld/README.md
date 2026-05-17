# Pre-Handheld Archive — 2026-05-10

This directory holds the documentation set that described the project as an
**autonomous mapping robot** (codename `roomba`). It is preserved verbatim,
with deprecation headers stamped on each file, as a historical reference.

## What was archived and why

The project pivoted from an autonomous mobile robot to a **handheld LIDAR
scanner** on 2026-05-09. The pivot dropped roughly half the codebase
(motors, motor controller, ESP32-as-motor-coprocessor, Bluetooth/Xbox
controller stack, joy_linux, xpadneo, fuzzy frontier exploration, the
entire `roomba_navigation` package, draw-via-controller flow, and the
related test suites). The old docs would have required a full rewrite to
match the new architecture, so they are archived here and the new docs
live in `docs/` as a clean slate.

## Contents

| File                          | Original location                     | Replaced by                       |
| ----------------------------- | ------------------------------------- | --------------------------------- |
| `rules.md`                    | repo root                             | [`docs/AGENT_RULES.md`](../../AGENT_RULES.md) |
| `project_requirements.md`     | repo root                             | [`docs/SPEC.md`](../../SPEC.md)             |
| `project_status.md`           | repo root                             | [`docs/STATUS.md`](../../STATUS.md)         |
| `esp32_firmware.md`           | `roomba_ws/docs/`                     | [`firmware/esp32/README.md`](../../../firmware/esp32/README.md) + [`docs/UART_PROTOCOL.md`](../../UART_PROTOCOL.md) |

## Frozen at

- **Branch tip:** `main`, commit `c4f4c0a` (Stage 5 — LD14P LIDAR bench test)
- **Pivot commit:** `5088514` on the `handheld` branch (Stage H1)
- **Archive date:** 2026-05-10

## Reading guide

- For "what was this project supposed to do?" — start with
  [`project_requirements.md`](project_requirements.md) (Section 1: System Overview).
- For "what state was the autonomous-robot codebase in?" — start with
  [`project_status.md`](project_status.md) (Section 1: Executive Summary).
- For "what rules were the agents working under?" — see
  [`rules.md`](rules.md). Most carry forward unchanged into
  [`docs/AGENT_RULES.md`](../../AGENT_RULES.md); the differences are summarised
  at the top of that file.
- For "how was the ESP32 wired before the pivot?" — see
  [`esp32_firmware.md`](esp32_firmware.md). The current ESP32 wiring is
  completely different.

> Do not edit files in this directory. They are a frozen snapshot.
