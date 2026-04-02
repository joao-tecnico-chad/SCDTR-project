# CAN_ORDER

Multicore RP2040 CAN firmware using `mcp_can` and MCP2515.

## Features
- Core 0 handles UI, calibration state machine, and protocol parsing.
- Core 1 exclusively owns the MCP2515 CAN controller.
- Synchronized distributed calibration over CAN.
- Relative calibration start delay to avoid per-node `millis()` desynchronization.
- Repeated calibration-plan broadcast for more robust startup across multiple nodes.

## Recent fix
The calibration plan now uses a relative start delay instead of an absolute tick from the coordinator. The coordinator also retransmits the calibration plan multiple times so all nodes receive both calibration frames before the sequence starts.

## Main file
- `src/main.cpp`
