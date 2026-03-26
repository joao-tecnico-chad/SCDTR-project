# Distributed Calibration

This folder contains the shared calibration firmware for all controller nodes.

## Upload
Flash `distributed_calibration.ino` to every Raspberry Pi Pico node.

## Startup
On serial monitor at `115200`, each node asks for:
- node ID
- total number of nodes

Use IDs from `1..N`. Node `1` acts as coordinator.

## Commands
- `u <i> <val>`: set PWM of luminaire `i`
- `u 0 <val>`: set PWM of all luminaires synchronously
- `g u <i>`: get duty cycle of node `i`
- `r <i> <val>`: set illuminance reference in lux
- `g r <i>`: get reference of node `i`
- `g y <i>`: get illuminance of node `i`
- `g v <i>`: get LDR voltage of node `i`
- `s <x> <i>`: start stream of `x` (`y` or `u`) on local node
- `S <x> <i>`: stop stream
- `g b <x> <i>`: get local last-minute buffer
- `c <pwm>`: start distributed calibration from coordinator
- `rpt`: print local gain row after calibration
- `R`: restart and recalibrate

## Calibration
After wake-up and network discovery, the coordinator can trigger a synchronized calibration sequence. Each node estimates its own row of the distributed gain matrix by measuring:
- baseline illuminance
- self-coupling gain
- cross-coupling gains from the other luminaires
