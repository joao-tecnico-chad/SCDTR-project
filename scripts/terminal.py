#!/usr/bin/env python3
"""
Interactive serial terminal for the distributed illumination system.
Connects to a Pico node and provides a serial console with local echo.
Usage: python3 terminal.py
"""

import serial
import serial.tools.list_ports
import sys
import threading
import time


def find_picos():
    """Find all connected Pico USB serial ports."""
    ports = []
    for p in serial.tools.list_ports.comports():
        if 'usbmodem' in p.device:
            ports.append(p.device)
    ports.sort()
    return ports


def reader_thread(ser):
    """Background thread that prints incoming serial data."""
    while True:
        try:
            data = ser.read(1024)
            if data:
                text = data.decode('utf-8', errors='replace')
                sys.stdout.write(text)
                sys.stdout.flush()
        except:
            break


def main():
    ports = find_picos()

    if not ports:
        print("No Picos found! Plug them in via USB.")
        return

    print("=" * 40)
    print("  Distributed Illumination Terminal")
    print("=" * 40)
    print()
    print("Available Picos:")
    for i, port in enumerate(ports):
        print(f"  [{i + 1}] {port}")
    print()

    choice = input(f"Select Pico (1-{len(ports)}): ").strip()
    try:
        idx = int(choice) - 1
        if idx < 0 or idx >= len(ports):
            raise ValueError
    except ValueError:
        print("Invalid choice.")
        return

    port = ports[idx]
    print(f"\nConnecting to {port} at 115200 baud...")
    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(0.3)

    # Show any pending boot messages instead of discarding them
    pending = ser.read(4096)
    if pending:
        print(pending.decode('utf-8', errors='replace'), end='')

    print(f"Connected! Type commands and press Enter.")
    print(f"Type 'help' to see all commands.")
    print(f"Type 'quit' or Ctrl+C to exit.\n")
    print("-" * 40)

    # Start background reader
    t = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    t.start()

    try:
        while True:
            line = input()
            if line.strip().lower() == 'quit':
                break
            ser.write((line + '\n').encode())
            time.sleep(0.05)
    except (KeyboardInterrupt, EOFError):
        pass

    print("\nDisconnected.")
    ser.close()


if __name__ == '__main__':
    main()
