#!/usr/bin/env python3
"""
Comprehensive PX4 connection diagnostic tool
Tests multiple UART devices, baudrates, and configurations
"""

import os
import sys
import time
from pymavlink import mavutil

# Test configurations
DEVICES = ['/dev/ttyTHS1', '/dev/ttyTHS0', '/dev/ttyUSB0', '/dev/ttyACM0']
BAUDRATES = [921600, 57600, 115200, 460800]
TIMEOUT = 5  # seconds per attempt

def check_device_exists(device):
    """Check if device file exists and is accessible"""
    if not os.path.exists(device):
        return False, "Device does not exist"

    if not os.access(device, os.R_OK | os.W_OK):
        return False, f"Permission denied (try: sudo chmod 666 {device})"

    return True, "OK"

def test_connection(device, baud):
    """Attempt connection with given device and baudrate"""
    print(f"\n{'='*60}")
    print(f"Testing: {device} @ {baud} baud")
    print(f"{'='*60}")

    # Check device exists
    exists, msg = check_device_exists(device)
    if not exists:
        print(f"✗ {msg}")
        return False
    else:
        print(f"✓ Device accessible: {msg}")

    # Try to open connection
    try:
        print("  Opening connection...")
        master = mavutil.mavlink_connection(
            device,
            baud=baud,
            autoreconnect=False,
            rtscts=False,
            dsrdtr=False,
            timeout=TIMEOUT
        )
        print("  ✓ Serial port opened")
    except Exception as e:
        print(f"  ✗ Failed to open: {e}")
        return False

    # Wait for heartbeat
    print(f"  Waiting for heartbeat ({TIMEOUT}s timeout)...")
    start = time.time()

    hb = master.wait_heartbeat(timeout=TIMEOUT)

    if hb is None:
        elapsed = time.time() - start
        print(f"  ✗ No heartbeat after {elapsed:.1f}s")

        # Try to read any raw data
        print("  Checking for ANY data on the line...")
        master.mav.srcSystem = 255
        master.mav.srcComponent = 1

        any_data = False
        for _ in range(50):
            try:
                msg = master.recv_match(blocking=False, timeout=0.1)
                if msg:
                    any_data = True
                    print(f"    Received: {msg.get_type()}")
            except:
                pass

        if not any_data:
            print("    No data detected on serial line")

        master.close()
        return False

    # Success!
    print(f"  ✓✓✓ HEARTBEAT RECEIVED! ✓✓✓")
    print(f"  System ID: {master.target_system}")
    print(f"  Component ID: {master.target_component}")
    print(f"  Autopilot: {hb.autopilot}")
    print(f"  Type: {hb.type}")
    print(f"  MAVLink version: {hb.mavlink_version}")

    # Try to get a few more messages
    print("\n  Reading additional messages...")
    msg_types = set()
    for _ in range(100):
        msg = master.recv_match(blocking=False, timeout=0.01)
        if msg:
            msg_types.add(msg.get_type())

    print(f"  Message types received: {sorted(msg_types)}")

    master.close()
    return True

def list_serial_devices():
    """List all available serial devices"""
    print("\nAvailable serial devices:")
    print("="*60)

    for device in ['/dev/ttyTHS0', '/dev/ttyTHS1', '/dev/ttyUSB0',
                   '/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']:
        if os.path.exists(device):
            stat = os.stat(device)
            perms = oct(stat.st_mode)[-3:]
            print(f"  {device} (permissions: {perms})")

def main():
    print("="*60)
    print("PX4 Connection Diagnostic Tool")
    print("="*60)

    list_serial_devices()

    print("\n\nStarting systematic connection tests...")
    print("This will test multiple devices and baudrates.")
    print("="*60)

    success = False

    for device in DEVICES:
        if not os.path.exists(device):
            print(f"\n✗ Skipping {device} (does not exist)")
            continue

        for baud in BAUDRATES:
            if test_connection(device, baud):
                success = True
                print("\n" + "="*60)
                print("SUCCESS! Use these settings:")
                print(f"  Device: {device}")
                print(f"  Baudrate: {baud}")
                print("="*60)
                return 0

            # Small delay between attempts
            time.sleep(0.5)

    if not success:
        print("\n" + "="*60)
        print("NO HEARTBEAT FOUND on any configuration")
        print("="*60)
        print("\nNext steps:")
        print("1. Verify PX4 is powered on and booted")
        print("2. Check physical wiring:")
        print("   - Jetson TX → PX4 RX")
        print("   - Jetson RX → PX4 TX")
        print("   - GND → GND")
        print("3. Connect PX4 to computer via USB and check parameters:")
        print("   - param show SER_TEL2_BAUD")
        print("   - param show SER_TEL2_PROTO")
        print("   - Make sure SER_TEL2_PROTO = 1 (MAVLink)")
        print("4. Test PX4 with MAVProxy on computer:")
        print("   - mavproxy.py --master=/dev/ttyUSB0")
        print("5. Try connecting PX4 directly to computer with FTDI cable")
        print("   to verify PX4 is transmitting MAVLink")
        return 1

if __name__ == '__main__':
    sys.exit(main())
