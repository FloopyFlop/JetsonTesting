#!/usr/bin/env python3
"""
Minimal PX4 connection test using pymavlink
Tests: Heartbeat, attitude data, and basic telemetry
"""

from pymavlink import mavutil
import time

# Configuration
DEVICE = "/dev/ttyTHS1"
BAUD = 921600

print("=" * 50)
print("PX4 Connection Test")
print("=" * 50)
print(f"Device: {DEVICE}")
print(f"Baud: {BAUD}")
print("=" * 50)

# Connect
print("\n[1/4] Connecting to PX4...")
try:
    master = mavutil.mavlink_connection(
        DEVICE,
        baud=BAUD,
        autoreconnect=True,
        rtscts=False,  # Jetson UART fix
        dsrdtr=False   # Jetson UART fix
    )
    print("✓ Serial port opened")
except Exception as e:
    print(f"✗ FAILED: {e}")
    exit(1)

# Wait for heartbeat
print("\n[2/4] Waiting for heartbeat...")
hb = master.wait_heartbeat(timeout=30)

if hb is None:
    print("✗ NO HEARTBEAT RECEIVED")
    print("\nTroubleshooting:")
    print("  - Check wiring (TX/RX reversed?)")
    print("  - Verify PX4 TELEM port parameters")
    print("  - Check baudrate matches")
    print("  - Ensure nvgetty is disabled")
    exit(1)

print(f"✓ Heartbeat received!")
print(f"  System ID: {master.target_system}")
print(f"  Component ID: {master.target_component}")
print(f"  Autopilot: {hb.autopilot}")
print(f"  Type: {hb.type}")

# Request data stream
print("\n[3/4] Requesting data stream (20 Hz)...")
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    20,  # Hz
    1    # start
)
print("✓ Data stream requested")

# Read messages
print("\n[4/4] Reading messages (10 seconds)...\n")
msg_counts = {}
start_time = time.time()

while time.time() - start_time < 10:
    msg = master.recv_match(blocking=False)
    if msg:
        msg_type = msg.get_type()
        msg_counts[msg_type] = msg_counts.get(msg_type, 0) + 1

        # Print first occurrence of each message type
        if msg_counts[msg_type] == 1:
            print(f"  {msg_type}: {msg}")

    time.sleep(0.01)

# Summary
print("\n" + "=" * 50)
print("Message Summary (10 sec)")
print("=" * 50)
for msg_type, count in sorted(msg_counts.items(), key=lambda x: x[1], reverse=True):
    hz = count / 10.0
    print(f"  {msg_type:<30} {count:>5} msgs ({hz:>6.1f} Hz)")

print("\n✓ Connection test SUCCESSFUL!")
