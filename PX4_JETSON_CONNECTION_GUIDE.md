# PX4 ↔ NVIDIA Jetson Connection Guide (pymavlink)

Complete guide for connecting PX4 flight controller to NVIDIA Jetson using pymavlink over serial UART.

---

## 1. Hardware Wiring

### UART Voltage Compatibility
- Jetson UART: **3.3V TTL logic**
- PX4 TELEM ports: **3.3V TTL logic**
- ✅ **No level shifter needed**

### Wiring Diagram
```
Jetson UART TX  →  PX4 TELEM RX
Jetson UART RX  →  PX4 TELEM TX
Jetson GND      →  PX4 GND
```

### Jetson UART Pin Reference

| Jetson Model | UART Device     | Pins (40-pin header)      |
|--------------|-----------------|---------------------------|
| Nano Dev Kit | `/dev/ttyTHS1`  | Pin 8 (TX), Pin 10 (RX)   |
| Xavier NX    | `/dev/ttyTHS1`  | Carrier board dependent   |
| Orin Nano/NX | `/dev/ttyTHS1`  | Carrier board dependent   |

**Important**: Ensure the UART port is enabled and not used by the system console.

---

## 2. PX4 Configuration

### Enable MAVLink on TELEM Port

For **TELEM2** (adjust for your port):

```
SER_TEL2_BAUD = 921600       # Baud rate (or 57600 for stability)
SER_TEL2_PROTO = 1           # MAVLink protocol
MAV_0_MODE = 2               # Onboard mode
MAV_0_RATE = 100             # 100 Hz (optional)
```

### Legacy PX4 Versions (v1.12 and earlier):
```
MAV_1_CONFIG = TELEM2
MAV_1_MODE = Onboard
MAV_1_BAUD = 921600
```

### Apply Settings
1. Set parameters in QGroundControl or PX4 console
2. **Restart PX4** (power cycle or `reboot` command)
3. Verify settings with `param show SER_TEL2*`

---

## 3. Jetson UART Setup

### Enable UART and Set Permissions

```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER

# Set permissions for UART device
sudo chmod 666 /dev/ttyTHS1

# Disable serial console (critical!)
sudo systemctl disable nvgetty.service
sudo systemctl stop nvgetty.service

# Reboot to apply changes
sudo reboot
```

### Verify UART is Available
```bash
# List available serial devices
ls -l /dev/ttyTHS*

# Expected output:
# crw-rw-rw- 1 root dialout ... /dev/ttyTHS1
```

---

## 4. Install pymavlink

```bash
# Using pip
pip install pymavlink

# Or using uv (as in your setup)
uv pip install pymavlink

# Optional: MAVProxy for debugging
pip install mavproxy
```

---

## 5. Minimal Test Script

Create `test_px4_connection.py`:

```python
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
```

### Run Test
```bash
chmod +x test_px4_connection.py
python3 test_px4_connection.py
```

### Expected Output
```
==================================================
PX4 Connection Test
==================================================
Device: /dev/ttyTHS1
Baud: 921600
==================================================

[1/4] Connecting to PX4...
✓ Serial port opened

[2/4] Waiting for heartbeat...
✓ Heartbeat received!
  System ID: 1
  Component ID: 1
  Autopilot: 12
  Type: 2

[3/4] Requesting data stream (20 Hz)...
✓ Data stream requested

[4/4] Reading messages (10 seconds)...
  HEARTBEAT: {...}
  ATTITUDE: {...}
  ...

==================================================
Message Summary (10 sec)
==================================================
  ATTITUDE                       200 msgs ( 20.0 Hz)
  HEARTBEAT                       10 msgs (  1.0 Hz)
  ...

✓ Connection test SUCCESSFUL!
```

---

## 6. Common Problems & Solutions

### ❌ NO HEARTBEAT RECEIVED

**Possible causes:**
1. **Wrong UART device**
   - Verify: `ls -l /dev/ttyTHS*`
   - Try: `/dev/ttyTHS0` if `/dev/ttyTHS1` doesn't work

2. **nvgetty occupying port**
   ```bash
   sudo systemctl status nvgetty
   # Should be "inactive (dead)"
   ```

3. **Wrong baudrate**
   - Check PX4: `param show SER_TEL2_BAUD`
   - Match in Python script

4. **TX/RX reversed**
   - Double-check wiring diagram above

5. **Wrong TELEM port configured**
   - Verify which physical port you're using
   - Check corresponding `SER_TEL*` parameters

6. **PX4 not sending MAVLink**
   - Check `SER_TEL2_PROTO = 1` (MAVLink)
   - Reboot PX4 after parameter changes

### ❌ Garbled/Corrupted Messages

**Causes:**
- Baudrate mismatch
- Cable too long (>30cm) at 921600 baud
- Electrical interference

**Solution:**
- Reduce to 57600 baud on both sides
- Use shorter, shielded cable

### ❌ Permission Denied

```bash
# Fix:
sudo chmod 666 /dev/ttyTHS1
sudo usermod -a -G dialout $USER
# Then logout/login or reboot
```

---

## 7. Debugging with MAVProxy

MAVProxy is excellent for diagnosing connection issues:

```bash
# Install
pip install mavproxy

# Run
mavproxy.py --master=/dev/ttyTHS1,921600
```

**If connection is good, you'll see:**
```
Connect /dev/ttyTHS1,921600
Received 250 parameters
online system 1
APM: ArduCopter V4.3.0 (px4-v5)
...
```

**MAVProxy commands:**
```
status               # System status
param show SER*      # View serial parameters
mode STABILIZE       # Change flight mode
arm throttle         # Arm (use with caution!)
```

---

## 8. Your Current Setup (MavDataHose)

Your [firehose_node.py](../MavDataHose/mav_data_hose/firehose_node.py) already implements a production-ready MAVLink→ROS2 bridge with:

✅ Robust UART connection (`/dev/ttyTHS1`, 921600 baud)
✅ RTS/CTS flow control disabled (Jetson fix)
✅ Auto-reconnect enabled
✅ Message interval requests
✅ Published ROS2 topics: IMU, GPS, attitude, odometry, battery, etc.

### Quick Start with MavDataHose

From [JetsonConnection.md](JetsonConnection.md):

```bash
# 1. Sync files to Jetson
rsync -avz --delete --ignore-times MavDataHose/ magpie@192.168.55.1:~/abm-sync/

# 2. SSH to Jetson
ssh magpie@192.168.55.1

# 3. Build ROS2 package
cd ~/abm-sync
source .venv/bin/activate
colcon build --symlink-install --packages-select mav_data_hose

# 4. Run firehose node
source install/setup.bash
ros2 run mav_data_hose firehose_node

# 5. Monitor topics (in another terminal)
ros2 topic list
ros2 topic echo /mav/attitude
ros2 topic hz /mav/imu
```

---

## 9. Validation Checklist

Before running your application, verify:

- [ ] Physical wiring correct (TX↔RX, GND↔GND)
- [ ] PX4 `SER_TEL2_BAUD` matches code baud rate
- [ ] PX4 `SER_TEL2_PROTO = 1` (MAVLink)
- [ ] PX4 rebooted after parameter changes
- [ ] Jetson nvgetty service disabled
- [ ] `/dev/ttyTHS1` has 666 permissions
- [ ] User in `dialout` group
- [ ] pymavlink installed
- [ ] Test script receives heartbeat
- [ ] MAVProxy shows messages (optional debug step)

---

## 10. Next Steps

Once basic connection works:

1. **Message Filtering**: Only request messages you need
   ```python
   # Request specific messages at specific rates
   master.mav.command_long_send(
       master.target_system,
       master.target_component,
       mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
       0,
       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,  # Message ID
       50000,  # Interval in microseconds (20 Hz)
       0, 0, 0, 0, 0
   )
   ```

2. **Sending Commands**: Arm, set mode, control attitude
   ```python
   # Set mode to OFFBOARD
   master.set_mode('OFFBOARD')

   # Arm
   master.arducopter_arm()
   ```

3. **High-Rate Control**: For control loops, use uXRCE-DDS (DDS bridge) instead of serial MAVLink for lower latency

---

## Hardware-Specific Notes

### Which Jetson do you have?
Tell me your exact model so I can provide specific pin diagrams:
- Jetson Nano Dev Kit
- Jetson Xavier NX Dev Kit
- Jetson Orin Nano Dev Kit
- Custom carrier board

### Which Flight Controller?
- Pixhawk 4
- Pixhawk 6C/6X
- Holybro Durandal
- Cube Orange/Yellow
- Custom board

---

## Support

If you encounter issues:

1. Run the test script above and report the exact error
2. Share output of:
   ```bash
   ls -l /dev/ttyTHS*
   systemctl status nvgetty
   dmesg | grep tty
   ```
3. From PX4, share:
   ```
   param show SER_TEL2*
   param show MAV_0*
   ```

This should get you connected reliably!
