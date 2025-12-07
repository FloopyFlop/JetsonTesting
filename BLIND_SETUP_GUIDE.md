# PX4 Setup Without USB Access

Since your PX4 doesn't have a USB port, here's how to proceed:

## Most Common Issue: SER_TEL2_PROTO Not Set

The #1 reason for "no data on line" is **PX4 TELEM port not configured for MAVLink**.

By default, many PX4 builds have `SER_TEL2_PROTO = 0` (disabled) or set to GPS/other protocols.

## Required PX4 Parameters

For TELEM2 port (most common companion computer port):

```
SER_TEL2_BAUD = 921600
SER_TEL2_PROTO = 1        # ← THIS IS CRITICAL (1 = MAVLink)
MAV_0_CONFIG = 102        # 102 = TELEM2
MAV_0_MODE = 2            # Onboard mode
```

## How to Set Parameters Without USB

### Method 1: SD Card Config File (Best for blind setup)

1. **Remove SD card from PX4**
2. **Mount on your computer**
3. **Edit or create: `/etc/extras.txt`** on the SD card
4. **Add these lines:**

```bash
# Force TELEM2 to MAVLink at 921600 baud
param set SER_TEL2_BAUD 921600
param set SER_TEL2_PROTO 1
param set MAV_0_CONFIG 102
param set MAV_0_MODE 2
param save
```

5. **Eject SD card**
6. **Put back in PX4**
7. **Reboot PX4** (power cycle)

The `/etc/extras.txt` file runs on boot and will set these parameters.

### Method 2: Use FTDI USB-to-Serial Adapter

**Get a 3.3V FTDI adapter:**
- FTDI FT232RL (set to 3.3V!)
- CP2102 USB-to-TTL adapter
- SparkFun FTDI Basic 3.3V

**Wire to PX4 TELEM port:**
```
FTDI TX  → PX4 TELEM RX (Pin 3 of JST-GH)
FTDI RX  → PX4 TELEM TX (Pin 2 of JST-GH)
FTDI GND → PX4 GND      (Pin 6 of JST-GH)
```

**Connect to Mac and use MAVProxy:**
```bash
# Install MAVProxy
pip3 install mavproxy

# Connect (replace XXXX with your device)
mavproxy.py --master=/dev/tty.usbserial-XXXX,57600

# Once connected, you'll see MAV> prompt:
MAV> param set SER_TEL2_BAUD 921600
MAV> param set SER_TEL2_PROTO 1
MAV> param set MAV_0_CONFIG 102
MAV> param set MAV_0_MODE 2
MAV> param save
MAV> reboot
```

### Method 3: Default PX4 Configuration Might Already Be Correct

Some PX4 builds enable TELEM2 by default. Let's verify your wiring is correct:

## Double-Check Your Wiring

### TELEM Port Pinout (Standard 6-pin JST-GH)

Looking at the connector **from the wire side** (not the board):

```
 ___________________
|  1  2  3  4  5  6 |
|___________________|
```

| Pin | Signal | Connect to Jetson          |
|-----|--------|----------------------------|
| 1   | VCC    | **DO NOT CONNECT**         |
| 2   | TX     | **Pin 10 (UART RX)**       |
| 3   | RX     | **Pin 8 (UART TX)**        |
| 4   | CTS    | Leave unconnected          |
| 5   | RTS    | Leave unconnected          |
| 6   | GND    | **Any GND pin (Pin 6, 9, 14, 20, etc.)** |

### Jetson Xavier NX 40-Pin Header

```
Pin 8  (UART1_TXD) → PX4 TELEM RX (Pin 3)
Pin 10 (UART1_RXD) → PX4 TELEM TX (Pin 2)
Pin 6  (GND)       → PX4 GND (Pin 6)
```

**Critical: TX goes to RX, and RX goes to TX** (crossed connection)

## Verify Jetson UART is Transmitting

Let's make sure the Jetson side is actually working:

### Loopback Test

1. **Disconnect from PX4**
2. **On Jetson, short Pin 8 to Pin 10** (TX to RX) with a wire
3. **Run this test:**

```python
import serial
import time

ser = serial.Serial('/dev/ttyTHS1', 921600, timeout=1)
time.sleep(0.1)

# Clear buffers
ser.reset_input_buffer()
ser.reset_output_buffer()

# Send test data
test_msg = b'LOOPBACK_TEST_123'
ser.write(test_msg)
time.sleep(0.1)

# Read back
received = ser.read(len(test_msg))
print(f"Sent:     {test_msg}")
print(f"Received: {received}")

if received == test_msg:
    print("✓ Jetson UART is working!")
else:
    print("✗ Jetson UART loopback failed")

ser.close()
```

If this works, Jetson UART is fine.

## Alternative UART on Jetson Xavier NX

Xavier NX has multiple UARTs. Try enabling `/dev/ttyTHS0`:

```bash
# Check what's using ttyTHS0
sudo systemctl status nvgetty
# If active on ttyTHS0, disable it:
sudo systemctl disable nvgetty
sudo systemctl stop nvgetty

# Check if ttyTHS0 exists
ls -l /dev/ttyTHS0

# If it exists, modify test script to use it:
DEVICE = "/dev/ttyTHS0"
```

## What's Your Flight Controller Model?

Knowing the exact model helps:
- Pixhawk 4 / 4 Mini
- Pixhawk 6C / 6X
- Holybro Kakute H7
- Holybro Durandal
- mRo Pixracer
- Cube Orange / Yellow / Black
- Custom board?

Different boards have different default configurations.

## Next Steps

1. **Try the SD card method** (easiest if you can remove the SD card)
2. **Get an FTDI adapter** ($5-10 on Amazon) to access PX4 console
3. **Verify wiring one more time** - TX/RX crossed, GND connected
4. **Try different TELEM port** - if you have TELEM1, try that instead

## If Still No Data After All This

Then the issue is likely:
1. **PX4 not booting properly** - check power, LED patterns
2. **Damaged UART hardware** on PX4 or Jetson
3. **Wrong TELEM port** - try all available ports on PX4

Let me know which method you want to try first!
