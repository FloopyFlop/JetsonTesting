# PX4 Parameter Checklist

## Critical Parameters to Verify

Connect PX4 to QGroundControl or use NSH console and check:

### 1. Find which TELEM port you're using physically

Most common ports:
- **TELEM1**: Often reserved for telemetry radio
- **TELEM2**: Common for companion computer
- **GPS2/TELEM3**: Alternative option

### 2. Check Serial Port Configuration

For **TELEM2** (adjust SER_TEL1, SER_TEL3, etc. based on your physical port):

```
param show SER_TEL2_BAUD
param show SER_TEL2_PROTO
```

**Required values:**
```
SER_TEL2_BAUD = 921600    # Or 57600 for stability
SER_TEL2_PROTO = 1        # MAVLink v1/v2
```

### 3. Check MAVLink Stream Configuration

```
param show MAV_0_CONFIG
param show MAV_0_MODE
param show MAV_0_RATE
```

**Recommended values:**
```
MAV_0_CONFIG = TELEM2     # Must match your physical port
MAV_0_MODE = 2            # Onboard mode
MAV_0_RATE = 100          # Optional, 100 Hz
```

### 4. Alternative: Legacy Parameters (PX4 v1.12 and older)

```
param show MAV_1_CONFIG
param show MAV_1_MODE
param show MAV_1_BAUD
```

Should be:
```
MAV_1_CONFIG = TELEM2
MAV_1_MODE = 2
```

## Setting Parameters (if needed)

### Via QGroundControl:
1. Connect PX4 via USB
2. Go to Vehicle Setup → Parameters
3. Search for "SER_TEL2"
4. Set:
   - `SER_TEL2_BAUD = 921600`
   - `SER_TEL2_PROTO = 1`
5. Search for "MAV_0"
6. Set:
   - `MAV_0_CONFIG = TELEM2`
   - `MAV_0_MODE = 2`
7. **Reboot PX4** (critical!)

### Via NSH Console (USB serial):
```bash
param set SER_TEL2_BAUD 921600
param set SER_TEL2_PROTO 1
param set MAV_0_CONFIG 102  # TELEM2
param set MAV_0_MODE 2
param save
reboot
```

## Verification After Reboot

```bash
param show SER_TEL2_BAUD
param show SER_TEL2_PROTO
param show MAV_0_CONFIG
param show MAV_0_MODE
```

## Common Port Mappings

| Physical Port | SER_TEL*_CONFIG | MAV_*_CONFIG value |
|---------------|-----------------|-------------------|
| TELEM1        | SER_TEL1        | 101               |
| TELEM2        | SER_TEL2        | 102               |
| GPS2/TELEM3   | SER_GPS2        | 201               |

## Testing PX4 Transmits MAVLink

Connect PX4 to your computer via USB and run:

```bash
# On computer (not Jetson)
mavproxy.py --master=/dev/ttyACM0  # or /dev/ttyUSB0

# Or with pymavlink
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0')
m.wait_heartbeat()
print('Heartbeat received!')
"
```

If this works, PX4 is transmitting MAVLink correctly and the issue is:
- Wiring between Jetson ↔ PX4
- Baudrate mismatch
- Wrong UART on Jetson side

## Wiring Double-Check

**Correct wiring:**
```
Jetson Pin 8 (UART1 TX)  →  PX4 TELEM2 RX
Jetson Pin 10 (UART1 RX) →  PX4 TELEM2 TX
Jetson GND               →  PX4 GND
```

**DO NOT connect:**
- VCC/5V (unless you need to power the Jetson from PX4, which is uncommon)

## Baudrate Test

If 921600 doesn't work, try 57600 on BOTH sides:

**PX4:**
```
param set SER_TEL2_BAUD 57600
reboot
```

**Jetson script:**
```python
BAUD = 57600  # Change this line
```

Lower baudrates are more reliable with longer cables or electrical noise.
