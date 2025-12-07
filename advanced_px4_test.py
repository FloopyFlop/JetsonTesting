#!/usr/bin/env python3
"""
Advanced PX4 connection test with specific message requests
Demonstrates proper message interval setup and monitoring
"""

from pymavlink import mavutil
import time
import sys

# Configuration
DEVICE = "/dev/ttyTHS1"
BAUD = 921600
TARGET_RATE_HZ = 50  # Request 50 Hz for high-rate messages

class PX4Tester:
    def __init__(self, device, baud):
        self.device = device
        self.baud = baud
        self.master = None
        self.msg_counts = {}
        self.start_time = None

    def connect(self):
        """Establish connection to PX4"""
        print("=" * 60)
        print("Advanced PX4 Connection Test")
        print("=" * 60)
        print(f"Device: {self.device}")
        print(f"Baud: {self.baud}")
        print(f"Target rate: {TARGET_RATE_HZ} Hz")
        print("=" * 60)

        print("\n[1/5] Opening serial connection...")
        try:
            self.master = mavutil.mavlink_connection(
                self.device,
                baud=self.baud,
                autoreconnect=True,
                source_system=1,
                source_component=1,
                rtscts=False,
                dsrdtr=False,
                timeout=30
            )
            print("✓ Serial port opened")
        except Exception as e:
            print(f"✗ FAILED: {e}")
            return False

        print("\n[2/5] Waiting for heartbeat...")
        hb = self.master.wait_heartbeat(timeout=30)

        if hb is None:
            print("✗ NO HEARTBEAT RECEIVED")
            self._print_troubleshooting()
            return False

        print(f"✓ Heartbeat received!")
        print(f"  System ID: {self.master.target_system}")
        print(f"  Component ID: {self.master.target_component}")
        print(f"  Autopilot type: {hb.autopilot}")
        print(f"  Vehicle type: {hb.type}")
        print(f"  MAVLink version: {hb.mavlink_version}")

        return True

    def request_message_intervals(self):
        """Request specific messages at defined intervals"""
        print("\n[3/5] Requesting specific message intervals...")

        # Messages to request with their IDs
        messages = {
            'HIGHRES_IMU': mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
            'ATTITUDE': mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            'ATTITUDE_QUATERNION': mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
            'LOCAL_POSITION_NED': mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            'GLOBAL_POSITION_INT': mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            'BATTERY_STATUS': mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
        }

        interval_us = int(1e6 / TARGET_RATE_HZ)

        for name, msg_id in messages.items():
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    msg_id,
                    interval_us,
                    0, 0, 0, 0, 0
                )
                print(f"  ✓ Requested {name} at {TARGET_RATE_HZ} Hz")
            except Exception as e:
                print(f"  ✗ Failed to request {name}: {e}")

        # Wait for acknowledgements
        print("  Waiting for ACKs...")
        time.sleep(0.5)

        # Check for command acknowledgements
        ack_count = 0
        timeout = time.time() + 2
        while time.time() < timeout:
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=False, timeout=0.1)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    ack_count += 1

        print(f"  Received {ack_count} ACKs")

    def monitor_messages(self, duration=10):
        """Monitor incoming messages for specified duration"""
        print(f"\n[4/5] Monitoring messages for {duration} seconds...")
        print("  (Showing first occurrence of each message type)\n")

        self.msg_counts = {}
        self.start_time = time.time()
        first_msg_data = {}

        while time.time() - self.start_time < duration:
            msg = self.master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                self.msg_counts[msg_type] = self.msg_counts.get(msg_type, 0) + 1

                # Print details for first occurrence of high-value messages
                if msg_type not in first_msg_data:
                    first_msg_data[msg_type] = True
                    if msg_type in ['ATTITUDE', 'HIGHRES_IMU', 'LOCAL_POSITION_NED',
                                   'GLOBAL_POSITION_INT', 'BATTERY_STATUS']:
                        print(f"  {msg_type}:")
                        print(f"    {msg}")

            time.sleep(0.001)

    def print_summary(self):
        """Print message rate summary"""
        elapsed = time.time() - self.start_time

        print("\n[5/5] Summary")
        print("=" * 60)
        print(f"Message Rates ({elapsed:.1f} sec sample)")
        print("=" * 60)

        # Sort by count (descending)
        sorted_msgs = sorted(self.msg_counts.items(),
                           key=lambda x: x[1],
                           reverse=True)

        for msg_type, count in sorted_msgs:
            hz = count / elapsed
            bar = "█" * min(int(hz / 2), 50)
            print(f"  {msg_type:<30} {count:>6} msgs ({hz:>6.1f} Hz) {bar}")

        print("=" * 60)

        # Check if we got expected messages
        expected = ['HEARTBEAT', 'ATTITUDE', 'HIGHRES_IMU']
        missing = [m for m in expected if m not in self.msg_counts]

        if missing:
            print(f"\n⚠ Missing expected messages: {missing}")
            self._print_troubleshooting()
        else:
            print("\n✓ Connection test SUCCESSFUL!")
            print("\nNext steps:")
            print("  - Run your ROS2 firehose_node")
            print("  - Monitor topics: ros2 topic list")
            print("  - Check rates: ros2 topic hz /mav/imu")

    def _print_troubleshooting(self):
        """Print troubleshooting tips"""
        print("\nTroubleshooting:")
        print("  1. Check physical wiring (TX↔RX, GND↔GND)")
        print("  2. Verify PX4 parameters:")
        print("     param show SER_TEL2_BAUD")
        print("     param show SER_TEL2_PROTO")
        print("  3. Check Jetson UART:")
        print("     ls -l /dev/ttyTHS*")
        print("     systemctl status nvgetty")
        print("  4. Try lower baudrate (57600)")
        print("  5. Test with MAVProxy:")
        print("     mavproxy.py --master=/dev/ttyTHS1,921600")

    def run(self):
        """Run complete test sequence"""
        if not self.connect():
            return False

        self.request_message_intervals()
        self.monitor_messages(duration=10)
        self.print_summary()

        return True


def main():
    tester = PX4Tester(DEVICE, BAUD)
    success = tester.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
