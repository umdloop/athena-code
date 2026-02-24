#!/usr/bin/env python3

import can
import time
import argparse
from collections import defaultdict

def monitor_can_rate(interface, channel, arb_id, duration=None):
    """Monitor CAN message rate for a specific arbitration ID."""
    bus = can.interface.Bus(channel=channel, interface=interface)
    
    message_count = 0
    start_time = time.time()
    last_report_time = start_time
    
    print(f"Monitoring CAN ID 0x{arb_id:X} on {interface}:{channel}")
    print("Time (s) | Messages/sec")
    print("-" * 30)
    
    try:
        while True:
            msg = bus.recv(timeout=0.1)
            current_time = time.time()
            
            if msg and msg.arbitration_id == arb_id:
                message_count += 1
            
            # Report every second
            if current_time - last_report_time >= 1.0:
                elapsed = current_time - last_report_time
                rate = message_count / elapsed
                print(f"{current_time - start_time:8.1f} | {rate:12.2f}")
                
                message_count = 0
                last_report_time = current_time
            
            # Check duration limit
            if duration and (current_time - start_time) >= duration:
                break
                
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor CAN message rate for specific arbitration ID")
    parser.add_argument("arb_id", type=lambda x: int(x, 0), help="Arbitration ID (hex or decimal)")
    parser.add_argument("-i", "--interface", default="socketcan", help="CAN interface type (default: socketcan)")
    parser.add_argument("-c", "--channel", default="can0", help="CAN channel (default: can0)")
    parser.add_argument("-d", "--duration", type=float, help="Duration in seconds (optional)")
    
    args = parser.parse_args()
    monitor_can_rate(args.interface, args.channel, args.arb_id, args.duration)
