#!/usr/bin/env python3

import can
import time
import argparse
from collections import defaultdict

def monitor_can_ids(interface, channel, duration=None):
    """Monitor all CAN message IDs and their rates."""
    bus = can.interface.Bus(channel=channel, interface=interface)
    
    id_counts = defaultdict(int)
    total_counts = defaultdict(int)
    start_time = time.time()
    last_report_time = start_time
    
    print(f"Monitoring CAN IDs on {interface}:{channel}")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            msg = bus.recv(timeout=0.1)
            current_time = time.time()
            
            if msg:
                id_counts[msg.arbitration_id] += 1
                total_counts[msg.arbitration_id] += 1
            
            # Report every second
            if current_time - last_report_time >= 1.0:
                elapsed = current_time - last_report_time
                print(f"\n--- Time: {current_time - start_time:.1f}s ---")
                print(f"{'CAN ID':<12} | {'Messages/sec':<15} | {'Total'}")
                print("-" * 50)
                
                for arb_id in sorted(id_counts.keys()):
                    rate = id_counts[arb_id] / elapsed
                    print(f"0x{arb_id:08X} | {rate:15.2f} | {id_counts[arb_id]}")
                
                id_counts.clear()
                last_report_time = current_time
            
            # Check duration limit
            if duration and (current_time - start_time) >= duration:
                break
                
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    finally:
        bus.shutdown()
        
        # Print final summary if duration was specified
        if duration:
            total_elapsed = time.time() - start_time
            print(f"\n{'='*60}")
            print(f"SUMMARY - Total Duration: {total_elapsed:.1f}s")
            print(f"{'='*60}")
            print(f"{'CAN ID':<12} | {'Avg Rate (msg/s)':<18} | {'Total Messages'}")
            print("-" * 60)
            
            for arb_id in sorted(total_counts.keys()):
                avg_rate = total_counts[arb_id] / total_elapsed
                print(f"0x{arb_id:08X} | {avg_rate:18.2f} | {total_counts[arb_id]}")
            
            print(f"{'='*60}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor all CAN message IDs and their rates")
    parser.add_argument("-i", "--interface", default="socketcan", help="CAN interface type (default: socketcan)")
    parser.add_argument("-c", "--channel", default="can0", help="CAN channel (default: can0)")
    parser.add_argument("-d", "--duration", type=float, help="Duration in seconds (optional)")
    
    args = parser.parse_args()
    monitor_can_ids(args.interface, args.channel, args.duration)
