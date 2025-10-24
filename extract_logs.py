#!/usr/bin/env python3
"""
Extract flight logs from TX via serial
Usage: python3 extract_logs.py [port] [baudrate]
Example: python3 extract_logs.py /dev/ttyUSB0 115200
"""

import serial
import serial.tools.list_ports
import time
import sys
import os
from datetime import datetime

def extract_log(port='/dev/ttyUSB0', baudrate=115200):
    """Extract flight log from ESP32 TX via serial (SPIFFS filesystem)"""
    
    print(f"[*] Connecting to {port} at {baudrate} baud...")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=5)
    except Exception as e:
        print(f"[ERROR] Could not open {port}: {e}")
        print("\nAvailable ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  - {p.device}: {p.description}")
        return False
    
    # Wait for connection to stabilize
    time.sleep(2)
    
    # Clear any pending data
    ser.reset_input_buffer()
    
    print("[*] Requesting log dump...")
    ser.write(b'DUMP_LOG\n')
    ser.flush()
    
    # Wait for response
    time.sleep(1)
    
    # Read until we see the start marker
    print("[*] Waiting for data...")
    in_data_section = False
    log_lines = []
    
    timeout_count = 0
    max_timeouts = 10
    
    while timeout_count < max_timeouts:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if '[LOG_DUMP_START]' in line:
                print("[✓] Log dump started")
            elif '[DATA_START]' in line:
                print("[*] Receiving log data...", end='', flush=True)
                in_data_section = True
            elif '[DATA_END]' in line:
                print("\n[✓] Log data received")
                in_data_section = False
            elif '[LOG_DUMP_END]' in line:
                print("[✓] Log dump complete")
                break
            elif in_data_section:
                log_lines.append(line)
                if len(log_lines) % 100 == 0:
                    print('.', end='', flush=True)
            elif '[INFO]' in line or '[ERROR]' in line:
                print(f"    {line}")
        else:
            time.sleep(0.1)
            timeout_count += 1
    
    ser.close()
    
    if not log_lines:
        print("[ERROR] No log data received")
        return False
    
    # Generate filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"flight_log_{timestamp}.csv"
    
    print(f"[*] Saving {len(log_lines)} lines to {filename}...")
    
    with open(filename, 'w') as f:
        for line in log_lines:
            f.write(line + '\n')
    
    file_size = os.path.getsize(filename)
    print(f"[✓] Log saved: {filename} ({file_size} bytes)")
    print(f"[✓] Total records: {len(log_lines) - 1}")  # -1 for header
    
    return True

def main():
    print("=" * 60)
    print("Flight Log Extractor - V-Flight Computer")
    print("=" * 60)
    
    # Parse arguments
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    success = extract_log(port, baudrate)
    
    if success:
        print("\n[✓] Extraction complete!")
        print("\nYou can now:")
        print("  - Open the CSV in Excel/LibreOffice")
        print("  - Analyze with Python/pandas")
        print("  - Plot with any data visualization tool")
    else:
        print("\n[!] Extraction failed")
        print("\nTroubleshooting:")
        print("  1. Check TX is powered on and connected")
        print("  2. Verify correct port (try: ls /dev/ttyUSB*)")
        print("  3. Ensure no other programs are using the port")
        print("  4. Try different baud rate (e.g., 9600)")
        sys.exit(1)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n[!] Interrupted by user")
        sys.exit(1)

