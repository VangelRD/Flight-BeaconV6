# Flight Log Extraction Guide

## What Was Changed

### TX Code Changes ✅
1. **Logging enabled** - Now logs at 100Hz to `/flight_log.csv` on LittleFS
2. **Serial commands added** - Type commands to interact with TX
3. **Log dump function** - Streams entire log file over serial

### Available Commands
Connect to TX via serial monitor and type:
- `DUMP_LOG` - Download the flight log
- `HELP` - Show available commands

## How to Extract Logs

### Method 1: Using Python Script (Easiest) ⭐

After your flight:

```bash
# Install pyserial if needed
pip install pyserial

# Extract logs (auto-detects TX)
python3 extract_logs.py

# Or specify port
python3 extract_logs.py /dev/ttyUSB0 115200
```

The script will:
- Connect to TX
- Request log dump
- Save to `flight_log_YYYYMMDD_HHMMSS.csv`
- Show progress and statistics

### Method 2: Manual via Serial Monitor

1. Open serial monitor (PlatformIO, screen, minicom, etc.):
   ```bash
   screen /dev/ttyUSB0 115200
   # or
   pio device monitor -b 115200
   ```

2. Type: `DUMP_LOG` and press Enter

3. Wait for output between `[DATA_START]` and `[DATA_END]`

4. Copy and paste the CSV data to a file

5. Exit serial monitor:
   - screen: Press `Ctrl+A` then `K` then `Y`
   - pio: Press `Ctrl+C`

### Method 3: Using screen with automatic capture

```bash
# Start screen with logging
screen -L -Logfile flight_log.txt /dev/ttyUSB0 115200

# Type: DUMP_LOG
# Exit screen: Ctrl+A, K, Y

# Extract CSV from log file
grep -A 999999 '\[DATA_START\]' flight_log.txt | grep -B 999999 '\[DATA_END\]' | grep -v '\[DATA' > flight_log.csv
```

## Log File Format

CSV with 18 columns:
```
time,temp,pressure,alt,ax,ay,az,gx,gy,gz,mx,my,mz,lat,lon,galt,spd,sats
```

- `time` - Timestamp (ms since boot)
- `temp` - Temperature (°C)
- `pressure` - Pressure (Pa)
- `alt` - Calculated altitude (m)
- `ax,ay,az` - Acceleration (m/s²)
- `gx,gy,gz` - Gyroscope (rad/s)
- `mx,my,mz` - Magnetometer (µT)
- `lat,lon` - GPS coordinates
- `galt` - GPS altitude (m)
- `spd` - GPS speed (m/s)
- `sats` - GPS satellites

## Data Analysis Examples

### Python/Pandas
```python
import pandas as pd
import matplotlib.pyplot as plt

# Load log
df = pd.read_csv('flight_log_20251024_143000.csv')

# Convert time to seconds
df['time_s'] = df['time'] / 1000

# Plot altitude
plt.figure(figsize=(12, 6))
plt.plot(df['time_s'], df['alt'])
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.title('Flight Altitude Profile')
plt.grid(True)
plt.show()

# Plot acceleration
plt.figure(figsize=(12, 6))
plt.plot(df['time_s'], df['az'], label='Vertical (Z)')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s²)')
plt.title('Vertical Acceleration')
plt.legend()
plt.grid(True)
plt.show()

# Find max altitude
max_alt = df['alt'].max()
max_alt_time = df.loc[df['alt'].idxmax(), 'time_s']
print(f"Max altitude: {max_alt:.1f}m at {max_alt_time:.1f}s")
```

### LibreOffice Calc / Excel
1. Open the CSV file
2. Select data → Create Chart
3. Choose line chart
4. Set X-axis to `time` or `time_s`
5. Plot `alt`, `az`, `pressure`, etc.

## Troubleshooting

### "No data received"
- Check TX is powered on
- Verify correct port: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
- Try reconnecting USB cable
- Close other serial programs (Arduino IDE, screen, etc.)

### "Permission denied"
```bash
# Add yourself to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

### "Port in use"
```bash
# Find what's using it
lsof | grep ttyUSB0
# Kill the process
kill <PID>
```

### Log file empty
- Check if filesystem mounted (look for `[FS] Filesystem ready` in boot log)
- Verify logging was enabled and running (check TX status output)

## Storage Capacity

With 100Hz logging and ~100 bytes per line:
- **10 seconds** → ~100KB
- **60 seconds** → ~600KB
- **5 minutes** → ~3MB

ESP32 LittleFS partition: typically 1-4MB depending on configuration.

**Tip**: For long flights, consider reducing log rate or adding SD card.

