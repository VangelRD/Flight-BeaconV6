# V-Flight Computer - Quick Start Guide

## Hardware Setup

### Transmitter (TX)
1. Connect sensors to ESP32:
   - **I2C Bus** (SDA=GPIO21, SCL=GPIO22):
     - BMP280 pressure sensor
     - LSM9DS1 IMU (accel/gyro + magnetometer)
   - **UART1** (RX=GPIO16, TX=GPIO17):
     - LC76G GNSS module (TX→GPIO16, RX→GPIO17)
   - **SPI** (for LoRa):
     - MOSI→GPIO23, MISO→GPIO19, SCK→GPIO18
     - CS→GPIO25, RST→GPIO14, DIO0→GPIO4

2. Power with stable 3.3V supply

### Receiver (RX)
1. Connect LoRa module (same SPI pins as TX)
2. Power with USB or stable supply

## Software Setup

### Install PlatformIO
```bash
# VS Code extension (recommended)
# OR CLI:
pip install platformio
```

### Build and Upload

#### Transmitter
```bash
cd tx/
pio run -t upload
pio device monitor -b 115200
```

#### Receiver
```bash
cd rx/
pio run -t upload
pio device monitor -b 115200
```

## Usage

1. **Power up TX** - Wait for sensor initialization (check serial output)
2. **Power up RX** - Should start receiving packets within seconds
3. **Monitor serial** - RX displays live telemetry at ~5Hz
4. **Download logs** - Both TX and RX log to LittleFS filesystem

## Data Rates

- **TX Local Logging**: 100Hz (all sensors)
- **LoRa Transmission**: 5Hz (32-byte packets)
- **Range**: 2+ km line-of-sight (SF9, BW125kHz)

## Retrieving Logs

Use PlatformIO's filesystem upload tool or ESP32 filesystem browser:

```bash
# List files
pio run -t uploadfs

# Or use serial file transfer tools
```

Log files:
- TX: `/flight_log.csv`
- RX: `/rx_log_[timestamp].csv`

## Troubleshooting

### No LoRa Link
- Check antenna connections
- Verify both devices use same frequency/settings
- Ensure continuous RX mode (RX should call `LoRa.receive()`)

### Sensors Not Found
- Check I2C wiring and pull-ups
- Try I2C scanner to detect addresses
- Verify power supply (3.3V stable)

### GPS No Fix
- Move outdoors with clear sky view
- Wait 30-60 seconds for cold start
- Check baud rate (9600 for LC76G)

## Known Issues

See README.md "Common issues" section:
- LoRa packets too large → Reduced to 32 bytes
- RX continuous mode → Fixed with `LoRa.receive()` in callback
- Settings mismatch → Both use SF9, BW125, CR4/6

## Next Steps

- Bench test all sensors before flight
- Test LoRa range on ground
- Verify log file integrity
- Check flash capacity for flight duration
