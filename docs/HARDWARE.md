# Hardware Documentation

## Bill of Materials (BOM)

### Transmitter (TX)
| Component | Part Number | Quantity | Notes |
|-----------|-------------|----------|-------|
| Microcontroller | ESP32 Dev Board | 1 | 240MHz dual-core |
| Pressure Sensor | BMP280 | 1 | I2C, 0x76 or 0x77 |
| IMU | LSM9DS1 | 1 | 9-DOF, I2C |
| GNSS | LC76G | 1 | Multi-constellation |
| LoRa Module | SX1278 (Ra-02) | 1 | 433 MHz |
| Antenna | 433MHz | 1 | Matched impedance |

### Receiver (RX)
| Component | Part Number | Quantity | Notes |
|-----------|-------------|----------|-------|
| Microcontroller | ESP32 Dev Board | 1 | Any generic board |
| LoRa Module | SX1278 (Ra-02) | 1 | 433 MHz |
| Antenna | 433MHz | 1 | Matched impedance |

## Wiring Diagrams

### TX I2C Bus
```
ESP32               BMP280
GPIO21 (SDA) -----> SDA
GPIO22 (SCL) -----> SCL
3.3V -------------> VCC
GND --------------> GND

ESP32               LSM9DS1
GPIO21 (SDA) -----> SDA (both AG and M)
GPIO22 (SCL) -----> SCL (both AG and M)
3.3V -------------> VCC
GND --------------> GND
```

### TX/RX LoRa SPI
```
ESP32               SX1278
GPIO23 -----------> MOSI
GPIO19 -----------> MISO
GPIO18 -----------> SCK
GPIO25 -----------> NSS/CS
GPIO14 -----------> RST
GPIO4  -----------> DIO0
3.3V -------------> VCC
GND --------------> GND
```

### TX GNSS UART
```
ESP32               LC76G
GPIO16 (RX1) -----> TX
GPIO17 (TX1) -----> RX
3.3V -------------> VCC
GND --------------> GND
```

## Assembly Notes

1. **I2C Pull-ups**: Most breakout boards include pull-ups. If using bare chips, add 4.7kΩ resistors to SDA/SCL.

2. **Power Supply**: Use a stable 3.3V regulator. Total current draw:
   - ESP32: ~80-200mA
   - Sensors: ~20-50mA
   - LoRa TX: up to 120mA during transmission

3. **Antenna**: Use proper 433MHz antenna. Avoid operating without antenna (can damage LoRa module).

4. **Mounting**: Secure all components. Consider vibration damping for flight.

## PCB Design (Future)

Consider custom PCB with:
- Integrated voltage regulation
- Protection circuitry
- Optimized antenna routing
- Compact form factor

## Testing Checklist

- [ ] All I2C devices detected
- [ ] GPS gets fix outdoors
- [ ] LoRa link established
- [ ] Sensors read valid data
- [ ] Filesystem accessible
- [ ] Logging works
- [ ] Range test passed
