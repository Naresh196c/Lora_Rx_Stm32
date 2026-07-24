# ZED-X20P Base to ZED-F9P Rover over RYLR896 LoRa

This package contains the production code for:

- Base: ZED-X20P -> ESP32 -> RYLR896
- Rover: RYLR896 -> STM32F103 -> ZED-F9P

The rover sends a complete-frame ACK only after the RTCM frame passes CRC-24Q and is written to the F9P UART. The ESP32 retries the same frame when the ACK is missing.

## 1. Files

- `ESP32_X20P_LoRa_Base/ESP32_X20P_LoRa_Base.ino`
- `STM32_F9P_LoRa_Rover_Project/` complete STM32CubeIDE project
- `STM32_F9P_LoRa_Rover/main.c` source-only copy

## 2. Base wiring

### ZED-X20P / EVK-X20P TTL UART to ESP32

| X20P | ESP32 |
|---|---|
| UART TX, 3.3 V TTL | GPIO26 RX |
| GND | GND |

Do not connect an RS-232 DB9 signal directly to the ESP32. Use the EVK's 3.3 V TTL UART pins.

### ESP32 to base RYLR896

| RYLR896 | ESP32 |
|---|---|
| TXD | GPIO16 RX |
| RXD | GPIO17 TX |
| VDD | regulated 3.3 V |
| GND | GND |

Place 100 nF and 10-47 uF capacitors close to the RYLR896 supply.

## 3. Rover wiring

### Rover RYLR896 to STM32F103

| RYLR896 | STM32F103 |
|---|---|
| TXD | PA3 / USART2 RX |
| RXD | PA2 / USART2 TX |
| VDD | regulated 3.3 V |
| GND | GND |

### STM32F103 to ZED-F9P

| STM32F103 | ZED-F9P |
|---|---|
| PA9 / USART1 TX | F9P UART RX |
| GND | GND |

PA9 carries binary RTCM only. Do not connect a text serial monitor to PA9 during live operation.

## 4. Matching LoRa settings

Both projects use:

- Base address: 2
- Rover address: 1
- Network ID: 5
- Frequency: 866000000 Hz
- Module UART: 115200 baud
- RF parameter: `AT+PARAMETER=7,9,1,4`
- RF power: 15 dBm

The RF parameter above is the highest-throughput laboratory setting used in this project. For a more robust but slower 125 kHz link, change both projects to:

`AT+PARAMETER=7,7,1,4`

Never change only one side.

## 5. Configure the ZED-X20P base

Use u-center 2.

### Base timing mode

For initial testing:

- `CFG-TMODE-MODE = SURVEY_IN`
- `CFG-TMODE-SVIN_MIN_DUR = 300`
- `CFG-TMODE-SVIN_ACC_LIMIT = 1 to 2 m`

For repeatable absolute waypoints, enter accurately surveyed base coordinates and use fixed mode.

### X20P UART connected to ESP32

- Baud: 115200
- RTCM3X output: enabled
- NMEA output: disabled on this UART
- UBX output: disabled on this UART

### Recommended first RTCM profile

Start with:

- RTCM 1005: every 10 seconds
- RTCM 1074 GPS MSM4: 1 Hz
- RTCM 1094 Galileo MSM4: 1 Hz

After stable RTK operation, add:

- RTCM 1084 GLONASS MSM4: 1 Hz
- RTCM 1230: every 10 seconds
- RTCM 1124 BeiDou MSM4: 1 Hz

Do not enable NMEA or UBX on the same X20P UART. Do not mix MSM4 and MSM7 for the observation stream. MSM4 is strongly recommended for this RYLR896 link.

## 6. Configure the ZED-F9P rover

On the F9P UART connected to STM32 PA9:

- Baud: 115200
- RTCM3X input: enabled
- 8 data bits, no parity, 1 stop bit

The F9P can use UART2 as the correction input. UART1 can also be used if configured correctly.

## 7. Flashing

### ESP32

1. Open `ESP32_X20P_LoRa_Base.ino` in Arduino IDE.
2. Select your ESP32 board.
3. Upload.
4. Open Serial Monitor at 115200.

Expected successful messages:

`RTCM 1074 ACK seq=0021 len=... queue=...`

### STM32

1. Import `STM32_F9P_LoRa_Rover_Project` into STM32CubeIDE.
2. Build the project.
3. Flash using ST-Link.
4. The project keeps Serial Wire Debug enabled.

At startup PC13 blinks three times. During operation PC13 toggles every time a CRC-valid RTCM frame is written to the F9P.

## 8. Reliability behavior

- Every RTCM frame is CRC-checked at the base.
- A frame is split into up to eight fragments.
- The rover accepts fragments in any order and ignores duplicates.
- The rover validates the reassembled RTCM CRC.
- The rover writes the complete binary frame to the F9P.
- The rover sends `Axxxx` ACK after successful injection.
- The base retries the complete frame up to three times if the ACK is missing.
- If an ACK was lost, the rover recognizes the duplicate sequence and re-ACKs without injecting the same RTCM frame twice.
- The ESP32 replaces older queued RTCM frames of the same type with newer data to prevent stale correction backlog.

## 9. Troubleshooting

### Base shows no RTCM frames

- Verify X20P UART baud is 115200.
- Verify RTCM output is enabled on the exact UART connected to GPIO26.
- Verify common ground.
- Verify you are using 3.3 V TTL UART, not RS-232.

### Base shows `ACK timeout`

- Check both LoRa addresses, network ID, frequency, and RF parameters.
- Check antennas and 3.3 V supply stability.
- Check PA2/PA3 wiring.
- Confirm STM32 startup LED blinks three times.

### PC13 toggles but F9P remains standalone

- Check PA9 is connected to F9P UART RX, not TX.
- Enable RTCM3X input on that F9P UART.
- Set the F9P UART to 115200.
- Verify the base has completed survey-in or uses valid fixed coordinates.
- Verify the X20P outputs observation messages compatible with the F9P.

### Many queue replacements or failed frames

The RTCM stream is larger than the LoRa link capacity. Reduce the X20P RTCM output to GPS + Galileo MSM4 at 1 Hz first. Add other constellations only after the link remains stable.

## 10. Deployment note

The 500 kHz radio setting is intended for throughput testing. Check the applicable Indian WPC requirements for frequency, bandwidth, power, and duty cycle before field deployment.
