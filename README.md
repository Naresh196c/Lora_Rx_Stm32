# STM32 LoRa RTCM Receiver for ZED-F9P Rover

STM32F103 rover receiver firmware for receiving RTCM3 correction messages over a REYAX RYLR896 LoRa module and forwarding valid binary RTCM to a ZED-F9P rover.

## Description

This project is the rover-side receiver for an RTK correction link. It receives Base64-encoded RTCM fragments from an ESP32 LoRa transmitter, reassembles complete RTCM3 frames, validates the RTCM CRC-24Q, and writes only valid binary RTCM to the ZED-F9P UART.

The current firmware is configured for one-way RTCM streaming. The rover receives and injects RTCM into the F9P, but it does not send ACK packets back to the base.

## Hardware

- STM32F103C8T6 board, commonly Blue Pill
- ZED-F9P GNSS receiver configured as RTK rover
- REYAX RYLR896 LoRa module
- 3.3 V power supply
- Common ground between STM32, RYLR896, and F9P

## Data Flow

```text
ESP32 base -> RYLR896 LoRa -> STM32 USART2 -> RTCM validation -> STM32 USART1 PA9 -> ZED-F9P RX
```

## Wiring

### RYLR896 to STM32

| RYLR896 | STM32F103 |
| --- | --- |
| TXD | PA3 / USART2 RX |
| RXD | PA2 / USART2 TX |
| VCC | 3.3 V |
| GND | GND |

### STM32 to ZED-F9P

| STM32F103 | ZED-F9P |
| --- | --- |
| PA9 / USART1 TX | F9P UART RX |
| GND | GND |

PA9 carries binary RTCM only. If you open PA9 in PuTTY or another serial terminal, it will look like garbage characters. That is normal for binary RTCM.

## Default Settings

| Setting | Value |
| --- | --- |
| RYLR896 UART baud | 115200 |
| F9P RTCM UART baud | 115200 |
| Base address | 2 |
| Rover address | 1 |
| Network ID | 5 |
| LoRa frequency | 866000000 Hz |
| LoRa RF parameter | `AT+PARAMETER=7,9,1,4` |
| LoRa RF power | `AT+CRFOP=15` |
| Rover ACK | Disabled |
| PA9 debug HEX | Disabled |

The matching ESP32 transmitter must use the same address, network ID, frequency, and RF parameter.

## Important Build Options

In `Core/Src/main.c`:

```c
#define DEBUG_RTCM_HEX_ON_USART1 0U
#define SEND_BASE_ACK            0U
```

For real F9P correction, keep `DEBUG_RTCM_HEX_ON_USART1` set to `0U`. If it is set to `1U`, PA9 outputs readable HEX text for debugging and the F9P will not receive valid RTCM.

## STM32CubeIDE Setup

1. Open STM32CubeIDE.
2. Import this folder as an existing STM32 project.
3. Build the project.
4. Flash the STM32F103 using ST-Link.
5. Confirm the PC13 LED blinks at startup.

During operation, PC13 toggles whenever a CRC-valid RTCM frame is written to the F9P UART.

## F9P Rover Configuration

On the F9P UART connected to STM32 PA9:

- Baud rate: `115200`
- Protocol input: RTCM3 enabled
- Common ground connected to STM32

In u-center, check RTCM input counters or the RTCM message view. If direct X20P-to-F9P works but STM32-to-F9P does not, connect STM32 PA9 to the same F9P RX pin and UART that worked during the direct test.

## Expected Behavior

When valid LoRa RTCM frames are received:

- STM32 reassembles all fragments for a frame.
- STM32 validates RTCM CRC-24Q.
- STM32 writes the binary RTCM frame to USART1 / PA9.
- PC13 toggles once per valid injected frame.
- No ACK is sent back to the ESP32 base.

## Troubleshooting

### F9P does not show RTCM input

Check:

- STM32 PA9 is connected to F9P UART RX, not TX
- STM32 GND and F9P GND are connected
- F9P UART baud is 115200
- RTCM3 input is enabled on that F9P UART
- `DEBUG_RTCM_HEX_ON_USART1` is `0U`
- PC13 toggles when LoRa RTCM is received

### PuTTY shows garbage on PA9

That is expected. PA9 outputs binary RTCM for the F9P. Binary RTCM is not readable text.

### PC13 does not toggle

The STM32 is not receiving or validating complete RTCM frames. Check:

- RYLR896 wiring on PA2/PA3
- LoRa address, network ID, frequency, and RF parameter
- Both LoRa antennas
- ESP32 transmitter Serial Monitor for `RTCM ... sent`
- RTCM output enabled on the base GNSS receiver

### Link is unstable

For more range or robustness, use this RF parameter on both transmitter and receiver:

```text
AT+PARAMETER=7,7,1,4
```

The RTCM data rate must fit within the LoRa link capacity. Start with a small RTCM profile such as 1005/1006 plus GPS/Galileo MSM4 at 1 Hz, then add more messages only after the link is stable.
