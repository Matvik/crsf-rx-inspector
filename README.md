# CRSF RX Inspector

A small Python utility for inspecting CRSF/ELRS traffic from an RX connected through a serial port, including Betaflight passthrough.

## What it does

- scans available COM ports
- lets you choose the baud rate
- opens the serial connection
- listens for CRSF frames
- validates sync, length and CRC
- decodes supported CRSF/ELRS frames
- prints Device Info and ELRS status in English
- avoids inventing a `Target` value when the RX does not actually send one

## Supported frame categories

The parser is intentionally conservative and only handles frame types that are actually documented and validated:

- `0x29` — `DEVICE_INFO`
- `0x14` — `LINK_STATISTICS`
- `0x2E` — `ELRS_STATUS` (when present)
- other CRSF/ELRS frames are ignored unless explicitly supported

## Requirements

Install the Python serial library:

```bash
pip install pyserial
```

## Running the script

From the project folder:

```bash
python crsf_info.py
```

## Connection modes

This script works in two different setups:

### 1) Direct serial connection (no passthrough)

If you connect the RX to a USB-TTL serial adapter directly, you do not need to run any Betaflight passthrough command.

Typical setup:

1. Connect the RX directly to a USB-TTL serial adapter.
2. Plug that adapter into your PC.
3. Start the script.
4. Select the COM port and baud rate.
5. Let it listen for CRSF traffic.

In this mode, do not run `serialpassthrough` at all.

### 2) Through Betaflight / FC passthrough

If the RX is exposed through the flight controller, then you can enable passthrough from Betaflight CLI. In this case, the command is only used inside Betaflight and only when the RX is being exposed via the FC UART.

Examples:

```bash
serialpassthrough 0 420000
```

```bash
serialpassthrough 1 420000
```

Or using the UART name directly, depending on firmware:

```bash
serialpassthrough UART1 420000
```

```bash
serialpassthrough UART2 420000
```

Then:

1. Connect the FC UART/USB serial adapter as required by your setup.
2. Open Betaflight CLI.
3. Run the passthrough command for the correct UART.
4. Start the script.
5. Select the COM port and baud rate.
6. Let it listen for CRSF traffic.

Common baud rates:

- `416666` — CRSF standard
- `420000` — common FC/ELRS setting
- `115200` — fallback option

## Important note

This tool does not invent metadata. It only prints information that is actually present in incoming CRSF/ELRS packets.

For example:

- `Device name` may be present in `DEVICE_INFO`
- `Target` appears only if the RX truly sends target-like information in a supported frame
- if no target-like data is present, the script shows `<not detected>` instead of guessing

## Example output

```text
DEVICE INFO
Device name       : RM DBR4-TD
Destination       : FC (0xC8)
Origin            : RX (0xEC)
Serial number     : 0x454C5253 (1162629715)
Hardware ID       : 0x00000000
Firmware ID       : 0x00030000
Parameters total  : 0
Parameter version : 0
Raw payload       : 52 4D 20 44 42 52 34 2D 54 44 00 45 4C 52 53 00 00 00 00 00 03 00 00 00 00
```

## Notes

- The script is intentionally conservative.
- It only shows fields that are validated and decoded from CRSF packets.
- It avoids fake or guessed values.

## License

MIT
