import serial
import serial.tools.list_ports
import time
import struct
import sys


# ============================================================
# CRSF CONSTANTS
# ============================================================

CRSF_SYNC = 0xC8

CRSF_FRAMETYPE_GPS = 0x02
CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08
CRSF_FRAMETYPE_LINK_STATISTICS = 0x14
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_FRAMETYPE_ATTITUDE = 0x1E
CRSF_FRAMETYPE_FLIGHT_MODE = 0x21
CRSF_DEVICE_PING = 0x28
CRSF_DEVICE_INFO = 0x29
CRSF_DEVICE_SETTINGS_ENTRY = 0x2B
CRSF_DEVICE_SETTINGS_READ = 0x2C
CRSF_DEVICE_SETTINGS_WRITE = 0x2D
CRSF_ELRS_STATUS = 0x2E
CRSF_COMMAND = 0x32
CRSF_MSP_REQ = 0x7A
CRSF_MSP_RESP = 0x7B
CRSF_MSP_WRITE = 0x7C
CRSF_DISPLAYPORT_CMD = 0x7D

CRSF_ADDRESS_BROADCAST = 0x00
CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_ADDRESS_RECEIVER = 0xEC

# CRSF standard UART speed
CRSF_BAUD_STANDARD = 416666

# Common FC/ELRS value
CRSF_BAUD_420K = 420000


# ============================================================
# CRC8-D5
# ============================================================

def crc8(data):
    crc = 0

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF

    return crc


# ============================================================
# HELPERS
# ============================================================

def hexstr(data):
    return " ".join(f"{x:02X}" for x in data)


def warn(message):
    print(f"[WARN] {message}")


def error(message):
    print(f"[ERROR] {message}")


def printable_ascii(data, min_len=4):
    """Return printable ASCII chunks found in raw bytes."""
    chunks = []
    current = []

    for byte in data:
        if 32 <= byte <= 126:
            current.append(chr(byte))
        else:
            if len(current) >= min_len:
                chunks.append("".join(current))
            current = []

    if len(current) >= min_len:
        chunks.append("".join(current))

    return chunks


def describe_address(value):
    labels = {
        0x00: "BROADCAST",
        0xC8: "FC",
        0xEC: "RX",
    }
    return labels.get(value, f"0x{value:02X}")


def extract_printable_strings(data, min_len=3):
    strings = []
    current = []

    for byte in data:
        if 32 <= byte <= 126:
            current.append(chr(byte))
        else:
            if len(current) >= min_len:
                strings.append("".join(current))
            current = []

    if len(current) >= min_len:
        strings.append("".join(current))

    return strings


def decode_u32_be(data, offset):
    if offset + 4 > len(data):
        return None
    return struct.unpack(">I", data[offset:offset + 4])[0]


# ============================================================
# CRSF FRAME CREATION
# ============================================================

def make_extended_frame(frame_type, destination, origin, payload=b""):
    """
    CRSF extended frame:

        Sync
        Length
        Type
        Destination
        Origin
        Payload
        CRC

    Frame length excludes Sync + Length.
    """

    body = bytes([
        frame_type,
        destination,
        origin
    ]) + payload

    crc = crc8(body)

    frame_length = len(body) + 1

    frame = bytes([
        CRSF_SYNC,
        frame_length
    ]) + body + bytes([crc])

    return frame


def make_device_ping(destination):
    """
    DEVICE_PING 0x28

    According to CRSF:
        Type + Destination + Origin + CRC
        = 4 bytes
    """

    return make_extended_frame(
        CRSF_DEVICE_PING,
        destination,
        CRSF_ADDRESS_FLIGHT_CONTROLLER
    )


# ============================================================
# CRSF FRAME PARSER
# ============================================================

def parse_frame(frame):

    if len(frame) < 4:
        return

    if frame[0] != CRSF_SYNC:
        error(f"Wrong sync: {frame[0]:02X}")
        return

    expected_length = frame[1]
    actual_length = len(frame) - 2

    if expected_length != actual_length:
        error(f"Length mismatch: field={expected_length} actual={actual_length}")
        return

    received_crc = frame[-1]
    calculated_crc = crc8(frame[2:-1])

    if received_crc != calculated_crc:
        error(f"CRC mismatch: RX={received_crc:02X} CALC={calculated_crc:02X}")
        return

    frame_type = frame[2]
    payload = frame[5:-1]

    if frame_type == CRSF_DEVICE_INFO:
        info = parse_device_info(frame)
        if info is not None:
            print_device_info(info)

    elif frame_type == CRSF_ELRS_STATUS:
        parse_elrs_status(frame)

    elif frame_type == CRSF_FRAMETYPE_LINK_STATISTICS:
        parse_link_statistics(frame)

    elif frame_type in (CRSF_MSP_REQ, CRSF_MSP_RESP, CRSF_MSP_WRITE, CRSF_DISPLAYPORT_CMD):
        pass


# ============================================================
# DEVICE INFO
# ============================================================

def parse_device_info(frame):

    if len(frame) < 8:
        return None

    if frame[2] != CRSF_DEVICE_INFO:
        return None

    destination = frame[3]
    origin = frame[4]

    payload = frame[5:-1]

    # Device name is null terminated.
    # Some devices may have a different layout or no zero terminator,
    # so we degrade gracefully instead of failing outright.

    device_name = ""
    if payload:
        zero = payload.find(b"\x00")
        if zero >= 0:
            device_name = payload[:zero].decode("ascii", errors="replace").strip()
        else:
            ascii_name = printable_ascii(payload, min_len=3)
            if ascii_name:
                device_name = ascii_name[0]

    pos = payload.find(b"\x00") + 1 if b"\x00" in payload else 0
    if pos == 0 and payload:
        pos = max(0, len(payload) - 14)

    # serial 4
    # hardware 4
    # firmware 4
    # parameters total 1
    # parameter version 1

    if len(payload) < pos + 14:
        return {
            "destination": destination,
            "origin": origin,
            "device_name": device_name,
            "serial_number": None,
            "hardware_id": None,
            "firmware_id": None,
            "parameters_total": None,
            "parameter_version": None,
            "payload_hex": hexstr(payload),
        }

    serial_number = decode_u32_be(payload, pos)
    pos += 4

    hardware_id = decode_u32_be(payload, pos)
    pos += 4

    firmware_id = decode_u32_be(payload, pos)
    pos += 4

    parameters_total = payload[pos] if pos < len(payload) else None
    pos += 1

    parameter_version = payload[pos] if pos < len(payload) else None

    return {
        "destination": destination,
        "origin": origin,
        "device_name": device_name,
        "target": None,
        "serial_number": serial_number,
        "hardware_id": hardware_id,
        "firmware_id": firmware_id,
        "parameters_total": parameters_total,
        "parameter_version": parameter_version,
        "payload_hex": hexstr(payload),
    }


def print_device_info(info):

    device_name = info.get('device_name') or '<unknown>'

    print()
    print("=" * 60)
    print("DEVICE INFO")
    print(f"Device name       : {device_name}")
    print(f"Destination       : {describe_address(info['destination'])} (0x{info['destination']:02X})")
    print(f"Origin            : {describe_address(info['origin'])} (0x{info['origin']:02X})")

    if info['serial_number'] is not None:
        print(f"Serial number     : 0x{info['serial_number']:08X} ({info['serial_number']})")
    if info['hardware_id'] is not None:
        print(f"Hardware ID       : 0x{info['hardware_id']:08X}")
    if info['firmware_id'] is not None:
        print(f"Firmware ID       : 0x{info['firmware_id']:08X}")
    if info['parameters_total'] is not None:
        print(f"Parameters total  : {info['parameters_total']}")
    if info['parameter_version'] is not None:
        print(f"Parameter version : {info['parameter_version']}")
    if info['payload_hex']:
        print(f"Raw payload       : {info['payload_hex']}")

    print("=" * 60)
    print()


# ============================================================
# LINK STATISTICS
# ============================================================

def parse_link_statistics(frame):
    payload = frame[5:-1]
    if len(payload) < 10:
        return

    uplink_rssi_1 = payload[0]
    uplink_rssi_2 = payload[1]
    uplink_link_quality = payload[2]
    uplink_snr = struct.unpack("<b", bytes([payload[3]]))[0]
    active_antenna = payload[4]
    rf_mode = payload[5]
    uplink_tx_power = payload[6]
    downlink_rssi = payload[7]
    downlink_link_quality = payload[8]
    downlink_snr = struct.unpack("<b", bytes([payload[9]]))[0]

    rf_mode_label = {
        0: "4 FPS",
        1: "50 FPS",
        2: "150 Hz",
    }.get(rf_mode, f"unknown ({rf_mode})")

    print()
    print("=" * 60)
    print("CRSF LINK STATISTICS")
    print(f"Uplink RSSI 1      : {uplink_rssi_1} dBm * -1")
    print(f"Uplink RSSI 2      : {uplink_rssi_2} dBm * -1")
    print(f"Uplink quality     : {uplink_link_quality}%")
    print(f"Uplink SNR         : {uplink_snr} dB")
    print(f"Active antenna     : {active_antenna}")
    print(f"RF mode            : {rf_mode_label}")
    print(f"Uplink TX power    : {uplink_tx_power}")
    print(f"Downlink RSSI      : {downlink_rssi} dBm * -1")
    print(f"Downlink quality   : {downlink_link_quality}%")
    print(f"Downlink SNR       : {downlink_snr} dB")
    print(f"Raw payload        : {hexstr(payload)}")
    print("=" * 60)
    print()


# ============================================================
# ELRS STATUS
# ============================================================

def extract_target_name(payload):
    strings = extract_printable_strings(payload, min_len=3)
    if not strings:
        return None

    priority = [
        "TARGET",
        "ELRS",
        "BETAFPV",
        "FR",
        "FM",
        "RX",
        "TX",
        "VTX",
    ]

    for token in priority:
        for value in strings:
            if token in value.upper():
                return value

    return strings[0]


def decode_elrs_status_payload(payload):
    info = {
        "target": None,
        "printable": [],
        "status_flags": None,
        "model_id": None,
        "firmware_version": None,
        "package_version": None,
        "raw_hex": hexstr(payload),
    }

    printable = extract_printable_strings(payload, min_len=3)
    info["printable"] = printable
    info["target"] = extract_target_name(payload)

    if len(payload) >= 1:
        info["status_flags"] = payload[0]

    if len(payload) >= 2:
        info["model_id"] = payload[1]

    if len(payload) >= 4:
        info["firmware_version"] = struct.unpack("<H", payload[2:4])[0]

    if len(payload) >= 6:
        info["package_version"] = struct.unpack("<H", payload[4:6])[0]

    return info


def parse_elrs_status(frame):

    payload = frame[5:-1]
    status = decode_elrs_status_payload(payload)
    target_value = status["target"] or "<not detected>"

    print()
    print("=" * 60)
    print("ELRS STATUS")
    print(f"Target            : {target_value}")

    if status["printable"]:
        print(f"Printable text    : {', '.join(status['printable'])}")
    if status["status_flags"] is not None:
        print(f"Status flags      : 0x{status['status_flags']:02X}")
    if status["model_id"] is not None:
        print(f"Model ID          : {status['model_id']}")
    if status["firmware_version"] is not None:
        print(f"Firmware version  : {status['firmware_version']}")
    if status["package_version"] is not None:
        print(f"Package version   : {status['package_version']}")
    print(f"Length            : {len(payload)} bytes")
    print(f"Raw payload       : {status['raw_hex']}")
    print("=" * 60)
    print()


# ============================================================
# CRSF STREAM READER
# ============================================================

def process_buffer(buffer):

    frames = []

    while True:

        # ----------------------------------------------------
        # Find CRSF sync
        # ----------------------------------------------------

        try:
            index = buffer.index(CRSF_SYNC)

        except ValueError:

            # No sync byte.
            #
            # Keep nothing because there is no valid beginning
            # of a CRSF frame.

            buffer.clear()

            break

        # Remove garbage before sync

        if index > 0:
            warn(f"RX garbage before sync: {hexstr(buffer[:index])}")
            del buffer[:index]

        # Need sync + length

        if len(buffer) < 2:
            break

        frame_length = buffer[1]

        # Valid CRSF length is 2..62

        if frame_length < 2 or frame_length > 62:
            warn(f"Invalid frame length: {frame_length:02X}")
            del buffer[0]
            continue

        total_length = frame_length + 2

        if len(buffer) < total_length:
            break

        frame = bytes(
            buffer[:total_length]
        )

        del buffer[:total_length]

        frames.append(frame)

    return frames


# ============================================================
# WAIT FOR RESPONSE
# ============================================================

def listen_for_response(
    ser,
    buffer,
    timeout=3.0
):

    start = time.time()

    received_anything = False
    valid_frames = 0

    while time.time() - start < timeout:

        data = ser.read(
            ser.in_waiting or 1
        )

        if not data:
            continue

        received_anything = True

        buffer.extend(data)

        frames = process_buffer(buffer)

        for frame in frames:
            valid_frames += 1
            parse_frame(frame)

    return received_anything, valid_frames


# ============================================================
# SEND AND LISTEN
# ============================================================

def send_test(
    ser,
    buffer,
    frame,
    listen_time=3.0
):

    if ser.in_waiting:
        old = ser.read(ser.in_waiting)
        if old:
            buffer.extend(old)
            for old_frame in process_buffer(buffer):
                parse_frame(old_frame)

    ser.write(frame)
    ser.flush()

    listen_for_response(
        ser,
        buffer,
        listen_time
    )


# ============================================================
# PORT SELECTION
# ============================================================

ports = list(
    serial.tools.list_ports.comports()
)

if not ports:
    print("No COM ports found.")
    input("Press Enter...")
    sys.exit()


print()
print("=" * 78)
print("                    AVAILABLE COM PORTS")
print("=" * 78)
print()

for i, port in enumerate(ports, 1):

    print(
        f"{i}. {port.device:<8} "
        f"- {port.description}"
    )

print()

while True:

    try:

        choice = int(
            input("Select COM port number: ")
        )

        if 1 <= choice <= len(ports):
            break

    except ValueError:
        pass

    print("Invalid selection.")


port_name = ports[
    choice - 1
].device


# ============================================================
# BAUDRATE
# ============================================================

print()
print("=" * 78)
print("                         BAUDRATE")
print("=" * 78)
print()

print("1. 416666  (CRSF standard)")
print("2. 420000  (common FC setting)")
print("3. 115200")
print("4. Enter manually")

print()

while True:

    choice = input("Choice: ").strip()

    if choice == "1":

        baudrate = 416666
        break

    elif choice == "2":

        baudrate = 420000
        break

    elif choice == "3":

        baudrate = 115200
        break

    elif choice == "4":

        try:

            baudrate = int(
                input(
                    "Baudrate: "
                )
            )

            break

        except ValueError:

            print("Invalid baudrate.")

    else:
        print("Enter 1, 2, 3 or 4.")


# ============================================================
# OPEN SERIAL
# ============================================================

print()
print("=" * 78)

print(
    f"Opening {port_name} "
    f"@ {baudrate}..."
)

print("=" * 78)

try:

    ser = serial.Serial(
        port=port_name,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
        write_timeout=1.0
    )

except Exception as e:

    print()
    print(
        "Failed to open COM port:"
    )

    print(e)

    input(
        "Press Enter to exit..."
    )

    sys.exit()


print()
print("Port opened successfully.")
print(f"Port     : {port_name}")
print(f"Baudrate : {baudrate}")

print()


# ============================================================
# BUFFER
# ============================================================

rx_buffer = bytearray()


# ============================================================
# IMPORTANT:
# We do NOT immediately clear and start.
#
# First observe the existing CRSF traffic.
# ============================================================

listen_for_response(
    ser,
    rx_buffer,
    timeout=3.0
)


# ============================================================
# PING TESTS
# ============================================================


# ------------------------------------------------------------
# Broadcast
# ------------------------------------------------------------

broadcast_ping = make_device_ping(
    CRSF_ADDRESS_BROADCAST
)

send_test(
    ser,
    rx_buffer,
    broadcast_ping,
    listen_time=3.0
)


# ------------------------------------------------------------
# Receiver
# ------------------------------------------------------------

receiver_ping = make_device_ping(
    CRSF_ADDRESS_RECEIVER
)

send_test(
    ser,
    rx_buffer,
    receiver_ping,
    listen_time=3.0
)


# ============================================================
# PRINT KNOWN FRAMES
# ============================================================

ser.close()

print("Done.")
input("Press Enter to exit...")