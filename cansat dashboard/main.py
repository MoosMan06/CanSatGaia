import serial
import struct
import csv

# Initialize the serial port (update with your serial port parameters)
ser = serial.Serial(port='com10', baudrate=9600, timeout=0.1)

# Constants for parsing
HEADER_SIZE = 8
MAGIC_NUMBER = b'\x67\x61\x69\x61'  # Magic number "gaia"

# FIFO buffer
buffer = bytearray()

# Initialize the CSV file
with open("output.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["packet_num", "data_type", "field", "value"])  # Write headers

def write_packet_to_csv(packet):
    with open("output.csv", "a", newline="") as f:
        writer = csv.writer(f)
        packet_num = packet["packet_num"]
        data_type = packet["data_type"]
        data = packet["data"]
        match data_type:
            case 0x01:
                writer.writerow([packet_num, data_type, "gps_pos_lat", struct.unpack("<f", data[:4])[0]]) 
                writer.writerow([packet_num, data_type, "gps_pos_lng", struct.unpack("<f", data[4:8])[0]])
                writer.writerow([packet_num, data_type, "gps_pos_alt", struct.unpack("<f", data[8:12])[0]])
            case 0x02:
                writer.writerow([packet_num, data_type, "g_forces_x", struct.unpack("<f", data[:4])[0]])
                writer.writerow([packet_num, data_type, "g_forces_y", struct.unpack("<f", data[4:8])[0]])
                writer.writerow([packet_num, data_type, "g_forces_z", struct.unpack("<f", data[8:12])[0]])
            case 0x03:
                writer.writerow([packet_num, data_type, "rotation_x", struct.unpack("<f", data[:4])[0]])
                writer.writerow([packet_num, data_type, "rotation_y", struct.unpack("<f", data[4:8])[0]])
                writer.writerow([packet_num, data_type, "rotation_z", struct.unpack("<f", data[8:12])[0]])
            case 0x04:
                writer.writerow([packet_num, data_type, "time", struct.unpack("<I", data[:4])[0]])
            case 0x05:
                writer.writerow([packet_num, data_type, "gps_fix_age", struct.unpack("<I", data[:4])[0]])
            case 0x06:
                writer.writerow([packet_num, data_type, "gps_hdop", struct.unpack("<f", data[:4])[0]])
            case 0x07:
                writer.writerow([packet_num, data_type, "gps_num_of_sats", struct.unpack("<B", data[:1])[0]])
            case 0x08:
                writer.writerow([packet_num, data_type, "gps_fail_percentage", struct.unpack("<f", data[:4])[0]])
            case 0x09:
                writer.writerow([packet_num, data_type, "co2_concentration", struct.unpack("<H", data[:2])[0]])
            case 0x0A:
                writer.writerow([packet_num, data_type, "temperature", struct.unpack("<f", data[:4])[0]])
            case 0x0B:
                writer.writerow([packet_num, data_type, "pressure", struct.unpack("<f", data[:4])[0]])
            case 0x0C:
                writer.writerow([packet_num, data_type, "dust_concentration", struct.unpack("<H", data[:2])[0]])
            case 0x0D:
                writer.writerow([packet_num, data_type, "uv_radiation", struct.unpack("<f", data[:4])[0]])
            case _:
                writer.writerow([packet_num, data_type, "unknown", data.hex()])

def parse_packet(buffer):
    packets = []
    i = 0

    while i + HEADER_SIZE <= len(buffer):
        # Check for the magic number
        if buffer[i:i+4] != MAGIC_NUMBER:
            i += 1  # Skip bytes until we find the magic number
            continue

        # Read the data type and length
        packet_num = buffer[i+4:i+6]
        data_type = buffer[i+6]
        data_length = buffer[i+7]

        # Calculate total packet size
        packet_size = HEADER_SIZE + data_length

        # Check if the full packet is in the buffer
        if i + packet_size > len(buffer):
            break  # Wait for more data

        # Extract the packet
        packet = buffer[i:i+packet_size]
        data = packet[HEADER_SIZE:packet_size]
        processed_data = data.replace(b'gaia\x00', b'gaia')
        packets.append({
            "data_type": data_type,
            "packet_num": struct.unpack("<H", packet_num)[0],
            "data": processed_data,
        })

        # Move to the next packet
        i += packet_size

    # Keep any remaining bytes in the buffer
    return packets, buffer[i:]

while True:
    # Read bytes from the serial port
    incoming_data = ser.read(1024)  # Adjust the chunk size as needed
    if incoming_data:
        buffer.extend(incoming_data)  # Append to the FIFO buffer

        # Parse packets from the buffer
        packets, buffer = parse_packet(buffer)

        # Handle the decoded packets
        for packet in packets:
            write_packet_to_csv(packet)