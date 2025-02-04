import serial
import struct

class LidarReader:
    HEADER = 0x54
    POINTS_PER_PACKET = 12

    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def connect(self):
        """Establish a serial connection to the LiDAR device."""
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )

    def disconnect(self):
        """Close the serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()

    def read_packet(self):
        """Read a single data packet from the LiDAR device."""
        while True:
            # Read the header byte
            header = self.serial.read(1)
            if not header:
                continue

            if header[0] == self.HEADER:
                # Read the rest of the packet
                packet = self.serial.read(46)  # Total packet size is 48 bytes
                if len(packet) == 46:
                    print(f"Raw packet: {packet.hex()} Length: {len(packet)}")  # Debugging log
                    return header + packet
                else:
                    print(f"Incomplete packet received: {packet.hex()} Length: {len(packet)}")  # Debugging log

    @staticmethod
    def parse_packet(packet):
        """Parse the data packet and extract relevant information."""
        if packet[0] != LidarReader.HEADER:
            print("Invalid header detected.")
            return None

        ver_len = packet[1]
        speed = struct.unpack('<H', packet[2:4])[0] / 100.0  # Speed in degrees/sec
        start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0  # Start angle in degrees
        end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0  # End angle in degrees
        timestamp = struct.unpack('<H', packet[44:46])[0] / 1000.0  # Timestamp in seconds

        # Skip CRC verification for logging purposes
        print("Skipping CRC verification.")

        # Extract points
        points = []
        for i in range(LidarReader.POINTS_PER_PACKET):
            offset = 6 + i * 3
            distance = struct.unpack('<H', packet[offset:offset + 2])[0] / 1000.0  # Distance in meters
            #print(f"Distance: {distance} Offset: {offset}")
            intensity = packet[offset + 2]  # Signal intensity
            points.append((distance, intensity))


        # Interpolate angles, handling wrap-around
        print(f"Start angle: {start_angle} End angle: {end_angle}")
        if end_angle < start_angle:
            end_angle += 360.0  # Handle wrap-around

        angle_step = (end_angle - start_angle) / (LidarReader.POINTS_PER_PACKET - 1)
        angles = [(start_angle + i * angle_step) % 360.0 for i in range(LidarReader.POINTS_PER_PACKET)]

        # Combine points with angles
        data = [{'angle': angle, 'distance': distance, 'intensity': intensity}
                for angle, (distance, intensity) in zip(angles, points)]

        return {
            'speed': speed,
            'start_angle': start_angle,
            'end_angle': end_angle % 360.0,  # Normalize end_angle to 0-360
            'timestamp': timestamp,
            'points': data
        }

    def stream_data(self):
        """Continuously read and print LiDAR data."""
        try:
            self.connect()
            print("Streaming LiDAR data...")
            while True:
                packet = self.read_packet()
                if packet:
                    data = self.parse_packet(packet)
                    if data:
                        print(data)
        except KeyboardInterrupt:
            print("Stopping data stream...")
        finally:
            self.disconnect()

if __name__ == "__main__":
    lidar = LidarReader(port="/dev/ttyUSB0", baudrate=230400)
    lidar.stream_data()
