import time
import math
import matplotlib.pyplot as plt
from read_lidar import LidarReader

class LidarDataCollector:
    def __init__(self, duration=10, port='/dev/ttyUSB0', baudrate=230400, max_radius=10.0):
        self.duration = duration
        self.port = port
        self.baudrate = baudrate
        self.max_radius = max_radius
        self.data = []

    def collect_data(self):
        """Collect LiDAR data for the specified duration."""
        lidar = LidarReader(port=self.port, baudrate=self.baudrate)
        lidar.connect()
        start_time = time.time()

        try:
            while time.time() - start_time < self.duration:
                packet = lidar.read_packet()
                if packet:
                    parsed_data = lidar.parse_packet(packet)
                    if parsed_data:
                        for point in parsed_data['points']:
                            if point['distance'] <= self.max_radius:
                                self.data.append((point['angle'], point['distance']))
        finally:
            lidar.disconnect()

    def plot_data(self):
        """Plot the collected LiDAR data as a 2D graph."""
        angles = [math.radians(angle) for angle, distance in self.data]
        distances = [distance for angle, distance in self.data]

        x = [distance * math.cos(angle) for angle, distance in zip(angles, distances)]
        y = [-distance * math.sin(angle) for angle, distance in zip(angles, distances)] #change sign of y axis to account for left handed coordinate system
        plt.figure(figsize=(8, 8))
        plt.scatter(x, y, s=5, c='blue', alpha=0.5)
        plt.title("LiDAR Data Visualization")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.axis('equal')
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    collector = LidarDataCollector(duration=10, max_radius=10.0)
    print("Collecting data for 10 seconds...")
    collector.collect_data()
    print("Data collection complete. Plotting...")
    collector.plot_data()
