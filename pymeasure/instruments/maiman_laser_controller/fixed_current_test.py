"""
Fixed Current Test Script for Relay Autotune PID Tuning
========================================================

This script tests the feasibility of fixing the current and monitoring temperature
oscillations for relay autotune PID coefficient calculation.

Usage:
    python fixed_current_test.py --port COM3 --current 500 --duration 60 --target-temp 25

The script will:
    1. Connect to the laser diode driver
    2. Set a fixed current value
    3. Monitor temperature over time
    4. Optionally flip current at zero crossings
    5. Save results to CSV for analysis
"""

import time
import csv
import argparse
from datetime import datetime
from pathlib import Path
from dataclasses import dataclass, field

# Import Maiman library components
import sys
sys.path.insert(0, str(Path(__file__).parent))

try:
    from maiman_laser_controller.src.Device import Device
    from maiman_laser_controller.src.SerialCommunication import SerialCommunication
    from maiman_laser_controller.src.DeviceConfig import DeviceConfig
    from maiman_laser_controller.src.DeviceModel import DeviceModel
    from maiman_laser_controller.src.Logger import Logger, LogEvent
except ImportError as e:
    print(f"Error importing Maiman library: {e}")
    print("Make sure the maiman_laser_controller module is in the Python path")
    sys.exit(1)


@dataclass
class TestConfig:
    """Configuration for fixed current test"""
    port: str = "COM3"
    fixed_current: float = 500.0  # mA
    target_temperature: float = 25.0  #°C - zero crossing point
    test_duration: int = 120  # seconds
    sampling_interval: float = 0.5  # seconds between readings
    enable_zero_crossing_flip: bool = True
    output_dir: str = "./autotune_results"


@dataclass
class TemperatureReading:
    """Single temperature reading with timestamp and current state"""
    timestamp: float
    elapsed_time: float
    temperature: float
    current_setpoint: float
    is_positive_current: bool
    crossed_zero: bool = False


class FixedCurrentTester:
    """Test fixed current behavior and temperature response"""

    def __init__(self, config: TestConfig):
        self.config = config
        self.device = None
        self.logger = None
        self.readings = []
        self.start_time = None
        self.is_positive_current = True
        self.last_temp = None

        # Create output directory
        Path(config.output_dir).mkdir(parents=True, exist_ok=True)

    def initialize_device(self) -> bool:
        """Initialize device connection and configuration"""
        try:
            print(f"\n{'='*60}")
            print("DEVICE INITIALIZATION")
            print(f"{'='*60}")

            # Create logger
            log_file = Path(self.config.output_dir) / f"device_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            self.logger = Logger(str(log_file), enable_logging=True)
            print(f"✓ Logger initialized: {log_file}")

            # Create serial communication
            serial_comm = SerialCommunication(self.config.port)
            print(f"✓ Serial communication created for port {self.config.port}")

            # Create device configuration and model
            device_config = DeviceConfig()
            device_model = DeviceModel()
            print(f"✓ Device configuration and model created")

            # Create device instance
            self.device = Device(device_model, serial_comm, device_config, self.logger)
            print(f"✓ Device instance created")

            # Connect and initialize
            serial_comm.connect()
            print(f"✓ Serial connection established")

            self.device.initialize_device()
            print(f"✓ Device initialized and configured")

            return True

        except Exception as e:
            print(f"✗ Device initialization failed: {e}")
            self.logger.errorRecording(f"Device initialization error: {str(e)}")
            return False

    def start_test(self) -> bool:
        """Start the device and begin test"""
        try:
            print(f"\n{'='*60}")
            print("STARTING TEST")
            print(f"{'='*60}")

            # Enable device
            self.device.start_device()
            print(f"✓ Device started")

            # Set initial current
            self._set_current(self.config.fixed_current)

            self.start_time = time.time()
            print(f"✓ Test started at {datetime.now().strftime('%H:%M:%S')}")
            print(f"\nTest Configuration:")
            print(f"  Fixed Current: {self.config.fixed_current} mA")
            print(f"  Target Temperature (zero crossing): {self.config.target_temperature}°C")
            print(f"  Duration: {self.config.test_duration} seconds")
            print(f"  Sampling Interval: {self.config.sampling_interval} seconds")
            print(f"  Zero Crossing Flip: {'Enabled' if self.config.enable_zero_crossing_flip else 'Disabled'}")

            return True

        except Exception as e:
            print(f"✗ Failed to start test: {e}")
            self.logger.errorRecording(f"Test start error: {str(e)}")
            return False

    def _set_current(self, current_value: float):
        """Set the device current to a fixed value"""
        try:
            self.device.set_current_code(current_value)
            print(f"  Current set to {current_value} mA")
        except Exception as e:
            print(f"  Warning: Failed to set current: {e}")
            self.logger.errorRecording(f"Failed to set current: {str(e)}")

    def _flip_current(self):
        """Flip current sign (positive to negative or vice versa)"""
        self.is_positive_current = not self.is_positive_current
        new_current = self.config.fixed_current if self.is_positive_current else -self.config.fixed_current

        try:
            self.device.set_current_code(new_current)
            return new_current
        except Exception as e:
            print(f"  Warning: Failed to flip current: {e}")
            self.logger.errorRecording(f"Failed to flip current: {str(e)}")
            return None

    def _check_zero_crossing(self, current_temp: float) -> bool:
        """Detect if temperature crossed the target threshold"""
        if self.last_temp is None:
            self.last_temp = current_temp
            return False

        # Check for crossing from below or above
        crossed = (self.last_temp < self.config.target_temperature <= current_temp or
                   self.last_temp > self.config.target_temperature >= current_temp)

        self.last_temp = current_temp
        return crossed

    def collect_data(self):
        """Main data collection loop"""
        print(f"\n{'='*60}")
        print("COLLECTING DATA")
        print(f"{'='*60}\n")
        print(f"{'Time (s)':>10} | {'Temp (°C)':>10} | {'Current (mA)':>12} | Status")
        print("-" * 60)

        while True:
            elapsed = time.time() - self.start_time

            if elapsed > self.config.test_duration:
                break

            try:
                # Read temperature
                temp = self.device.get_temperature_measured()

                # Determine current state
                current_value = self.config.fixed_current if self.is_positive_current else -self.config.fixed_current

                # Check for zero crossing
                crossed = False
                if self.config.enable_zero_crossing_flip:
                    crossed = self._check_zero_crossing(temp)
                    if crossed:
                        new_current = self._flip_current()
                        current_value = new_current if new_current is not None else current_value

                # Create reading
                reading = TemperatureReading(
                    timestamp=time.time(),
                    elapsed_time=elapsed,
                    temperature=temp,
                    current_setpoint=current_value,
                    is_positive_current=self.is_positive_current,
                    crossed_zero=crossed
                )
                self.readings.append(reading)

                # Display status
                status = "⟳ FLIPPED" if crossed else ""
                print(f"{elapsed:>10.2f} | {temp:>10.2f} | {current_value:>12.1f} | {status}")

                # Log crossing events
                if crossed:
                    self.logger.eventRecording(
                        LogEvent(
                            level="INFO",
                            message=f"Zero crossing detected at {elapsed:.2f}s, temp={temp:.2f}°C, current flipped to {current_value:.1f}mA"
                        )
                    )

                time.sleep(self.config.sampling_interval)

            except Exception as e:
                print(f"Error reading data: {e}")
                self.logger.errorRecording(f"Data collection error: {str(e)}")
                time.sleep(self.config.sampling_interval)

        print("-" * 60)

    def stop_test(self):
        """Stop the test and disconnect device"""
        try:
            print(f"\n{'='*60}")
            print("STOPPING TEST")
            print(f"{'='*60}")

            self.device.stop_device()
            print(f"✓ Device stopped")

            # Disconnect
            if self.device._Device__serial_comm:
                self.device._Device__serial_comm.disconnect()
                print(f"✓ Serial connection closed")

        except Exception as e:
            print(f"Warning: Error during shutdown: {e}")

    def save_results(self) -> str:
        """Save test results to CSV file"""
        try:
            csv_file = Path(self.config.output_dir) / f"fixed_current_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

            with open(csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'Elapsed Time (s)',
                    'Temperature (°C)',
                    'Current Setpoint (mA)',
                    'Is Positive Current',
                    'Zero Crossing Detected',
                    'Timestamp'
                ])

                for reading in self.readings:
                    writer.writerow([
                        f"{reading.elapsed_time:.3f}",
                        f"{reading.temperature:.4f}",
                        f"{reading.current_setpoint:.1f}",
                        "Yes" if reading.is_positive_current else "No",
                        "Yes" if reading.crossed_zero else "No",
                        datetime.fromtimestamp(reading.timestamp).strftime('%H:%M:%S.%f')[:-3]
                    ])

            print(f"✓ Results saved to: {csv_file}")
            return str(csv_file)

        except Exception as e:
            print(f"✗ Failed to save results: {e}")
            return None

    def print_summary(self):
        """Print summary statistics"""
        if not self.readings:
            print("No data collected")
            return

        temperatures = [r.temperature for r in self.readings]
        currents = [r.current_setpoint for r in self.readings]
        zero_crossings = sum(1 for r in self.readings if r.crossed_zero)

        print(f"\n{'='*60}")
        print("TEST SUMMARY")
        print(f"{'='*60}")
        print(f"Total Readings: {len(self.readings)}")
        print(f"Test Duration: {self.readings[-1].elapsed_time:.2f} seconds")
        print(f"\nTemperature Statistics:")
        print(f"  Min:     {min(temperatures):.2f}°C")
        print(f"  Max:     {max(temperatures):.2f}°C")
        print(f"  Average: {sum(temperatures)/len(temperatures):.2f}°C")
        print(f"  Range:   {max(temperatures) - min(temperatures):.2f}°C")
        print(f"\nCurrent Statistics:")
        print(f"  Fixed Value: {self.config.fixed_current:.1f} mA")
        print(f"  Flips Applied: {zero_crossings}")
        print(f"\nData saved to: {self.config.output_dir}")


def main():
    parser = argparse.ArgumentParser(
        description='Test fixed current approach for relay autotune PID tuning'
    )
    parser.add_argument('--port', type=str, default='COM3', help='Serial port (default: COM3)')
    parser.add_argument('--current', type=float, default=500, help='Fixed current in mA (default: 500)')
    parser.add_argument('--duration', type=int, default=120, help='Test duration in seconds (default: 120)')
    parser.add_argument('--target-temp', type=float, default=25.0, help='Target temperature for zero crossing (default: 25.0)')
    parser.add_argument('--sampling', type=float, default=0.5, help='Sampling interval in seconds (default: 0.5)')
    parser.add_argument('--no-flip', action='store_true', help='Disable zero crossing current flip')
    parser.add_argument('--output-dir', type=str, default='./autotune_results', help='Output directory (default: ./autotune_results)')

    args = parser.parse_args()

    # Create configuration
    config = TestConfig(
        port=args.port,
        fixed_current=args.current,
        target_temperature=args.target_temp,
        test_duration=args.duration,
        sampling_interval=args.sampling,
        enable_zero_crossing_flip=not args.no_flip,
        output_dir=args.output_dir
    )

    # Run test
    tester = FixedCurrentTester(config)

    try:
        if not tester.initialize_device():
            return 1

        if not tester.start_test():
            return 1

        tester.collect_data()
        tester.print_summary()

        csv_file = tester.save_results()

        if csv_file:
            print(f"\n✓ Test completed successfully!")
            print(f"Next steps:")
            print(f"  1. Open the CSV file in Excel or Python to visualize the temperature response")
            print(f"  2. Check if temperature oscillates around the target when current flips")
            print(f"  3. Measure oscillation period and amplitude for PID calculation")
            print(f"\nCSV file: {csv_file}")

        return 0

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n✗ Test failed with error: {e}")
        return 1
    finally:
        tester.stop_test()


if __name__ == "__main__":
    exit(main())
