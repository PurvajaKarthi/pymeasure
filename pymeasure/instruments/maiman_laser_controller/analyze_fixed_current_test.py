"""
Analysis Script for Fixed Current Test Results
==============================================

This script analyzes the CSV output from fixed_current_test.py and generates
plots to visualize temperature response and current flips.

Usage:
    python analyze_fixed_current_test.py <csv_file>

Or without arguments to use the most recent CSV file in ./autotune_results/
"""

import csv
import argparse
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass
from typing import List, Tuple

try:
    import matplotlib.pyplot as plt
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("Warning: matplotlib not installed. Install with: pip install matplotlib numpy")


@dataclass
class Reading:
    """Represents a single reading from test"""
    elapsed_time: float
    temperature: float
    current: float
    crossed_zero: bool


class FixedCurrentAnalyzer:
    """Analyze fixed current test results"""

    def __init__(self, csv_file: str):
        self.csv_file = Path(csv_file)
        self.readings: List[Reading] = []
        self.zero_crossings: List[Tuple[float, float]] = []  # (time, temp)

        if not self.csv_file.exists():
            raise FileNotFoundError(f"CSV file not found: {csv_file}")

        self.load_data()

    def load_data(self):
        """Load data from CSV file"""
        print(f"Loading data from: {self.csv_file}")

        with open(self.csv_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                reading = Reading(
                    elapsed_time=float(row['Elapsed Time (s)']),
                    temperature=float(row['Temperature (°C)']),
                    current=float(row['Current Setpoint (mA)']),
                    crossed_zero=row['Zero Crossing Detected'] == 'Yes'
                )
                self.readings.append(reading)

                if reading.crossed_zero:
                    self.zero_crossings.append((reading.elapsed_time, reading.temperature))

        print(f"✓ Loaded {len(self.readings)} readings")
        print(f"✓ Found {len(self.zero_crossings)} zero crossings")

    def get_statistics(self) -> dict:
        """Calculate statistics from readings"""
        if not self.readings:
            return {}

        temps = [r.temperature for r in self.readings]
        currents = [r.current for r in self.readings]

        stats = {
            'total_readings': len(self.readings),
            'duration': self.readings[-1].elapsed_time,
            'temp_min': min(temps),
            'temp_max': max(temps),
            'temp_avg': sum(temps) / len(temps),
            'temp_range': max(temps) - min(temps),
            'current_positive': sum(1 for c in currents if c > 0),
            'current_negative': sum(1 for c in currents if c < 0),
            'zero_crossings': len(self.zero_crossings),
        }

        return stats

    def calculate_oscillation_metrics(self) -> dict:
        """Calculate period and amplitude of oscillations"""
        if len(self.zero_crossings) < 2:
            return {'period': None, 'amplitude': None}

        # Calculate period (time between zero crossings)
        periods = []
        for i in range(len(self.zero_crossings) - 1):
            period = self.zero_crossings[i + 1][0] - self.zero_crossings[i][0]
            periods.append(period)

        avg_period = sum(periods) / len(periods) if periods else None

        # Calculate amplitude (half the peak-to-peak)
        temps = [r.temperature for r in self.readings]
        if temps:
            amplitude = (max(temps) - min(temps)) / 2
        else:
            amplitude = None

        return {
            'periods': periods,
            'avg_period': avg_period,
            'amplitude': amplitude,
            'peak_to_peak': max(temps) - min(temps) if temps else None,
        }

    def print_summary(self):
        """Print analysis summary"""
        stats = self.get_statistics()
        oscillation = self.calculate_oscillation_metrics()

        print(f"\n{'='*60}")
        print("TEST ANALYSIS SUMMARY")
        print(f"{'='*60}")

        if stats:
            print(f"\nGeneral Statistics:")
            print(f"  Total Readings: {stats['total_readings']}")
            print(f"  Test Duration: {stats['duration']:.2f} seconds")
            print(f"\nTemperature Statistics:")
            print(f"  Min Temperature: {stats['temp_min']:.2f}°C")
            print(f"  Max Temperature: {stats['temp_max']:.2f}°C")
            print(f"  Average Temperature: {stats['temp_avg']:.2f}°C")
            print(f"  Temperature Range: {stats['temp_range']:.2f}°C")

            print(f"\nCurrent Statistics:")
            print(f"  Positive Current Samples: {stats['current_positive']}")
            print(f"  Negative Current Samples: {stats['current_negative']}")
            print(f"  Total Zero Crossings: {stats['zero_crossings']}")

        if oscillation['avg_period']:
            print(f"\nOscillation Metrics:")
            print(f"  Average Period: {oscillation['avg_period']:.2f} seconds")
            print(f"  Frequency: {1/oscillation['avg_period']:.3f} Hz")
            print(f"  Amplitude: {oscillation['amplitude']:.2f}°C")
            print(f"  Peak-to-Peak: {oscillation['peak_to_peak']:.2f}°C")

            if oscillation['periods']:
                print(f"\n  Individual Periods:")
                for i, period in enumerate(oscillation['periods'][:5], 1):
                    print(f"    Period {i}: {period:.2f}s")
                if len(oscillation['periods']) > 5:
                    print(f"    ... and {len(oscillation['periods']) - 5} more")

        print(f"\n{'='*60}")

    def plot_results(self, output_file: str = None):
        """Generate plots of test results"""
        if not MATPLOTLIB_AVAILABLE:
            print("Cannot plot: matplotlib not installed")
            return

        if not self.readings:
            print("No data to plot")
            return

        # Prepare data
        times = [r.elapsed_time for r in self.readings]
        temps = [r.temperature for r in self.readings]
        currents = [r.current for r in self.readings]

        # Create figure with subplots
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        fig.suptitle('Fixed Current Test Results', fontsize=16, fontweight='bold')

        # Plot 1: Temperature vs Time
        ax = axes[0]
        ax.plot(times, temps, 'b-', linewidth=2, label='Temperature')
        ax.scatter([zc[0] for zc in self.zero_crossings], [zc[1] for zc in self.zero_crossings],
                   color='red', s=100, marker='x', label='Zero Crossings', zorder=5)
        ax.set_ylabel('Temperature (°C)', fontsize=11, fontweight='bold')
        ax.set_title('Temperature Response Over Time', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')

        # Add zero crossing annotations
        for i, (t, temp) in enumerate(self.zero_crossings[:5]):  # Label first 5
            ax.annotate(f'ZC{i+1}', xy=(t, temp), xytext=(5, 10),
                       textcoords='offset points', fontsize=9,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.3))

        # Plot 2: Current vs Time
        ax = axes[1]
        ax.step(times, currents, 'g-', linewidth=2, where='post', label='Current Setpoint')
        ax.scatter([zc[0] for zc in self.zero_crossings], [currents[min(int(zc[0]/0.5), len(currents)-1)] for zc in self.zero_crossings],
                   color='red', s=100, marker='x', label='Current Flips', zorder=5)
        ax.set_ylabel('Current (mA)', fontsize=11, fontweight='bold')
        ax.set_title('Current Setpoint (Manual Flips)', fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)

        # Plot 3: Temperature with Current Overlay
        ax1 = axes[2]
        color = 'tab:blue'
        ax1.set_xlabel('Time (seconds)', fontsize=11, fontweight='bold')
        ax1.set_ylabel('Temperature (°C)', color=color, fontsize=11, fontweight='bold')
        line1 = ax1.plot(times, temps, color=color, linewidth=2, label='Temperature')
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.grid(True, alpha=0.3)

        ax2 = ax1.twinx()
        color = 'tab:green'
        ax2.set_ylabel('Current (mA)', color=color, fontsize=11, fontweight='bold')
        line2 = ax2.step(times, currents, color=color, linewidth=2, where='post', label='Current', alpha=0.7)
        ax2.tick_params(axis='y', labelcolor=color)

        ax1.set_title('Temperature and Current Together', fontsize=12, fontweight='bold')

        # Combined legend
        lines = line1 + [line2]
        labels = ['Temperature', 'Current']
        ax1.legend(lines, labels, loc='upper left', fontsize=10)

        plt.tight_layout()

        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"✓ Plot saved to: {output_file}")
        else:
            plt.show()

    def export_oscillation_data(self, output_file: str = None):
        """Export oscillation metrics to a text file"""
        oscillation = self.calculate_oscillation_metrics()
        stats = self.get_statistics()

        if not output_file:
            output_file = self.csv_file.parent / f"analysis_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

        with open(output_file, 'w') as f:
            f.write("FIXED CURRENT TEST ANALYSIS REPORT\n")
            f.write("=" * 60 + "\n\n")

            f.write(f"CSV File: {self.csv_file}\n")
            f.write(f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

            f.write("GENERAL STATISTICS\n")
            f.write("-" * 60 + "\n")
            f.write(f"Total Readings: {stats['total_readings']}\n")
            f.write(f"Test Duration: {stats['duration']:.2f} seconds\n")
            f.write(f"Temperature Min: {stats['temp_min']:.2f}°C\n")
            f.write(f"Temperature Max: {stats['temp_max']:.2f}°C\n")
            f.write(f"Temperature Average: {stats['temp_avg']:.2f}°C\n")
            f.write(f"Temperature Range: {stats['temp_range']:.2f}°C\n")
            f.write(f"Positive Current Samples: {stats['current_positive']}\n")
            f.write(f"Negative Current Samples: {stats['current_negative']}\n")
            f.write(f"Zero Crossings Detected: {stats['zero_crossings']}\n\n")

            f.write("OSCILLATION METRICS\n")
            f.write("-" * 60 + "\n")
            if oscillation['avg_period']:
                f.write(f"Average Period: {oscillation['avg_period']:.4f} seconds\n")
                f.write(f"Frequency: {1/oscillation['avg_period']:.4f} Hz\n")
                f.write(f"Amplitude (±): {oscillation['amplitude']:.4f}°C\n")
                f.write(f"Peak-to-Peak: {oscillation['peak_to_peak']:.4f}°C\n\n")

                f.write("Individual Periods:\n")
                for i, period in enumerate(oscillation['periods'], 1):
                    f.write(f"  Period {i}: {period:.4f} seconds\n")
            else:
                f.write("Insufficient zero crossings to calculate oscillation metrics\n\n")

            f.write("\nINTERPRETATION\n")
            f.write("-" * 60 + "\n")
            if oscillation['avg_period']:
                f.write(f"The system oscillates at approximately {1/oscillation['avg_period']:.2f} Hz\n")
                f.write(f"with an amplitude of ±{oscillation['amplitude']:.2f}°C.\n\n")
                f.write("Next steps for PID tuning:\n")
                f.write("1. Note the oscillation period (T_u): {:.4f} seconds\n".format(oscillation['avg_period']))
                f.write("2. Note the amplitude (A): {:.4f}°C\n".format(oscillation['amplitude']))
                f.write("3. Use Ziegler-Nichols formulas:\n")
                f.write("   - Ku (Ultimate Gain) = 4 * A / (π * error_band)\n")
                f.write("   - Kp = 0.6 * Ku\n")
                f.write("   - Ki = 1.2 * Ku / T_u\n")
                f.write("   - Kd = 0.075 * Ku * T_u\n")
            else:
                f.write("Not enough oscillations detected. Consider:\n")
                f.write("- Increasing test duration\n")
                f.write("- Increasing fixed current magnitude\n")
                f.write("- Lowering target temperature zero crossing point\n")

        print(f"✓ Analysis exported to: {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze fixed current test results'
    )
    parser.add_argument('csv_file', nargs='?', help='CSV file to analyze')
    parser.add_argument('--no-plot', action='store_true', help='Skip plotting')
    parser.add_argument('--output', type=str, help='Output file for plot (PNG)')

    args = parser.parse_args()

    # Find CSV file
    csv_file = args.csv_file
    if not csv_file:
        # Look for most recent CSV in autotune_results
        result_dir = Path('./autotune_results')
        if result_dir.exists():
            csv_files = sorted(result_dir.glob('fixed_current_test_*.csv'))
            if csv_files:
                csv_file = str(csv_files[-1])
                print(f"Using most recent file: {csv_file}\n")

    if not csv_file:
        print("Error: No CSV file specified and none found in ./autotune_results/")
        return 1

    try:
        analyzer = FixedCurrentAnalyzer(csv_file)
        analyzer.print_summary()
        analyzer.export_oscillation_data()

        if not args.no_plot:
            output_file = args.output
            if not output_file:
                output_file = Path(csv_file).parent / f"plot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            analyzer.plot_results(str(output_file))

        return 0

    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    exit(main())
