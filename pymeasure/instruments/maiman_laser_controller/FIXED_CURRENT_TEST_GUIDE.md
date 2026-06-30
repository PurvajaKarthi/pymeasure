# Fixed Current Test Guide

## Overview

This guide walks you through testing the **fixed current approach** for relay autotune PID coefficient calculation. This is the foundational step before implementing the full relay autotune algorithm.

## What You're Testing

Instead of using the device's built-in PID loop to control temperature, you will:

1. **Fix the current** to a constant value (e.g., 500 mA)
2. **Monitor temperature** as it rises
3. **Flip the current** when temperature crosses a threshold (zero crossing)
4. **Watch for oscillations** around an equilibrium temperature
5. **Measure oscillation characteristics** to calculate P, I, D coefficients

## Scripts Provided

### 1. `fixed_current_test.py` - Main Test Script
Runs the test and collects temperature/current data

**Features:**
- Initializes device connection
- Sets fixed current value
- Monitors temperature with configurable sampling
- Detects and flips current at zero crossings
- Saves results to CSV for analysis
- Provides real-time console output

### 2. `analyze_fixed_current_test.py` - Analysis Script
Analyzes test results and generates visualizations

**Features:**
- Loads CSV data from test
- Calculates oscillation period and amplitude
- Generates professional plots
- Exports analysis report
- Calculates Ziegler-Nichols PID coefficients

## Quick Start

### Step 1: Install Dependencies

```bash
# Required for main test
pip install pyserial PyYAML

# Optional but recommended for analysis
pip install matplotlib numpy
```

### Step 2: Run the Fixed Current Test

```bash
# Basic usage (uses defaults)
python fixed_current_test.py

# With custom parameters
python fixed_current_test.py \
    --port COM3 \
    --current 500 \
    --duration 120 \
    --target-temp 25.0 \
    --sampling 0.5
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--port` | COM3 | Serial port of laser driver |
| `--current` | 500 | Fixed current in mA (magnitude) |
| `--duration` | 120 | Test duration in seconds |
| `--target-temp` | 25.0 | Temperature threshold for zero crossing (°C) |
| `--sampling` | 0.5 | Sampling interval in seconds |
| `--no-flip` | - | Disable automatic current flip |
| `--output-dir` | ./autotune_results | Directory for results |

### Step 3: Wait for Test to Complete

The script will display real-time output:

```
============================================================
DEVICE INITIALIZATION
============================================================
✓ Logger initialized: ./autotune_results/device_test_20240115_143022.log
✓ Serial communication created for port COM3
✓ Device configuration and model created
✓ Device instance created
✓ Serial connection established
✓ Device initialized and configured

============================================================
COLLECTING DATA
============================================================

    Time (s) |  Temp (°C) | Current (mA) | Status
------------------------------------------------------------
      0.50 |      22.31 |       500.0 | 
      1.00 |      22.45 |       500.0 | 
      1.50 |      23.12 |       500.0 | 
      2.00 |      24.89 |       500.0 | 
      2.50 |      25.14 |      -500.0 | ⟳ FLIPPED
      3.00 |      25.00 |      -500.0 | 
      3.50 |      24.45 |      -500.0 | 
      4.00 |      24.12 |      -500.0 | 
      4.50 |      24.00 |       500.0 | ⟳ FLIPPED
```

### Step 4: Analyze Results

```bash
# Analyze the most recent test (auto-finds it)
python analyze_fixed_current_test.py

# Or specify a specific CSV file
python analyze_fixed_current_test.py ./autotune_results/fixed_current_test_20240115_143022.csv

# Generate plot and save to file
python analyze_fixed_current_test.py --output plot.png
```

## Expected Results

### Good Results (System is Oscillating)

```
OSCILLATION METRICS
------------------------------------------------------------
  Average Period: 3.45 seconds
  Frequency: 0.290 Hz
  Amplitude: ±2.15°C
  Peak-to-Peak: 4.30°C

  Individual Periods:
    Period 1: 3.42s
    Period 2: 3.48s
    Period 3: 3.44s
    ... and 2 more
```

**What this means:**
- ✅ Temperature oscillates around the zero crossing point
- ✅ Current flips are being detected correctly
- ✅ System has predictable oscillation pattern
- ✅ **Ready for PID coefficient calculation!**

### Problematic Results

#### Issue: No Zero Crossings Detected

```
Zero Crossings Detected: 0
Amplitude: None
```

**Possible causes:**
- Fixed current is too low (increase `--current`)
- Target temperature is above/below actual oscillation range
- Test duration too short (increase `--duration`)

**Solution:**
```bash
python fixed_current_test.py --current 750 --duration 180
```

#### Issue: Temperature Rises But Doesn't Flip

**Possible causes:**
- Device's internal PID is still active (conflicting control)
- Current flip command not working
- Device needs different state configuration

**Solution:**
Check if you need to disable the device's internal temperature control in the YAML config, or set specific state bits.

#### Issue: Erratic Temperature Readings

**Possible causes:**
- Poor serial connection
- High electrical noise
- Sampling interval too short

**Solution:**
```bash
python fixed_current_test.py --sampling 1.0  # Increase interval
```

## Understanding the Output Files

### CSV File Example

```
Elapsed Time (s),Temperature (°C),Current Setpoint (mA),Is Positive Current,Zero Crossing Detected
0.500,22.3100,500.0,Yes,No
1.000,22.4500,500.0,Yes,No
2.000,23.1200,500.0,Yes,No
2.500,25.1400,-500.0,No,Yes
3.000,25.0000,-500.0,No,No
4.500,24.0000,500.0,Yes,Yes
```

**Columns:**
- **Elapsed Time**: Seconds from test start
- **Temperature**: Current temperature reading (°C)
- **Current Setpoint**: Current value being applied (mA, positive or negative)
- **Is Positive Current**: Whether using positive (heating) or negative (cooling) current
- **Zero Crossing Detected**: Whether a flip occurred at this sample

### Plot Visualization

The generated plot shows three subplots:

1. **Top**: Temperature over time with zero crossing markers
2. **Middle**: Current setpoint showing flip events
3. **Bottom**: Temperature and current together for correlation analysis

**What to look for:**
- Temperature should cross the target line at each flip point
- Current should alternate between +500 and -500
- Pattern should be regular and repeating

## Next Steps After Testing

### If Results Look Good:

1. **Note the oscillation characteristics:**
   - Period (T_u): How long for one complete cycle
   - Amplitude (A): Temperature swing around zero crossing

2. **Calculate PID coefficients using Ziegler-Nichols:**

   ```
   Ku = 4 * A / (π * error_band)
   
   Kp = 0.6 * Ku
   Ki = 1.2 * Ku / T_u
   Kd = 0.075 * Ku * T_u
   ```

   Example:
   ```
   Period (T_u) = 3.45 seconds
   Amplitude (A) = 2.15°C
   Error band = ±0.5°C (acceptable tolerance)
   
   Ku = 4 * 2.15 / (π * 0.5) = 5.47
   Kp = 0.6 * 5.47 = 3.28
   Ki = 1.2 * 5.47 / 3.45 = 1.90
   Kd = 0.075 * 5.47 * 3.45 = 1.41
   ```

3. **Implement these values in your PID controller**

4. **Test the PID loop** with these coefficients

5. **Fine-tune if needed** (see tuning guide)

### If Results Need Adjustment:

Try different fixed current values:

```bash
# For slower/gentler response
python fixed_current_test.py --current 250 --duration 180

# For faster/more aggressive response  
python fixed_current_test.py --current 1000 --duration 180

# For different equilibrium temperature
python fixed_current_test.py --target-temp 30.0 --duration 180
```

## Troubleshooting

### Device Won't Connect

```
✗ Device initialization failed: Connection failed
```

**Solutions:**
1. Check the correct COM port: `python -m serial.tools.list_ports`
2. Ensure driver is powered on
3. Check USB cable connection

### No Temperature Readings

```
Error reading data: 'NoneType' object is not subscriptable
```

**Possible causes:**
- Device not initialized properly
- YAML config missing temperature sensor definition
- Wrong device ID

### Current Set Commands Fail

```
Warning: Failed to set current: ...
```

**Possible causes:**
- Device doesn't accept manual current commands in current mode
- YAML config doesn't define current parameter
- Device state needs to be set differently

## File Organization

```
./autotune_results/
├── device_test_20240115_143022.log          # Device log file
├── fixed_current_test_20240115_143022.csv   # Data from test
├── analysis_20240115_143022.txt             # Analysis report
└── plot_20240115_143022.png                 # Visualization
```

## Tips for Best Results

1. **Warm up the device first** - Run for 30 seconds before starting the real test
2. **Keep environment stable** - Avoid drafts, don't change room temperature during test
3. **Use reasonable current values** - Too low = no oscillation, too high = unstable
4. **Longer tests are better** - Aim for 5-10 complete oscillation cycles
5. **Document everything** - Note what you changed for next iteration

## Common Settings by Device Type

### SF6xxx Series (Lower Power)
```bash
python fixed_current_test.py \
    --current 300 \
    --duration 180 \
    --target-temp 25.0 \
    --sampling 0.2
```

### SF8xxx Series (Higher Power)
```bash
python fixed_current_test.py \
    --current 800 \
    --duration 120 \
    --target-temp 30.0 \
    --sampling 0.5
```

## Getting Help

If you encounter issues:

1. Check the device log file in `./autotune_results/device_test_*.log`
2. Review the analysis report: `analysis_*.txt`
3. Visualize the plot to see what's happening
4. Try with different parameters
5. Consult the Maiman Laser Diode Driver Library documentation

## Next Phase: Full Relay Autotune

Once this test is working reliably, you'll be ready to:

1. Implement the full relay autotune loop in Python
2. Automate the PID coefficient calculation
3. Test the resulting PID controller
4. Fine-tune for your specific system

Good luck with your testing! 🚀
