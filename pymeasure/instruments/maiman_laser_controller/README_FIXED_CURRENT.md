# Fixed Current Test for Relay Autotune PID

Quick reference for testing fixed current approach to laser diode driver control.

## 📋 Files Created

| File | Purpose |
|------|---------|
| `fixed_current_test.py` | Main test script - run this first |
| `analyze_fixed_current_test.py` | Analysis & visualization script |
| `FIXED_CURRENT_TEST_GUIDE.md` | Comprehensive documentation |
| `README_FIXED_CURRENT.md` | This file |

## ⚡ Quick Start (5 Minutes)

### 1. Install Dependencies
```bash
pip install pyserial PyYAML matplotlib numpy
```

### 2. Run the Test (replace COM3 with your port)
```bash
python fixed_current_test.py --port COM3 --current 500 --duration 120
```

### 3. Analyze Results
```bash
python analyze_fixed_current_test.py
```

## ✅ What Should Happen

**During the test:**
- Temperature should start at room temp
- Rise due to fixed +500 mA current
- Cross your target temperature (e.g., 25°C)
- Get flipped to -500 mA
- Drop back down
- Cross threshold again and flip back to +500 mA
- Repeat several times

**Console output should look like:**
```
      0.50 |      22.31 |       500.0 | 
      2.50 |      25.14 |      -500.0 | ⟳ FLIPPED
      4.50 |      24.00 |       500.0 | ⟳ FLIPPED
      6.50 |      25.20 |      -500.0 | ⟳ FLIPPED
```

**Analysis should show:**
```
Average Period: 3.45 seconds
Frequency: 0.290 Hz
Amplitude: ±2.15°C
```

## 🎯 Key Parameters to Adjust

| Parameter | Effect | Typical Range |
|-----------|--------|---------------|
| `--current` | Magnitude of temp change | 200-1000 mA |
| `--duration` | How long to run test | 60-300 seconds |
| `--target-temp` | Zero crossing point | 20-35°C |
| `--sampling` | How often to read temp | 0.1-1.0 seconds |

## 🔧 Troubleshooting Quick Fixes

### No oscillations detected?
```bash
# Try higher current
python fixed_current_test.py --current 750 --duration 180
```

### Connection failed?
```bash
# List available ports
python -m serial.tools.list_ports

# Use correct port
python fixed_current_test.py --port COM5
```

### No temperature readings?
- Check YAML config has temperature sensor defined
- Verify device is properly initialized
- Check device state (may need specific state bits set)

## 📊 After Testing

The analysis script will produce:

1. **Console Summary** - Key metrics and oscillation info
2. **Analysis Text File** - Detailed report in `./autotune_results/analysis_*.txt`
3. **CSV File** - Raw data in `./autotune_results/fixed_current_test_*.csv`
4. **Plot PNG** - Visualization in `./autotune_results/plot_*.png`

## 🧮 Using Results for PID Tuning

Once you have good oscillation data, use the values to calculate PID coefficients:

```python
from analyze_fixed_current_test import FixedCurrentAnalyzer

analyzer = FixedCurrentAnalyzer("fixed_current_test_*.csv")
metrics = analyzer.calculate_oscillation_metrics()

period = metrics['avg_period']  # e.g., 3.45 seconds
amplitude = metrics['amplitude']  # e.g., 2.15°C

# Ziegler-Nichols formulas
Ku = 4 * amplitude / (3.14159 * 0.5)  # 0.5 is error band
Kp = 0.6 * Ku
Ki = 1.2 * Ku / period
Kd = 0.075 * Ku * period

print(f"Kp = {Kp:.4f}")
print(f"Ki = {Ki:.4f}")
print(f"Kd = {Kd:.4f}")
```

## 📚 Full Documentation

See **FIXED_CURRENT_TEST_GUIDE.md** for:
- Detailed explanation of what's happening
- Expected vs problematic results
- Advanced troubleshooting
- Next steps for full relay autotune implementation
- Device-specific settings

## 🔗 Related Files

- **Maiman Library Docs**: See `LaserDiodeDriverLibraryDocumentation1.0.4.pdf`
- **Device YAML Config**: Check your device configuration file for parameter definitions
- **Test Notebook**: See `sf8xxx_full_test_notebook.ipynb` for additional context

## ⚠️ Safety Notes

- Ensure laser diode driver is properly powered and connected
- Monitor temperature during test - stop if overheating
- Ensure adequate ventilation
- Don't exceed device current limits

## 💡 Pro Tips

1. **Save your results** - Keep CSV and plot files for comparison
2. **Test multiple currents** - Find the sweet spot for your system
3. **Document settings** - Note what worked for repeatability
4. **Warm up first** - Run device for 30s before actual test
5. **Check logs** - Review device log if something seems wrong

## 🚀 Next Steps

1. ✅ Run fixed current test (you are here)
2. ⏳ Implement full relay autotune algorithm
3. ⏳ Calculate final PID coefficients
4. ⏳ Validate with actual temperature setpoint control
5. ⏳ Fine-tune P, I, D values for your specific hardware

---

**Need help?** Check `FIXED_CURRENT_TEST_GUIDE.md` for comprehensive documentation.
