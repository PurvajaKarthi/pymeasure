from pymeasure.instruments.maiman_laser_controller.controller import SF8xxx
import time
COM_PORT = "COM3"

dev = SF8xxx(COM_PORT, start_thread=False)


if not dev.connected:
    print("❌ Not connected")

else:
    print("✅ Connected")
    print("Serial number:", dev.serial_no)
    
   
    print("Setting driver current setpoint to 5.0 mA")
    dev.set_driver_current(int(5.0))

    time.sleep(0.2)

    readback = dev.get_driver_value()
    print("Driver current setpoint readback (mA):", readback)

    dev.end_threads = True
    dev.temperature_thread.join()
    dev.dev.close()
    print("Exited cleanly")





