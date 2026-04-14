# -*- coding: utf-8 -*-
"""
SF8xxx controller library

SF8xxx: control a board
Command classes: Set and Get things on/from board
Response: deals with response data

***NO NEGATIVE NUMBERS***

Created on Fri May 17 13:35:35 2024

@author: drm1g20
"""

import threading
import serial
import time
import sys

class SF8xxx:
    """
    Object handling I/O to and from SF8xxx.
    """
    def __init__(self, port, start_thread=True):
        self.port = port

        try:
            self.dev = serial.Serial(port, 115200, timeout=0.2)
        except serial.SerialException:
            self.connected = False
            return
        self.connected = True

        self.__lock = threading.Lock()
        self.end_threads = False

        # get details and initial status
        self.serial_no = self.get_serial_no()
        
        self.driver_off = not self.driver_state()[1]
        self.tec_off = not self.tec_state()[0]

        self.temperature = self.get_tec_temperature()

        # start temperature limit thread
        # (had issues with TEC turning off spontaneously while driver is on)
        temperature_threshold = 5
        poll_interval = 2
        self.temperature_thread = threading.Thread(target=self.poll_tec_temperature,
                                              args=(temperature_threshold,
                                                    poll_interval,))
        self.temperature_thread.start()

    
    def __del__(self):
        if not self.connected:
            return
        self.end_threads = True
        self.temperature_thread.join()
        self.dev.close()
        
        
    def __get_response(self, parameter):
        """
        Return Response object from getter function
        """
        with self.__lock:
            cmd = Getter(parameter)
            self.dev.write(cmd.data_bytes())
                
            return Response(self.dev.read_until(expected='\r'))
        
    
    def get_driver_state(self):
        """
        Return a 8-bit mask representing driver state
        """
        return self.__get_response('DRIVER_STATE').raw()
    
        
    def driver_state(self):
        """
        Returns the driver state: 
            Device, Driver, Current, Enable, NTC, Interlock
            ON/OFF, ON/OFF, INT/EXT, INT/EXT, DENY/ALLOW, DENY/ALLOW
        """
        state = self.get_driver_state()

        device = state[3] & 0x1
        driver = state[3] & 0x2
        current = state[3] & 0x4
        enable = state[2] & 0x1
        ntc = state[2] & 0x4
        interlock = state[2] & 0x8
        
        return device, driver, current, enable, ntc, interlock
        
    
    def driver_on(self):
        """
        Print driver on/off state specifically
        """
        state = self.get_driver_state()
        
        return state[3] & 0x2
        
    
    def get_driver_value(self):
        """
        Returns driver setpoint current
        """
        return self.__get_response('DRIVER_CURRENT_VALUE').rtoi() / 10
    
    
    def get_driver_current(self):
        """
        Returns driver current measurement
        """
        return self.__get_response('DRIVER_CURRENT_MEASURED').rtoi() / 10
    
    
    def get_driver_current_max(self):
        """
        Returns max driver current
        """
        return self.__get_response('DRIVER_CURRENT_MAXIMUM').rtoi() / 10
    
        
    def get_tec_state(self):
        """
        Return a bit mask representing driver state
        """
        return self.__get_response('TEC_STATE').raw()
    
    
    def tec_state(self):
        """
        Return TEC state:
            TEC, temp set, enable
            ON/OFF, INT/EXT, INT/EXT
        """
        state = self.get_tec_state()
        
        tec = state[3] & 0x2
        temp = state[3] & 0x4
        enable = state[2] & 0x1
        
        return tec, temp, enable
        
    
    def tec_on(self):
        state = self.get_tec_state()
        
        return state[3] & 0x2
        
        
    def get_tec_value(self):
        """
        Returns TEC setpoint temperature
        """
        return self.__get_response('TEC_TEMPERATURE_VALUE').rtoi() / 100
    
    
    def get_tec_temperature(self):
        return self.__get_response('TEC_TEMPERATURE_MEASURED').rtoi() / 100
    
    
    def get_tec_current(self):
        res = self.__get_response('TEC_CURRENT_MEASURED')
        current = res.rtoi()
            
        return current / 10
    
    
    def get_tec_current_limit(self):
        return self.__get_response('TEC_CURRENT_LIMIT').rtoi() / 10

    
    def get_lock_state(self):
        return self.__get_response('LOCK_STATE').raw()     


    def lock_state(self):
        """
        Return lock state:
            interlock, LD overcurrent, LD overhead, NTC, TEC error, TEC heat?
            ON/OFF, ON/OFF, ON/OFF, ON/OFF, ON/OFF, ON/OFF
        """
        state = self.get_lock_state()
        
        interlock = state[3] & 0x2
        ld_overcurrent = state[3] & 0x8
        ld_overheat = state[2] & 0x1
        ntc = state[2] & 0x2
        tec_error = state[2] & 0x4
        tec_selfheat = state[2] & 0x8
        
        return interlock, ld_overcurrent, ld_overheat, ntc, tec_error, \
                tec_selfheat


    def allow_interlock(self):
        self.__set_routine('DRIVER_STATE', 0x1000)


    def deny_interlock(self):
        self.__set_routine('DRIVER_STATE', 0x2000)


    def get_serial_no(self):
        return self.__get_response('SERIAL_NO').rtoi()


    def get_pid_p(self):
        return self.__get_response('PID_P').rtoi()
        

    def get_pid_i(self):
        return self.__get_response('PID_I').rtoi()
    

    def get_pid_d(self):
        return self.__get_response('PID_D').rtoi()
        
  
    def __set_routine(self, parameter, value):
        with self.__lock:
            cmd = Setter(parameter, value)
            self.dev.write(cmd.data_bytes())
            
            res = Response(self.dev.read_until(expected='\r'), 'set')
            if res.state == 'error':
                return 1
            
            return res
    
    
    def set_driver_state(self):
        # internal enables
        self.__set_routine('DRIVER_STATE', 0x0020)
        self.__set_routine('DRIVER_STATE', 0x0400)
        # deny ext NTC
        self.__set_routine('DRIVER_STATE', 0x4000)

            
    
    def set_driver_on(self):
        if self.tec_off:
            return 'tec'
        
        if type(self.__set_routine('DRIVER_STATE', 0x0008)) != None:
            self.driver_off = False
            return 0
        else:
            return str(self.serial_no) + "Failed to set driver on"
            
        
    def set_driver_off(self):
        if type(self.__set_routine('DRIVER_STATE', 0x0010)) != None:
            self.driver_off = True
            return 0
        else:
            return str(self.serial_no) + "Failed to set driver off"
            
    
    def set_driver_current_max(self, current_mA):
        self.__set_routine('DRIVER_CURRENT_MAXIMUM', current_mA * 10)
        
    
    def set_driver_current(self, current_mA):
        self.__set_routine('DRIVER_CURRENT_VALUE', current_mA * 10)

    
    def set_tec_temperature(self, temp_C):
        self.__set_routine('TEC_TEMPERATURE_VALUE', temp_C * 100)

        self.temperature = temp_C

    
    def set_tec_int(self):
        # internal enables
        self.__set_routine('TEC_STATE', 0x0020)
        self.__set_routine('TEC_STATE', 0x0400)
    
    
    def set_tec_on(self):
        if type(self.__set_routine('TEC_STATE', 0x0008)) != None:
            # Wait for the firmware to process the Start command and update the
            # TEC state register before reading it back.  The SF8025-NM does
            # not acknowledge P-type commands, so __set_routine returns after a
            # 200 ms read timeout -- but the state register may not yet reflect
            # the new TEC status at that instant.  500 ms is sufficient.
            time.sleep(0.5)
            if self.tec_on():
                self.tec_off = False
                return 0

            else:
                self.tec_off = True
                return str(self.serial_no) + "Failed to set TEC on. Interlock?"

        else:
            return str(self.serial_no) + "Failed to set TEC on"
        
        
    def set_tec_off(self):
        if not self.driver_off:
            return 'driver'
        
        if type(self.__set_routine('TEC_STATE', 0x0010)) != None:
            self.tec_off = True
            return 0
        else:
            return str(self.serial_no) + "Failed to set TEC off"


    def poll_tec_temperature(self, tolerance, poll_interval):
        """
        Will turn off driver if the TEC temperature rises 5 deg > setpoint
        To be run as a thread
        """
        while True:
            if self.end_threads:
                break
            
            threshold = self.temperature + tolerance

            temperature = self.get_tec_temperature()
            
            if temperature > threshold:
                self.set_driver_off()
                print("Device", self.serial_no,
                      ": Temperature (" + str(temperature)
                      + ") exceeds set threshold!!! Driver off.", end='\n')
                print("> ", end='')
                
            time.sleep(poll_interval)
            
        sys.exit(0)

    
class Command:
    """
    Builds SF8xxx byte arrays from input parameters.
    """
    def __init__(self):
        self.terminator = 0x0D
        
        self.parameters = {
            'DRIVER_STATE': '0700',
            'DRIVER_CURRENT_VALUE': '0300',
            'DRIVER_CURRENT_MAXIMUM': '0302',
            'DRIVER_CURRENT_MAXIMUM_LIMIT': '0306',
            'DRIVER_CURRENT_MEASURED': '0307',
            'DRIVER_VOLTAGE_MEASURED': '0407',
            
            'TEC_STATE': '0A1A',
            'TEC_TEMPERATURE_VALUE': '0A10',
            'TEC_TEMPERATURE_MAXIMUM': '0A11',
            'TEC_TEMPERATURE_MAXIMUM_LIMIT': '0A13',
            'TEC_TEMPERATURE_MEASURED': '0A15',
            'TEC_CURRENT_MEASURED': '0A16',
            'TEC_CURRENT_LIMIT': '0A17',
            'TEC_VOLTAGE_MEASURED': '0A18',
            
            'LOCK_STATE': '0800',

            'PID_P': '0A21',
            'PID_I': '0A22',
            'PID_D': '0A23',
            
            'SERIAL_NO': '0701'
            }
        
    
    def set_parameter(self, parameter):
        """
        parameter: hex string
        """
        self.data[1:5] = parameter.encode('ascii')
        
        
    def data_bytes(self):
        return bytes(self.data)
    
    
    def data_print(self):
        print(self.data_bytes())
        

class Setter(Command):
    """
    SF8xxx set "P" type commands.
    """
    def __init__(self, parameter, value):
        Command.__init__(self)
        self.data = bytearray(11)
        self.data[0] = 0x50
        self.data[-1] = self.terminator
        self.data[5] = 0x20
        self.set_parameter(self.parameters[parameter])
        self.set_input(value)
        
    
    # work on this one
    def set_input(self, value):
        """
        value: integer
        """
        def hextoa(val):
            val_len = 4
            val = str(hex(val))[2:].upper()
            
            while len(val) < val_len:
                val = '0' + val
            
            return val
        
        
        value = int(value)
        value = hextoa(value)

            
        self.data[6:10] = value.encode('ascii')

    
class Getter(Command):
    """
    SF8xxx request "J" type commands.
    """

    def __init__(self, parameter):
        Command.__init__(self)
        self.data = bytearray(6)
        self.data[0] = 0x4A
        self.data[-1] = self.terminator
        self.set_parameter(self.parameters[parameter])

    
class Response:
    """
    SF8xxx response "K" structure.
    """
    def __init__(self, data, flag='get'):        
        self.data = data
        self.state = 'untested'
        
        if len(data) == 0:
            if flag == 'set':
                return
            
            print("ERROR: Response: no data")
            self.state = 'error'
            return
        
        if self.data[0] == ord('E'):
            self.state = 'error'
        
        if(self.data == b'E0000\r'):
            print("ERROR:", self.data, "No terminator/buffer/format.")
        elif(self.data == b'E0001\r'):
            print("ERROR:", self.data, "Undefined header.")
        elif(self.data == b'E0002\r'):
            print("ERROR:", self.data, "CRC.")
        
    
    def raw(self):
        """
        Return response value as bytes
        """
        return self.data[6:10]
    
    
    def rtoa(self):
        """
        Return response value as ASCII string
        """
        return self.data[6:10].decode('ascii')
    
    
    def rtoi(self):
        """
        Return response value as decimal integer
        """
        return int(self.rtoa(), 16)
    
    
    def data_print(self):
        print(self.data)