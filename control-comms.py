import json
from enum import Enum
from functools import total_ordering
from typing import List, Tuple, Union

import serial
import serial.tools.list_ports

# Communication constants
TX_KEY_ACTION = "action"
TX_KEY_COMMAND = "command"
RX_KEY_STATUS = "status"
RX_KEY_TIMESTAMP = "timestamp"
RX_KEY_TERMINATED = "terminated"
RX_KEY_OBSERVATION = "observation"

class StatusCode(Enum):
    """Status codes for the ControlComms class

    Attributes:
        OK (int): A function returns successfully
        ERROR (int): An error occurred in the function
    """
    OK = 0
    ERROR = 1

@total_ordering
class DebugLevel(Enum):
    """Available debug levels for ControlComms

    Attributes:
        DEBUG_NONE (int): No debugging info printed to the console
        DEBUG_ERROR (int): Only print error messages
        DEBUG_WARN (int): Print warning and error messages
        DEBUG_INFO (int): Print info, warning, and error messages
    """
    DEBUG_NONE = 0
    DEBUG_ERROR = 1
    DEBUG_WARN = 2
    DEBUG_INFO = 3

    def __lt__(self, other):
        """Allow for < or > comparisons among the attributes"""
        if self.__class__ is other.__class__:
            return self.value < other.value
        return NotImplemented

class ControlComms:
    """Interface class to send commands and receive observations

    Instantiate this class and pass the location of the serial port. The object
    will open a serial connection to the attached hardware. You can call the
    step() function with your desired command and/or actions. It will wait for
    a response (up to the given timeout) and return the observation details.
    
    Attributes:
        ser (Serial): Serial port object
        timeout (float): Number of seconds to wait for a response
        debug_level (int): How much debugging info to print to the screen
    """

    def __init__(
        self, 
        timeout: float = 1.0, 
        debug_level: DebugLevel = DebugLevel.DEBUG_NONE
    ) -> None:
        """Constructor to assign attributes

        Args:
            timeout_ms (int): Number of milliseconds to wait for a response
            debug_level (int): How much debugging info to print to the screen
        """
        self.ser = serial.Serial()
        self.set_timeout(timeout)
        self.debug_level = debug_level

    def __del__(self) -> None:
        """Destructor: make sure to close the serial port"""
        self.close()

    def set_timeout(
        self, 
        timeout: float = 1.0
    ) -> None:
        """Set the amount of time (ms) to wait for a response

        Args:
            timeout_ms (int): Number of milliseconds to wait for a response
        """
        self.ser.timeout = timeout

    def get_serial_list(self) -> Tuple[Tuple[str, str, str], ...]:
        """Get a list of available serial ports

        Returns:
            Tuple of serial port information (each stored in a tuple)
        """
        serial_list = []
        for port, desc, hwid in serial.tools.list_ports.comports():
            serial_attrs = (port, desc, hwid)
            serial_list.append(serial_attrs)

        return tuple(serial_list)

    def connect(
        self, 
        port: str, 
        baud_rate: int = 115200
    ) -> StatusCode:
        """Attempt to connect to the given serial port

        Args:
            port (str): Location of port file (Linux/Mac) or COM string (Win)
            baud_rate (int): Baud rate of the connection

        Returns:
            OK for connection made, ERROR otherwise
        """

        # Try closing the port first (just in case it is still open)
        try:
            self.close()
        except Exception as e:
            if self.debug_level >= DebugLevel.DEBUG_ERROR:
                print(f"Error closing connection: {e}")
            return StatusCode.ERROR

        # Update the port settings
        self.ser.port = port
        self.ser.baudrate = baud_rate

        # Try to open a connection
        try:
            self.ser.open()
            ret = StatusCode.OK
        except Exception as e:
            if self.debug_level >= DebugLevel.DEBUG_ERROR:
                print(f"Error opening connection: {e}")
            ret = StatusCode.ERROR

        return ret

    def close(self) -> None:
        """Close the serial port"""
        self.ser.close()

    def step(
        self,
        command: int,
        action: List[float]
    ) -> Union[None, Tuple[int, int, bool, Tuple[float, ...]]]:
        """Send a command and action list, wait for observation in return

        Send a command and list of action values to the driver hardware. Wait
        for the given `timeout_ms` amount of time for a response from the
        driver. The response will be returned as several values.

        Args:
            command (int): Command value (user-defined) to send to the driver
            action (List[float]): List of actions to send to the driver

        Returns:
            tuple: A tuple containing:
                - status (int): User-defined status code for the driver
                - timestamp (int): User-defined timestamp
                - terminated (bool): True if the driver stopped, False otherwise
                - observation (Tuple[float]): Tuple of observations (floats)
        """
        
        # Send out the command
        try:
            msg = f"{{\"command\":{command},\"action\":{action}}}"
            self.ser.write(bytes(msg, encoding='utf-8'))
        except Exception as e:
            if self.debug_level >= DebugLevel.DEBUG_ERROR:
                print(f"Error sending message: {e}")
            return None

        # Wait for a response
        try:
            msg = self.ser.read_until('\n'.encode('utf-8'))
        except Exception as e:
            if self.debug_level >= DebugLevel.DEBUG_ERROR:
                print(f"Error receiving message: {e}")
            return None

        # Parse response
        try:
            msg = msg.decode('utf-8')
            data = json.loads(msg)
        except Exception as e:
            if self.debug_level >= DebugLevel.DEBUG_ERROR:
                print(f"Error parsing message: {e}")
            return None

        # Return tuple with results
        return (
            data[RX_KEY_STATUS], 
            data[RX_KEY_TIMESTAMP], 
            data[RX_KEY_TERMINATED],
            data[RX_KEY_OBSERVATION]
        )

if __name__ == "__main__":

    # TEST constants for stepper and encoder
    STATUS_OK = 0
    STATUS_STP_MOVING = 1
    CMD_RESET = 0
    CMD_MOVE_TO = 1
    CMD_MOVE_BY = 2

    # TEST
    SERIAL_PORT = "COM6"
    BAUD_RATE = 115200

    # Create interface
    ctrl = ControlComms(timeout = 1.0, debug_level = DebugLevel.DEBUG_ERROR)

    # List available serial ports
    print("Available USB ports:")
    serial_ports = ctrl.get_serial_list()
    if serial_ports:
        for port in serial_ports:
            print(f"  {port}")
    else:
        print("  No serial ports found")

    # Open serial port
    ctrl.connect(SERIAL_PORT, BAUD_RATE)

    # TEST: write
    ret = ctrl.step(2, [-5.0])

    if ret:
        for val in ret:
            print(val)