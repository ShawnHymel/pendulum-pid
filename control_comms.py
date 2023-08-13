"""
Communication interface for actions and observations over serial

Author:
    Shawn Hymel

Date:
    Created on: 2023-08-05

Version:
    0.1

This interface provides one main function, step(), and other supporting
functions to initialize a serial connection with the counterpart interface
running on a microcontroller or single-board computer.

Initialize the serial port by calling connect(). Call step() with the user-
defined command and action array. The command and action are sent to the
connected hardware, which performs the desired action and returns with 
a status, timestamp, terminated, and observation information as a tuple in the
following format:

    (status, timestamp, terminated, [obs0, obs1, ...])

Test this module with the command line:

    python control-comms.py -p "COM6" -b 115200 -c 2 -a 5.5 -d 1

Example:
    If you are using this module in your own program:

        ctrl = ControlComms()
        ctrl.connect("COM6", 115200)
        resp = ctrl.step(2, [5.5])
        if resp:
            status, timestamp, terminated, observation = resp

License:
    Zero-Clause BSD

    Permission to use, copy, modify, and/or distribute this software for
    any purpose with or without fee is hereby granted.

    THE SOFTWARE IS PROVIDED “AS IS” AND THE AUTHOR DISCLAIMS ALL
    WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES
    OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE
    FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
    DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
    AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
    OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
"""

import sys
import argparse
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
    """
    Status codes for the ControlComms class

    Attributes:
        OK (int): A function returns successfully
        ERROR (int): An error occurred in the function
    """
    OK = 0
    ERROR = 1

@total_ordering
class DebugLevel(Enum):
    """
    Available debug levels for ControlComms

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
    """
    Interface class to send commands and receive observations

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
        """
        Constructor to assign attributes

        Args:
            timeout_ms (int): Number of milliseconds to wait for a response
            debug_level (int): How much debugging info to print to the screen
        """
        self.ser = serial.Serial()
        self.set_timeout(timeout)
        self.debug_level = debug_level

    def __del__(self) -> None:
        """
        Destructor: make sure to close the serial port
        """
        self.close()

    def set_timeout(
        self, 
        timeout: float = 1.0
    ) -> None:
        """
        Set the amount of time (ms) to wait for a response

        Args:
            timeout_ms (int): Number of milliseconds to wait for a response
        """
        self.ser.timeout = timeout

    def get_serial_list(self) -> Tuple[Tuple[str, str, str], ...]:
        """
        Get a list of available serial ports

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
        """
        Attempt to connect to the given serial port

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
        """
        Close the serial port
        """
        self.ser.close()

    def step(
        self,
        command: int,
        action: List[float]
    ) -> Union[None, Tuple[int, int, bool, Tuple[float, ...]]]:
        """
        Send a command and action list, wait for observation in return

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
        except ValueError as e:
            if self.debug_level >= DebugLevel.DEBUG_ERROR:
                print(f"Error parsing message. Message received: {msg}")
            return None

        # Return tuple with results
        return (
            data[RX_KEY_STATUS], 
            data[RX_KEY_TIMESTAMP], 
            data[RX_KEY_TERMINATED],
            data[RX_KEY_OBSERVATION]
        )

if __name__ == "__main__":

    # Defaults
    DEFAULT_BAUD = 115200

    # Command line arguments
    parser = argparse.ArgumentParser(description="Serial control interface")
    parser.add_argument(
        '-p',
        '--port',
        dest='port',
        type=str,
        required=False,
        help="Serial port to connect to. Omit to print out all available ports."
    )
    parser.add_argument(
        '-b',
        '--baud',
        dest='baud',
        type=int,
        default=DEFAULT_BAUD,
        help="Baud rate (default = " + str(DEFAULT_BAUD) + ")."
    )
    parser.add_argument(
        '-c',
        '--command',
        dest='command',
        type=int,
        default=0,
        help="User-defined command (integer) to send (default = 0)."
    )
    parser.add_argument(
        '-a',
        '--action',
        type=float,
        nargs='+',
        help="One or more space-separated list of user-defined numbers " \
             "(float) to send."
    )
    parser.add_argument(
        '-d',
        '--debug',
        type=int,
        default=0,
        help="Debug level: none=0, error=1, warn=2, info=3 (default = 0)."
    )
                            
    # Parse arguments
    args = parser.parse_args()

    # Check debug level
    try:
        debug_level = DebugLevel(args.debug)
    except ValueError as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    # Create interface
    ctrl = ControlComms(timeout=1.0, debug_level=debug_level)

    # Print available serial ports if port not given
    if not args.port:
        print("Argument '--port' required. Available USB ports:")
        serial_ports = ctrl.get_serial_list()
        if serial_ports:
            for port in serial_ports:
                print(f"  {port}")
        else:
            print("  No serial ports found")
        sys.exit(0)

    # Open serial port
    ctrl.connect(args.port, args.baud)

    # Send command and action, wait for response
    resp = ctrl.step(args.command, args.action)

    # Print response as JSON string
    if resp:
        data = {}
        data[RX_KEY_STATUS] = resp[0]
        data[RX_KEY_TIMESTAMP] = resp[1]
        data[RX_KEY_TERMINATED] = resp[2]
        data[RX_KEY_OBSERVATION] = resp[3]
        print(json.dumps(data))
        sys.exit(0)
    else:
        sys.exit(1)
