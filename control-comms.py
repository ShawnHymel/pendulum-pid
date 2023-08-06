import serial
import serial.tools.list_ports


if __name__ == "__main__":

    # TEST
    SERIAL_PORT = "COM6"
    BAUD_RATE = 115200

    # List available serial ports
    print("Available USB ports")
    serial_ports = serial.tools.list_ports.comports()
    if serial_ports:
        for port, desc, hwid in serial_ports:
            print("  {} : {} [{}]".format(port, desc, hwid))
    else:
        print("No serial ports found")

    # Open serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

    # TEST write
    ser.write(b'{"command":2,"action":[-10]}')