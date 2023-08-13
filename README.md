# Inverted Pendulum with PID Controller Demo

This code controls [the inverted pendulum kit from STMicroelectronics](https://www.digikey.com/en/products/detail/stmicroelectronics/STEVAL-EDUKIT01/11696333). It uses the Arduino (STM32 NUCLEO-F401RE) that comes in the kit as the simple interface to the stepper motor and encoder. You call the `.step()` function from Python to send commands over USB serial to the Arduino board. This function tells the Arduino how far to move the stepper motor and returns with the encoder and stepper motor angles. This information is used to create a simple PID controller in Python.

Note that you can also use this interface to design any controller you wish in Python! However, the example Jupyter Notebook in this repo uses a simple PID controller.

## Getting Started

Construct the kit and connect the motor driver board to the Arduino as shown in [the kit's getting started guide](https://www.st.com/en/evaluation-tools/steval-edukit01.html#documentation).

Upload *pendulum-controller.ino* to the Arduino board. Make a note of the baud rate! By default, it is set to 500k, which means you will need a decent cable to ensure good communication. If you have communication issues, try a different cable or lowering the baud rate.

Open *pendulum-pid.ipynb* in [Jupyter Notebook or JupyterLab](https://docs.jupyter.org/en/latest/install.html#new-to-python-and-jupyter). Ensure that the `SERIAL_PORT` matches the serial port of your Arduino board and `BAUD_RATE` matches the baud rate set in the Arduino sketch (e.g. 500000). Run through the cells to control the stepper motor and (hopefully) balance the pendulum. As with any PID controller, you will likely need to tune the `K_P`, `K_I`, and `K_D` parameters.

## Copyright

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