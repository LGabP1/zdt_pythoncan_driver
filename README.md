# ZDT python-can driver

A Python interface to drive the Emm_V5.0 Closed-Loop Stepper Drive by 张大头 (Zhang Da Tou, ZDT) through CAN.
This library is heavily based on [Emm_V5.0 Manual Rev1.3](https://blog.csdn.net/zhangdatou666/article/details/132644047), a very well documented manual.

## Requirements

**Software:**
* Python 3.10 or higher. [python](https://www.python.org/downloads/)
* python-can. [python-can](https://python-can.readthedocs.io/en/stable/installation.html)

**Hardware:**
This library uses python-can as a backend to communicate with the CAN bus. One should thus use any compatible CAN USB interface. Such interfaces only need to support classical CAN for baudrates of at least $200 000$.

For example, [CANable](https://canable.io/)-based devices are relatively cheap devices that will do the job. Can be found for a few dollars.

## Installation

This project has a simple folder structure. To use it as a library, simply copy the `zdt_pythoncan_driver` folder into your project, the relevant classes can then be imported directly.

##  Usage

Please note that since v1.01 of this package, multiple controllers should be controllable simultaneously.

This is a simple code example that will send a position command.

```python
from zdt_pythoncan_driver.lib_zdt_driver import ZDTEmmV50Handler
from zdt_pythoncan_driver.lib_bytes import Checksums
from zdt_pythoncan_driver.lib_logger import MyLogger

if __name__ == "__main__":
    # ==== Create logger, will log with DEBUG level to console only. ====
    logger = MyLogger(file_level=-1, console_level=10)
    
    # ==== Create ZDT handler object and open the connection. ====
    zdt = ZDTEmmV50Handler('slcan', 'COM6', 500_000, 0x01, Checksums.CS0x6B, logger=logger)

    # ==== Send basic commands ====
    ret = zdt.cmd_motor_enable_control(enable=True) # Need to enable the control before sending commands to move it
    print(f"Command zdt.cmd_motor_enable_control(enable=True) returned {ret}!")

    ret = zdt.cmd_position_mode_control(3000, 1000, 0) # Drive to given position (in pulse count)
    print(f"Command zdt.cmd_position_mode_control(3000, 1000, 0) returned {ret}!")
```

Please also refer to `example.py` and to see other example with speed control. Please refer to `example_double.py` for an example code driving several ZDT devices. 

## Features

This library provides a simple interface to one or more ZDT controllers, allowing one to send commands and receive responses easily.

The following objects are of particular interest:
* `lib_zdt_driver.ZDTEmmV50Handler` - The main object to interact with the controller.
* Several dataclasses wrapping command parameters.
* Support for all checksums supported by the board through `lib_bytes.ChecksumABC` and its subclasses. These can be accessed with `lib_bytes.Checksums` enum as well for easier use. Checksums are:
    * `lib_bytes.Checksums.CS0x6B`: "0x6B" is always the ending byte. 
    * `lib_bytes.Checksums.CSXOR`: XOR checksum.
    * (`lib_bytes.Checksums.CSCRC8`: CRC8 checksum, implementation to fix)
    * (`lib_bytes.Checksums.CSModbus`: Not yet implemented)
* `lib_logger.MyLogger` - A simple `logging.Logger` subclass which can log to both console and file.

## Known issues

* Checksums which are not Checksums.CS0x6B checksums are not properly implemented.  
* Issue with cmd_M_M_synchronized_motion where broadcasting the "Go" message does not work.
* Docstring should be completed

## Future improvements and contributing

Beside the issues cited above, it would be great to add support for more ZDT stepmotor controllers and their commands.
I expect most of the commands to be mostly similar between the different controllers, so it should be fairly easy to add support for them but I have only access to the Emm_V5.0 controller.

I would be happy to receive any contributions to this project. Please also feel free to fork and use the code as you see fit.

## License

This project is licensed under the MIT License.

## Acknowledgements

This library is heavily based on [Emm_V5.0 Manual Rev1.3](https://blog.csdn.net/zhangdatou666/article/details/132644047), I would like to thank the author for the well documented manual with its explicit examples.

## Version history

* `v1.00` Initial release
* `v1.01` New CAN handling interface allowing several ZDT devices
* `v1.02` Updated and fixed commands 
* `v1.03` Fixed endianness issues, static bus instances to allow multiple devices handling
* `v1.04` Debug code allowing simultaneous control of several ZDT devices