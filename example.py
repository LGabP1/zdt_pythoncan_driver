import time

from zdt_pythoncan_driver.lib_zdt_driver import ZDTEmmV50Handler, ZDTReturnCode
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

    ret = zdt.cmd_speed_mode_control(1000, 0) # Drive at given speed (in RPM)
    print(f"Command zdt.cmd_speed_mode_control(1000, 0) returned {ret}!")

    if ret == ZDTReturnCode.OK: # Successfull command
        print("Command was successful! Now monitoring speed...")
        begin_time = time.time()

        while time.time() - begin_time < 3: # Monitor speed for 3 seconds.
            ret, speed = zdt.cmd_read_current_speed() # Monitor speed.
            print(f"Command zdt.cmd_monitor_speed() returned {ret}, {speed}!")

            print(f"Speed is {speed} rpm!")
        print("Done monitoring speed!")

    ret = zdt.cmd_immediate_stop() # Stop the motor.
    print(f"Command zdt.cmd_immediate_stop() returned {ret}!")

    ret = zdt.cmd_speed_mode_control(-1000, 0) # Drive at given speed (in RPM), opposite way
    print(f"Command zdt.cmd_speed_mode_control(-1000, 0) returned {ret}!")

    if ret == ZDTReturnCode.OK: # Successfull command
        print("Command was successful! Now monitoring speed...")
        begin_time = time.time()

        while time.time() - begin_time < 3: # Monitor speed for 3 seconds.
            ret, speed = zdt.cmd_read_current_speed() # Monitor speed.
            print(f"Command zdt.cmd_monitor_speed() returned {ret}, {speed}!")

            print(f"Speed is {speed} rpm!")
        print("Done monitoring speed!")
        
    ret = zdt.cmd_immediate_stop() # Stop the motor.
    print(f"Command zdt.cmd_immediate_stop() returned {ret}!")