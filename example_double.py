import time

from zdt_pythoncan_driver.lib_zdt_driver import ZDTEmmV50Handler, ZDTReturnCode
from zdt_pythoncan_driver.lib_bytes import Checksums
from zdt_pythoncan_driver.lib_logger import MyLogger

if __name__ == "__main__":
    # ==== Create logger, will log with DEBUG level to console only. ====
    logger = MyLogger(file_level=-1, console_level=10)
    
    # ==== Create ZDT handler object and open the connection. ====
    while True:
        try:
            zdt1 = ZDTEmmV50Handler('slcan', 'COM6', 500_000, 0x01, Checksums.CS0x6B, logger=logger, recv_timeout=0.3)
            break
        except PermissionError as e:
            logger.error(f"Failed to setup CAN for zdt1: {e}")
    while True:
        try:
            zdt2 = ZDTEmmV50Handler('slcan', 'COM6', 500_000, 0x02, Checksums.CS0x6B, logger=logger, recv_timeout=0.3)
            break
        except PermissionError as e:
            logger.error(f"Failed to setup CAN for zdt2: {e}")

    # ==== Send basic commands ====
    ret = zdt1.cmd_motor_enable_control(enable=True) # Need to enable the control before sending commands to move it
    print(f"Command zdt1.cmd_motor_enable_control(enable=True) returned {ret}!")
    time.sleep(0.5)
    ret = zdt2.cmd_motor_enable_control(enable=True) # Need to enable the control before sending commands to move it
    print(f"Command zdt2.cmd_motor_enable_control(enable=True) returned {ret}!")
    time.sleep(0.5)

    try:
        while True:
            zdt1.cmd_speed_mode_control(100, 0, False)
            print(f"Command zdt1.cmd_speed_mode_control(enable=True) returned {ret}!")
            time.sleep(0.5)
            
            zdt2.cmd_speed_mode_control(100, 0, False)
            print(f"Command zdt2.cmd_speed_mode_control(enable=True) returned {ret}!")
            time.sleep(0.5)

            ret = zdt1.cmd_immediate_stop(False)
            print(f"Command zdt1.cmd_immediate_stop(enable=True) returned {ret}!")
            time.sleep(0.3)

            ret = zdt2.cmd_immediate_stop(False)
            print(f"Command zdt2.cmd_immediate_stop(enable=True) returned {ret}!")
            time.sleep(0.3)
            
            zdt1.cmd_speed_mode_control(-100, 0, False)
            print(f"Command zdt1.cmd_speed_mode_control(enable=True) returned {ret}!")
            time.sleep(0.5)
            
            zdt2.cmd_speed_mode_control(-100, 0, False)
            print(f"Command zdt2.cmd_speed_mode_control(enable=True) returned {ret}!")
            time.sleep(0.5)

            ret = zdt1.cmd_immediate_stop(False)
            print(f"Command zdt1.cmd_immediate_stop(enable=True) returned {ret}!")
            time.sleep(0.3)

            ret = zdt2.cmd_immediate_stop(False)
            print(f"Command zdt2.cmd_immediate_stop(enable=True) returned {ret}!")
            time.sleep(0.3)
            
    finally:
        ret = zdt1.cmd_immediate_stop(False)
        print(f"Command zdt2.cmd_immediate_stop(enable=True) returned {ret}!")
        ret = zdt2.cmd_immediate_stop(False)
        print(f"Command zdt2.cmd_immediate_stop(enable=True) returned {ret}!")