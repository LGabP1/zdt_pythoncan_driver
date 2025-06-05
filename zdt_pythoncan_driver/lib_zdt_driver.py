"""
Describe ZDT commands
"""
from .lib_interface import ZDTCANInterface
from .lib_bytes import ChecksumABC, Checksums
from typing import Optional, Literal
from logging import Logger
from enum import IntEnum
from dataclasses import dataclass
from itertools import chain

class ZDTReturnCode(IntEnum):
    OK = 0x00 # Return code
    NONE = 0x01 # No return received or checksum mismatch
    ERROR = 0x02 # Error (00 ee message)
    CONDITIONS_NOT_MET = 0x03 # Conditions not met notice (00 e2 message)

@dataclass
class ZDTDriveConfigurationParameters:
    """Drive Configuration Parameters, used in 0x42 (read) and 0x48 (modify) commands.
    
    Note: received_bytes and num_config_params be left unset (is not used) when sending 0x48 modify command."""
    mot_type: int
    pulse_port_control_mode: int
    communication_port_multiplexing_mode: int
    en_pin_active_level: int
    dir_pin_effective_direction: int
    microstep: int
    microstep_interpolation: bool
    auto_screen_off: bool
    open_loop_mode_operating_current: int
    closed_loop_mode_maximum_stall_current: int
    closed_loop_mode_maximum_output_voltage: int
    serial_baud_rate: int
    CAN_communication_rate: int
    ID_address: int
    communication_checksum_method: int
    control_command_response: int
    stall_protection: bool
    stall_protection_speed_threshold: int
    stall_protection_current_threshold: int
    stall_protection_detection_time_threshold: int
    position_arrival_window: int
    received_bytes: int = 0
    num_config_params: int = 0

@dataclass
class ZDTSystemStatusParameters:
    """System Status Parameters, used in 0x43 (read) command.

    Parameters:
        received_bytes (int in [0,255]): Number of bytes in the command 
        num_config_params (int in [0,255]): Number of configuration parameters in the command
        bus_voltage (int in [0,65535]): In mV.
        bus_phase_current (int in [0,65535]): In mA.
        calibrated_encoder_value (int in [0,65535]): 0-65535 being a full rotation.
        motor_target_position (int in [-4294967295,4294967295]): 0-65535 being a full rotation.
        motor_real_time_speed (int in [-65535,65535]): In RPM.
        motor_real_time_position (int in [-4294967295,4294967295]): 0-65535 being a full rotation.
        motor_position_error (int in [-4294967295,4294967295]): 0-32767=0x7FFF being a full turn.
        flag_encoder_ready_status (bool): Whether encoder ready.
        flag_calibration_ready_status (bool): Whether calibration ready
        flag_homing_in_progress (bool): Whether Homing in progress.
        flag_homing_failure (bool): Whether Homing failure.!
        flag_enable_motor_status (bool): Whether motor enabled.
        flag_motor_in_position_flag (bool): Whether motor is in position.
        flag_motor_stall (bool): Whether motor is in stall mode.
        flag_stall_protection (bool): Whether motor is stall protected."""
    received_bytes: int
    num_config_params: int
    bus_voltage: int
    bus_phase_current: int
    calibrated_encoder_value: int
    motor_target_position: int
    motor_real_time_speed: int
    motor_real_time_position: int
    motor_position_error: int
    flag_encoder_ready_status: bool
    flag_calibration_ready_status: bool
    flag_homing_in_progress: bool
    flag_homing_failure: bool
    flag_enable_motor_status: bool
    flag_motor_in_position_flag: bool
    flag_motor_stall: bool
    flag_stall_protection: bool

@dataclass
class ZDTHomingParameters:
    """Homing parameters, used in 0x22 (read) and 0x4C (modify) commands.

    Parameters:
        - mode (Literal["Nearest", "Directional Nearest", "Collision", "Limit Switch"]): see below for modes description.
        - direction (Literal["Clockwise", "CounterClockwise"]): direction "Clockwise" or "CounterClockwise".
        - speed (int in [0, 65535]): homing speed in RPM.
        - timeout (int in [0, 4294967295]): homing timeout in ms.
        - detection_speed (int in [0, 65535]): Limitless Collision Homing Detection Speed in RPM.
        - detection_current (int in [0, 65535]): Limitless Collision Homing Detection Current in mA.
        - detection_time (int in [0, 65535]): Limitless Collision Homing Detection Time in ms.
        - enable_pwr_on_auto_trigger (bool): Enable Power-On Auto-Trigger Homing Function. False is not enabled, True is enabled.

    Homing modes:
        - "Nearest": Go to homing 0-position the nearest way (can go clockwise or counterclockwise). Will rotate 1 turn max.
        - "Directional Nearest": Rotates in given direction up to reaching 0-position (nearest but one way). Will rotate 1 turn max.
        - "Collision": The motor will rotate until it detects a collision (see criteria below). Can rotate without limits.
        - "Limit Switch": The motor will rotate until the external limit switch is triggered. Please refer to ZDT's documentation for the wiring. Can rotate without limits.
    """ 
    mode: Literal["Nearest", "Directional Nearest", "Collision", "Limit Switch"]
    direction: Literal["Clockwise", "CounteClockwise"] 
    speed: int
    timeout: int
    detection_speed: int
    detection_current: int
    detection_time: int
    enable_pwr_on_auto_trigger: bool

@dataclass
class ZDTMotorStatusFlags:
    """Motor Status Flags, used in 0x3A (read).
    
    Flags:
        enabled (bool): If Motor Enable Status Flag is True.
        in_pos (bool): If Motor In-Position Flag is True.
        stalled (bool): If Motor Stall Flag is True.
        stall_protection (bool): If Motor Stall Protection Flag is True."""
    enabled: bool
    in_pos: bool
    stalled: bool
    stall_protection: bool

@dataclass
class ZDTHomingStatusFlags:
    """Homing Status Flags, used in 0x3B (read).

    Flags:
        enc_ready (bool): If Encoder Ready Status Flag is True.
        CAL_table_ready (bool): If Calibration Table Ready Status Flag is True.
        homing_in_progress (bool): If Homing in Progress Flag is True.
        homing_failure (bool): If Homing Failure Flag is True."""
    enc_ready: bool
    CAL_table_ready: bool
    homing_in_progress: bool
    homing_failure: bool

class ZDTEmmV50Handler:
    """Handle ZDT_EmmV5.0Rev1.3 controller."""

    def __init__(self, interface: str, channel: str, bitrate: int, mot_id: int, checksum: ChecksumABC = Checksums.CS0x6B, send_timeout: float = 1.0, recv_timeout: float = 1.0, logger: Optional[Logger] = None):
        """Handle ZDT_EmmV5.0Rev1.3 controller.
        
        Args: 
            interface (str): CAN USB interface. e.g.: 'slcan'
            channel (str): Channel to use. e.g.: 'COM6'
            bitrate (str): Bus bitrate. e.g.: 500_000
            mot_id (int): ID of the target motor device. e.g.: 1
            checksum (ChecksumABC, optional): Checksum to use. Defaults to Checksums.CS0x6B
            send_timeout (float, optional): Timeout time (in s) for sending CAN packets. Defaults to 1.0.
            recv_timeout (float, optional): Timeout time (in s) for receiving CAN packets. Defaults to 1.0.
            logger (Logger, optional): Optional logger.
            """
        self.can_interface = ZDTCANInterface(interface, channel, bitrate, logger=logger) # can.Bus object wrapper
        self.mot_id = mot_id
        self.checksum = checksum
        self.send_timeout = send_timeout
        self.recv_timeout = recv_timeout
        self.logger = logger

        self.can_interface.open()
    
    def set_checksum(self, checksum: ChecksumABC) -> None:
        """Update checksum algorithm to use.
        
        Args:
            checksum (ChecksumABC): Checksum to use."""
        self.checksum = checksum
    
    def set_mot_id(self, mot_id: int) -> None:
        """Update ID of the target motor device.
        
        Args:
            mot_id (int): ID of the target motor device."""
        self.mot_id = mot_id

    @staticmethod
    def combined_payloads(*payloads: bytearray) -> bytearray:
        """Returns combined payloads."""
        combined = bytearray()
        if len(payloads) <= 0:
            return combined
        combined.extend(payloads[0])
        for i in range(1, len(payloads)):
            combined.extend((payloads[i])[1:]) # extend with removed command byte
        return combined

    def _check_checksum(self, recv_data: bytearray) -> bool:
        """Returns True if recv_data has corresponding checksum."""
        checksum_expected_value = recv_data[-1]
        checksum_computed_value = self.checksum.get(recv_data[:-1])[0]
        return checksum_expected_value == checksum_computed_value

    def cmd_trigger_encoder_CAL(self) -> ZDTReturnCode:
        """Command 0x06: Trigger Encoder Calibration
        Trigger Encoder Calibration.
        
        Args: None

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received.
        - ZDTReturnCode.CONDITIONS_NOT_MET if conditions not met"""
        payload = bytearray([0x06, 0x45])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x06 and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0x06 and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
    
    def cmd_reset_position_to_0(self) -> ZDTReturnCode:
        """Command 0x0A: Reset Current Position Angle to 0.
        Reset the current position angle, position error, and pulse count to zero.

        Args: None

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""
        payload = bytearray([0x0A, 0x6D])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xee: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x0A and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def cmd_release_stall_protection(self) -> ZDTReturnCode:
        """Command 0x0E: Release Stall Protection
        If a motor stall, send this command to release the stall protection. If motor is not in stalled mode, the conditions are not met.
        
        Args: None

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received.
        - ZDTReturnCode.CONDITIONS_NOT_MET if conditions not met

        Note: for some reason, seem to return 0E 02 even if the motor is not stall protected"""
        payload = bytearray([0x0E, 0x52])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x0E and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0x0E and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def cmd_factory_reset(self) -> ZDTReturnCode:
        """Command 0x0F: Factory Reset
        Restore Factory Settings.

        Args: None

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""
        payload = bytearray([0x0F, 0x5F])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x0F and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def cmd_read_firmware_and_hardware_version(self) -> tuple[ZDTReturnCode, int, int]:
        """Command 0x1F: Read Firmware Version and Corresponding Hardware Version.
        Read Firmware Version and Corresponding Hardware Version.

        Args: None

        Returns tuple[return_code: ZDTReturnCode, firmware_version: int, hardware_version: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            firmware_version (int in [0,255]): 0 if not ZDTReturnCode.OK. For example, 0xF4 is Emm42_V5.0.0
            hardware_version (int in [0,255]): 0 if not ZDTReturnCode.OK. For example, 0x78 is ZDT_X42_V1.2"""
        payload = bytearray([0x1F])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0, 0)
        if recv_data[0] == 0x1F and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            firmware_version = recv_data[1]
            hardware_version = recv_data[2]
            return (ZDTReturnCode.OK, firmware_version, hardware_version)
        return (ZDTReturnCode.NONE, 0, 0)
    
    def cmd_read_phase_resistance_and_inductance(self) -> tuple[ZDTReturnCode, int, int]:
        """Command 0x20: Read Phase Resistance and Inductance values
        Read Phase Resistance and Phase Inductance values.

        Args: None

        Returns tuple[return_code: ZDTReturnCode, resistance_value: int, phase_inductance: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            resistance_value (int in [0, 65535]): in mili Ohm. For example, 0x047A is 1146 mOhm 
            phase_inductance (int in [0, 65535]): in micro Henry. For example, 0x0D28 is 3368 microH"""
        payload = bytearray([0x20])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0, 0)
        if recv_data[0] == 0x20 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            resistance_value = int.from_bytes(recv_data[1:3], byteorder='big', signed=False)
            phase_inductance = int.from_bytes(recv_data[3:5], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, resistance_value, phase_inductance)
        return (ZDTReturnCode.NONE, 0, 0)

    def cmd_read_position_PID_parameters(self) -> tuple[ZDTReturnCode, int, int, int]:
        """Command 0x21: Read Position Loop PID Parameters
        Read Position Loop PID Parameters.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, kp: int, ki: int, kd: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            kp (int in [0, 4294967295]): kp value. Will be 0 if error.
            ki (int in [0, 4294967295]): ki value. Will be 0 if error.
            kd (int in [0, 4294967295]): kd value. Will be 0 if error."""
        payload = bytearray([0x21])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data1 = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout) # First payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0, 0, 0)
        if recv_data1[0] == 0x00 and recv_data1[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0, 0, 0)
        can_id, recv_data2 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 1, self.recv_timeout) # Second payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0, 0, 0)
        if recv_data2[0] == 0x00 and recv_data2[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0, 0, 0)
        
        recv_data = ZDTEmmV50Handler.combined_payloads(recv_data1, recv_data2)
        if recv_data[0] == 0x21 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            kp = int.from_bytes(recv_data[1:5], byteorder='big', signed=False)
            ki = int.from_bytes(recv_data[5:9], byteorder='big', signed=False)
            kd = int.from_bytes(recv_data[9:13], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, kp, ki, kd)
        return (ZDTReturnCode.NONE, 0, 0, 0)

    def cmd_read_homing_parameters(self) -> tuple[ZDTReturnCode, ZDTHomingParameters | None]:
        """Command 0x22: Read Origin Homing Parameters
        Read Origin Homing Parameters.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, homing_params: ZDTHomingParameters]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            homing_params (ZDTHomingParameters), Will be None if error:
                - mode (Literal["Nearest", "Directional Nearest", "Collision", "Limit Switch"]): see descriptions of modes below.
                - direction Literal["Clockwise", "CounterClockwise"] # direction "Clockwise" or "CounterClockwise".
                - speed (int in [0, 65535]): homing speed in RPM.
                - timeout (int in [0, 4294967295]): homing timeout in ms.
                - detection_speed (int in [0, 65535]): Limitless Collision Homing Detection Speed in RPM.
                - detection_current (int in [0, 65535]): Limitless Collision Homing Detection Current in mA.
                - detection_time (int in [0, 65535]): Limitless Collision Homing Detection Time in ms.
                - enable_pwr_on_auto_trigger (bool): Enable Power-On Auto-Trigger Homing Function. False is not enabled, True is enabled.
            
        Limitless Collision Homing Detection Criteria: 
            Motor speed < collision homing detection speed + motor phase current > collision homing detection current +
            duration > collision homing detection time.
            
        Homing mode descriptions:
            - "Nearest": Go to homing 0-position the nearest way (can go clockwise or counterclockwise). Will rotate 1 turn max.
            - "Directional Nearest": Rotates in given direction up to reaching 0-position (nearest but one way). Will rotate 1 turn max.
            - "Collision": The motor will rotate until it detects a collision (see criteria below). Can rotate without limits.
            - "Limit Switch": The motor will rotate until the external limit switch is triggered. Please refer to ZDT's documentation for the wiring. Can rotate without limits."""
        payload = bytearray([0x22])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data1 = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout) # First payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data1[0] == 0x00 and recv_data1[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data2 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 1, self.recv_timeout) # Second payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data2[0] == 0x00 and recv_data2[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data3 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 2, self.recv_timeout) # Third payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data3[0] == 0x00 and recv_data3[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)

        recv_data = ZDTEmmV50Handler.combined_payloads(recv_data1, recv_data2, recv_data3)
        if recv_data[0] == 0x22 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            mode = ("Nearest", "Directional Nearest", "Collision", "Limit Switch")[recv_data[1]]     
            direction = "Clockwise" if recv_data[2] == 0x00 else "CounterClockwise" # 
            speed = int.from_bytes(recv_data[3:5], byteorder='big', signed=False)
            timeout = int.from_bytes(recv_data[5:9], byteorder='big', signed=False)
            detection_speed = int.from_bytes(recv_data[9:11], byteorder='big', signed=False)
            detection_current = int.from_bytes(recv_data[11:13], byteorder='big', signed=False)
            detection_time =  int.from_bytes(recv_data[13:15], byteorder='big', signed=False)
            enable_pwr_on_auto_trigger = bool(recv_data[15])
            ret = ZDTHomingParameters(mode, direction, speed, timeout, detection_speed, detection_current, detection_time, enable_pwr_on_auto_trigger)
            return (ZDTReturnCode.OK, ret)
        return (ZDTReturnCode.NONE, None)

    def cmd_read_bus_voltage(self) -> tuple[ZDTReturnCode, int]:
        """Command 0x24: Read Bus Voltage
        Read Bus Voltage. Observation: this is in fact close to the powering voltage (a bit lower), so I am unsure what "Bus" refers to.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, voltage: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            voltage (int in [0, 65535]): Voltage in mV. Will be 0 if error.       
        """
        payload = bytearray([0x24])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0)
        if recv_data[0] == 0x24 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            voltage = int.from_bytes(recv_data[1:3], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, voltage)
        return (ZDTReturnCode.NONE, 0)
    
    def cmd_read_phase_current(self) -> tuple[ZDTReturnCode, int]:
        """Command 0x27: Read Phase Current
        Read Phase Current.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, current: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            current (int in [0, 65535]): Current in mA. Will be 0 if error."""
        payload = bytearray([0x27])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0)
        if recv_data[0] == 0x27 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            current = int.from_bytes(recv_data[1:3], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, current)
        return (ZDTReturnCode.NONE, 0)
    
    def cmd_read_CAL_encoder_value(self) -> tuple[ZDTReturnCode, int]:    
        """Command 0x31: Read CAL Encoder values
        Read Linearized Calibrated Encoder Value.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, lin_enc_val: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            lin_enc_val (int in [0, 65535]): Linearized Calibrated Encoder Value, 0-65535 range being a full rotation. Will be 0 if error.
        
        Note: After linearized calibration, the encoder value is quadrupled internally, with a range of 0-65535 for one full rotation"""
        payload = bytearray([0x31])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0)
        if recv_data[0] == 0x31 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            value = int.from_bytes(recv_data[1:3], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, value)
        return (ZDTReturnCode.NONE, 0)
    
    def cmd_read_input_pulse_count(self) -> tuple[ZDTReturnCode, int]: 
        """Command 0x32: Read Input Pulse Count
        Read Input Pulse Count.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, pulse_count: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            pulse_count (int in [-4294967295, 4294967295]): (Signed) input pulse count. Will be 0 if error."""
        payload = bytearray([0x32])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0)
        if recv_data[0] == 0x32 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            sign = +1 if recv_data[1] == 0x00 else -1 # get sign
            pulse_count = sign*int.from_bytes(recv_data[2:6], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, pulse_count)
        return (ZDTReturnCode.NONE, 0)
    
    def cmd_read_closed_loop_target_position(self) -> tuple[ZDTReturnCode, int]:
        """Command 0x33: Read Closed Loop Target Position.
        Read Motor Target Position.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, target_pos: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            target_pos (int in [-4294967295, 4294967295]): Motor Target Position (0-65535 being 1 full rotation). Will be 0 if error."""
        payload = bytearray([0x33])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0)
        if recv_data[0] == 0x33 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            sign = +1 if recv_data[1] == 0x00 else -1 # get sign
            target_pos = sign*int.from_bytes(recv_data[2:6], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, target_pos)
        return (ZDTReturnCode.NONE, 0)

    def cmd_read_open_loop_target_position(self) -> tuple[ZDTReturnCode, int]:
        """Command 0x34: Read Open Loop Target Pos
        Read Motor Real-Time Set Target Position (Open-Loop Mode Real-Time Position).
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, target_pos: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            target_pos (int in [-4294967295, 4294967295]): Open-Loop Mode Real-Time Position, 0x00_01_00_00 = 65536 being 360°. Will be 0 if error.
            
            Motor Real-Time Set Target Position : -0x00010000 = (−65536×360)/65536(−65536×360)/65536 = -360.0°"""
        payload = bytearray([0x34])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0)
        if recv_data[0] == 0x34 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            sign = +1 if recv_data[1] == 0x00 else -1 # get sign
            target_pos = sign*int.from_bytes(recv_data[2:6], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, target_pos)
        return (ZDTReturnCode.NONE, 0)

    def cmd_read_current_speed(self) -> tuple[ZDTReturnCode, int]:
        """Command 0x35: Read Motor Real-Time Speed
        Read Current Open-Loop Speed.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, speed: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            speed (int in [-65535, 65535]): (Signed) Open-Loop Mode Real-Time Speed in RPM. Will be 0 if error."""
        payload = bytearray([0x35])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0)
        if recv_data[0] == 0x35 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            sign = +1 if recv_data[1] == 0x00 else -1 # get sign
            speed = sign*int.from_bytes(recv_data[2:4], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, speed)
        return (ZDTReturnCode.NONE, 0)
        
    def cmd_read_current_position(self) -> tuple[ZDTReturnCode, int]:
        """Command 0x36: Read Motor Open-Loop current Position
        Read Motor Real-Time Position.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, position: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            position (int in [-4294967295, 4294967295]): (Signed) Current position (0-65535 being a full turn). Will be 0 if error.
            
            Motor Real-Time Position Angle = -0x00010000 = (−65536×360)/65536(−65536×360)/65536 = -360.0°"""
        payload = bytearray([0x36])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0)
        if recv_data[0] == 0x36 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            sign = +1 if recv_data[1] == 0x00 else -1 # get sign
            position = sign*int.from_bytes(recv_data[2:6], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, position)
        return (ZDTReturnCode.NONE, 0)

    def cmd_read_current_position_error(self) -> tuple[ZDTReturnCode, int]:
        """Command 0x37: Read Current Motor Position Error
        Read Current Motor Position Error.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, position_error: int]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            position_error (int in [-4294967295, 4294967295]): (Signed) Current position error (0-32767=0x7FFF being a full turn). Will be 0 if error.
            
            Motor Real-Time Position Error Angle = -0x00000008 = (−8×360)/65536(−8×360)/65536 = -0.0439453125°"""
        payload = bytearray([0x37])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, 0)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, 0)
        if recv_data[0] == 0x37 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            sign = +1 if recv_data[1] == 0x00 else -1 # get sign
            position_error = sign*int.from_bytes(recv_data[2:6], byteorder='big', signed=False)
            return (ZDTReturnCode.OK, position_error)
        return (ZDTReturnCode.NONE, 0)
    
    def cmd_read_motor_status_flags(self) -> tuple[ZDTReturnCode, ZDTMotorStatusFlags | None]:
        """Command 0x3A: Read Motor Status Flag
        Read Motor Status Flag.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, motor_status_flags: ZDTMotorStatusFlags]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            motor_status_flags (ZDTMotorStatusFlags), will be None if error:
                - enabled (bool): If Motor Enable Status Flag is True.
                - in_pos (bool): If Motor In-Position Flag is True.
                - stalled (bool): If Motor Stall Flag is True.
                - stall_protection (bool): If Motor Stall Protection Flag is True."""
        payload = bytearray([0x3A])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        if recv_data[0] == 0x3A and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            flag_byte = recv_data[1]
            enabled = bool(flag_byte & 0x01)
            in_pos = bool(flag_byte & 0x02)
            stalled = bool(flag_byte & 0x04)
            stall_protection = bool(flag_byte & 0x08)
            ret = ZDTMotorStatusFlags(enabled, in_pos, stalled, stall_protection)
            return (ZDTReturnCode.OK, ret)
        return (ZDTReturnCode.NONE, None)

    def cmd_read_homing_status_flags(self) -> tuple[ZDTReturnCode, ZDTHomingStatusFlags | None]:
        """Command 0x3B: Read Homing Status Flag
        Read Homing Status Flag.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, homing_status_flags: ZDTHomingStatusFlags]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            homing_status_flags (ZDTHomingStatusFlags), will be None if error:
                - enc_ready (bool): If Encoder Ready Status Flag is True.
                - CAL_table_ready (bool): If Calibration Table Ready Status Flag is True.
                - homing_in_progress (bool): If Homing in Progress Flag is True.
                - homing_failure (bool): If Homing Failure Flag is True."""
        payload = bytearray([0x3B])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        if recv_data[0] == 0x3B and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            flag_byte = recv_data[1]
            enc_ready = bool(flag_byte & 0x01)
            CAL_table_ready = bool(flag_byte & 0x02)
            homing_in_progress = bool(flag_byte & 0x04)
            homing_failure = bool(flag_byte & 0x08)
            ret = ZDTHomingStatusFlags(enc_ready, CAL_table_ready, homing_in_progress, homing_failure)
            return (ZDTReturnCode.OK, ret)
        return (ZDTReturnCode.NONE, None)
    
    def cmd_read_drive_config_parameters(self) -> tuple[ZDTReturnCode, ZDTDriveConfigurationParameters | None]:
        """Command 0x42: Read Drive Configuration Parameters
        Read Drive Configuration Parameters.

        Note: this assumes the return parameters come with the same format as the example given in the ZDT datasheet.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, params: ZDTDriveConfigurationParameters]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            params (ZDTDriveConfigurationParameters): dataclass embedding the returned parameters. Will be None if error.
            
        Parameter descriptions: #TODO: TODO & copy these descriptions to ZDTDriveConfigurationParameters docstring + comment 0x48
            received_bytes (int in [0,255]): Number of bytes in the command 
            num_config_params (int in [0,255]): Number of configuration parameters in the command 
            mot_type (int in [0,255]): Stepmotor type. Examples: 50 is 0.9°, 25 is 1.8°
            pulse_port_control_mode (int in [0,255]): 0 is PUL_OFF, 1 is PUL_OPEN, 2 is PUL_FOC, 3 is ESI_RCO.
            communication_port_multiplexing_mode (int in [0,255]): 0 is RxTx_OFF, 1 is ESI_ALO, 2 is UART_FUN, 3 is CAN1_MAP.
            en_pin_active_level (int in [0,255]): 0 is L, 1 is H, 2 is Hold (Always active)
            dir_pin_effective_direction (int in [0,255]): 0 is Clockwise, 1 is CounterClockwise
            microstep (int in [0,255]): 0 is 256 microsteps, else number of microsteps
            microstep_interpolation (bool): True is Enabled, False is Disabled
            auto_screen_off (bool): True is Enabled, False is Disabled
            open_loop_mode_operating_current (int in [0,65535]): in mA
            closed_loop_mode_maximum_stall_current (int in [0,65535]): in mA
            closed_loop_mode_maximum_output_voltage (int in [0,65535]): in mA
            serial_baud_rate (int in [0,255]): screen option order TODO: 0x00: 9600, 0x05: 115200
            CAN_communication_rate (int in [0,255]): screen option order TODO: 0x00: 10000, 0x07: 500000
            ID_address (int in [0,255]): direct info, for Serial/RS232/RS485/CAN common
            communication_checksum_method (int in [0,255]): 0: 0x6B, 1: XOR, 2: CRC-8, 3: Modbus # TODO verify
            control_command_response (int in [0,255]): TODO: find in menu and match. 1:  Control command list only returns confirmation of received command
            stall_protection (bool): False is Disabled, True is Enabled # TODO verify
            stall_protection_speed_threshold (int in [0,65535]): in RPM # TODO verify
            stall_protection_current_threshold (int in [0,65535]): in mA  # TODO verify
            stall_protection_detection_time_threshold (int in [0,65535]): in ms  # TODO verify
            position_arrival_window (int in [0,65535]): in 0.1° (0xA being 1°) # TODO verify
        
        Note 1: Stall Detection Criteria: 
            Motor actual speed < set stall detection speed threshold + motor actual phase current > 
            set stall detection phase current threshold + duration > set stall detection time threshold
        
        Note 2: When the difference between the target position (the position angle you sent) and the sensor's
        actual position (the position angle read by the sensor) is less than 0.1°, it is considered that the
        motor has reached the set position.
        The position arrival flag will be set. If the control command response is set to return the
        in-position command, the in-position command will be returned after reaching the position:
        Address + FD + 9F + 6B"""
        payload = bytearray([0x42, 0x6C])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data1 = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout) # First payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data1[0] == 0x00 and recv_data1[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data2 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 1, self.recv_timeout) # Second payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data2[0] == 0x00 and recv_data2[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data3 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 2, self.recv_timeout) # Third payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data3[0] == 0x00 and recv_data3[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data4 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 3, self.recv_timeout) # Fourth payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data4[0] == 0x00 and recv_data4[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data5 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 4, self.recv_timeout) # Fifth payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data5[0] == 0x00 and recv_data5[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)

        recv_data = ZDTEmmV50Handler.combined_payloads(recv_data1, recv_data2, recv_data3, recv_data4, recv_data5)
        if recv_data[0] == 0x42 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok ?
            arr_len_read = recv_data[1]
            if len(recv_data) + 1 != arr_len_read: # verify number of bytes
                return (ZDTReturnCode.ERROR, None)
            out_params = ZDTDriveConfigurationParameters( #Ok !
                received_bytes=recv_data[1],
                num_config_params=recv_data[2],
                mot_type=recv_data[3],
                pulse_port_control_mode= recv_data[4],
                communication_port_multiplexing_mode= recv_data[5],
                en_pin_active_level = recv_data[6],
                dir_pin_effective_direction = recv_data[7],
                microstep = recv_data[8],
                microstep_interpolation = bool(recv_data[9]),
                auto_screen_off = bool(recv_data[10]),
                open_loop_mode_operating_current = int.from_bytes(recv_data[11:13], byteorder='big', signed=False),
                closed_loop_mode_maximum_stall_current = int.from_bytes(recv_data[13:15], byteorder='big', signed=False),
                closed_loop_mode_maximum_output_voltage = int.from_bytes(recv_data[15:17], byteorder='big', signed=False),
                serial_baud_rate = recv_data[17],
                CAN_communication_rate = recv_data[18],
                ID_address = recv_data[19],
                communication_checksum_method = recv_data[20],
                control_command_response = recv_data[21],
                stall_protection = bool(recv_data[22]),
                stall_protection_speed_threshold = int.from_bytes(recv_data[23:25], byteorder='big', signed=False),
                stall_protection_current_threshold = int.from_bytes(recv_data[25:27], byteorder='big', signed=False),
                stall_protection_detection_time_threshold = int.from_bytes(recv_data[27:29], byteorder='big', signed=False),
                position_arrival_window = int.from_bytes(recv_data[29:31], byteorder='big', signed=False)
            )
            return (ZDTReturnCode.OK, out_params)
        return (ZDTReturnCode.NONE, None)
    
    def cmd_read_system_status_parameters(self) -> tuple[ZDTReturnCode, ZDTSystemStatusParameters | None]:
        """Command 0x43: Read System Status Parameters
        Read System Status Parameters.
        
        Args: None

        Returns tuple[return_code: ZDTReturnCode, system_status_params: ZDTSystemStatusParameters]:
            return_code (ZDTReturnCode):
                - ZDTReturnCode.OK if success
                - ZDTReturnCode.ERROR if error code 00 EE xx
                - ZDTReturnCode.NONE if no return received.
            system_status_params (ZDTSystemStatusParameters): dataclass embedding the returned parameters. Will be None if error.
        
        
        Parameter descriptions:
            received_bytes (int in [0,255]): Number of bytes in the command 
            num_config_params (int in [0,255]): Number of configuration parameters in the command
            bus_voltage (int in [0,65535]): In mV.
            bus_phase_current (int in [0,65535]): In mA.
            calibrated_encoder_value (int in [0,65535]): 0-65535 being a full rotation.
            motor_target_position (int in [-4294967295,4294967295]): 0-65535 being a full rotation.
            motor_real_time_speed (int in [-65535,65535]): In RPM.
            motor_real_time_position (int in [-4294967295,4294967295]): 0-65535 being a full rotation.
            motor_position_error (int in [-4294967295,4294967295]): 0-32767=0x7FFF being a full turn.
            flag_encoder_ready_status (bool): Whether encoder ready.
            flag_calibration_ready_status (bool): Whether calibration ready
            flag_homing_in_progress (bool): Whether Homing in progress.
            flag_homing_failure (bool): Whether Homing failure.!
            flag_enable_motor_status (bool): Whether motor enabled.
            flag_motor_in_position_flag (bool): Whether motor is in position.
            flag_motor_stall (bool): Whether motor is in stall mode.
            flag_stall_protection (bool): Whether motor is stall protected."""
        payload = bytearray([0x43, 0x7A])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data1 = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout) # First payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data1[0] == 0x00 and recv_data1[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data2 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 1, self.recv_timeout) # Second payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data2[0] == 0x00 and recv_data2[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data3 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 2, self.recv_timeout) # Third payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data3[0] == 0x00 and recv_data3[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data4 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 3, self.recv_timeout) # Fourth payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data4[0] == 0x00 and recv_data4[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)
        can_id, recv_data5 = self.can_interface.receive_cmd_from((self.mot_id<<8) + 4, self.recv_timeout) # Fifth payload
        if not can_id: # Timeout
            return (ZDTReturnCode.NONE, None)
        if recv_data5[0] == 0x00 and recv_data5[1] == 0xEE: # manual check if error
            return (ZDTReturnCode.ERROR, None)

        recv_data = ZDTEmmV50Handler.combined_payloads(recv_data1, recv_data2, recv_data3, recv_data4, recv_data5)
        print(f"{len(recv_data)=}")
        if recv_data[0] == 0x43 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # Ok
            if len(recv_data) + 1 != recv_data[1]: # verify number of bytes
                return (ZDTReturnCode.ERROR, None)
            # Ok !
            mot_target_pos_sign = -1 if recv_data[9] == 0x01 else +1
            mot_target_pos = mot_target_pos_sign*int.from_bytes(recv_data[10:14], byteorder='big', signed=False)
            mot_rt_speed_sign = -1 if recv_data[14] == 0x01 else +1
            mot_rt_speed = mot_rt_speed_sign*int.from_bytes(recv_data[15:17], byteorder='big', signed=False)
            mot_rt_pos_sign = -1 if recv_data[17] == 0x01 else +1
            mot_rt_pos = mot_rt_pos_sign*int.from_bytes(recv_data[18:22], byteorder='big', signed=False)
            mot_pos_error_sign = -1 if recv_data[22] == 0x01 else +1
            mot_pos_error = mot_pos_error_sign*int.from_bytes(recv_data[23:27], byteorder='big', signed=False)
            ready_status_flag_byte = recv_data[28]
            motor_status_flag_byte = recv_data[29]

            out_params = ZDTSystemStatusParameters(
                received_bytes=recv_data[1],
                num_config_params=recv_data[2],
                bus_voltage=int.from_bytes(recv_data[3:5], byteorder='big', signed=False),
                bus_phase_current=int.from_bytes(recv_data[5:7], byteorder='big', signed=False),
                calibrated_encoder_value=int.from_bytes(recv_data[7:9], byteorder='big', signed=False),
                motor_target_position=mot_target_pos,
                motor_real_time_speed=mot_rt_speed,
                motor_real_time_position=mot_rt_pos,
                motor_position_error=mot_pos_error,
                flag_encoder_ready_status=bool(ready_status_flag_byte & 0x01),
                flag_calibration_ready_status=bool(ready_status_flag_byte & 0x02),
                flag_homing_in_progress=bool(ready_status_flag_byte & 0x04),
                flag_homing_failure=bool(ready_status_flag_byte & 0x08),
                flag_enable_motor_status=bool(motor_status_flag_byte & 0x01),
                flag_motor_in_position_flag=bool(motor_status_flag_byte & 0x02),
                flag_motor_stall=bool(motor_status_flag_byte & 0x04),
                flag_stall_protection=bool(motor_status_flag_byte & 0x08),
            )
            return (ZDTReturnCode.OK, out_params)
        return (ZDTReturnCode.NONE, None)

    def cmd_edit_open_loop_operating_current(self, do_store: bool, ol_mode_current: int) -> ZDTReturnCode:
        """Command 0x44: Edit Open-Loop Operating Current
        Modify Open-Loop Mode Operating Current
        
        Args:
            do_store (bool): If False, parameter is not saved (lost on power off), if True: parameter is saved
            ol_mode_current (int in [0, 65535]): current in mA.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""
        assert 0 <= ol_mode_current <= 65535, f"ol_mode_current {ol_mode_current} is out of range [0, 65535]"
        
        storage_flag = 0x01 if do_store else 0x00
        payload = bytearray([0x44, 0x33, storage_flag])
        payload.extend(ol_mode_current.to_bytes(2, "big", signed=False))
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x44 and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def cmd_switch_open_loop_closed_loop(self, do_store: bool, new_mode: Literal["OpenLoop", "CloseLoop"]) -> ZDTReturnCode:
        """Command 0x46: Switch Open-Loop↔Closed-Loop modes
        Switch Open-Loop/Closed-Loop Mode (P_Pul Menu Option).
        
        Args: 
            do_store (bool): If False, parameter is not saved (lost on power off), if True: parameter is saved
            new_mode (Literal["OpenLoop", "CloseLoop"]): new mode to set (P_Pul Menu Option)

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""
        new_mode_byte: int
        match new_mode:
            case "OpenLoop":
                new_mode_byte = 0x01
            case "CloseLoop":
                new_mode_byte = 0x02
            case _:
                raise AssertionError(f"Mode {new_mode} is invalid !")
        storage_flag = 0x01 if do_store else 0x00
            
        payload = bytearray([0x46, 0x69, storage_flag, new_mode_byte])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x46 and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
    
    def cmd_edit_drive_config_parameters(self, do_store: bool, drive_config_params: ZDTDriveConfigurationParameters) -> ZDTReturnCode:
        """Command 0x48: Edit Drive Config Parameters
        Modify Drive Configuration Parameters.
        
        Args: 
            do_store (bool): If False, parameters are not saved (lost on power off), if True: parameters are saved
            drive_config_params (ZDTDriveConfigurationParameters): new parameters to use.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received.
        
        # TODO: describe params"""
        assert 0 <= drive_config_params.mot_type <= 255, f"{drive_config_params.mot_type=} is out of range [0, 255]"
        assert 0 <= drive_config_params.pulse_port_control_mode <= 255, f"{drive_config_params.pulse_port_control_mode=} is out of range [0, 255]"
        assert 0 <= drive_config_params.communication_port_multiplexing_mode <= 255, f"{drive_config_params.communication_port_multiplexing_mode=} is out of range [0, 255]"
        assert 0 <= drive_config_params.en_pin_active_level <= 255, f"{drive_config_params.en_pin_active_level=} is out of range [0, 255]"
        assert 0 <= drive_config_params.dir_pin_effective_direction <= 255, f"{drive_config_params.dir_pin_effective_direction=} is out of range [0, 255]"
        assert 0 <= drive_config_params.microstep <= 255, f"{drive_config_params.microstep=} is out of range [0, 255]"
        assert 0 <= drive_config_params.open_loop_mode_operating_current <= 65535, f"{drive_config_params.open_loop_mode_operating_current=} is out of range [0, 65535]"
        assert 0 <= drive_config_params.closed_loop_mode_maximum_stall_current <= 65535, f"{drive_config_params.closed_loop_mode_maximum_stall_current=} is out of range [0, 65535]"
        assert 0 <= drive_config_params.closed_loop_mode_maximum_output_voltage <= 65535, f"{drive_config_params.closed_loop_mode_maximum_output_voltage=} is out of range [0, 65535]"
        assert 0 <= drive_config_params.serial_baud_rate <= 255, f"{drive_config_params.serial_baud_rate=} is out of range [0, 255]"
        assert 0 <= drive_config_params.CAN_communication_rate <= 255, f"{drive_config_params.CAN_communication_rate=} is out of range [0, 255]"
        assert 0 <= drive_config_params.communication_checksum_method <= 255, f"{drive_config_params.communication_checksum_method=} is out of range [0, 255]"
        assert 0 <= drive_config_params.control_command_response <= 255, f"{drive_config_params.control_command_response=} is out of range [0, 255]"
        assert 0 <= drive_config_params.stall_protection_speed_threshold <= 65535, f"{drive_config_params.stall_protection_speed_threshold=} is out of range [0, 65535]"
        assert 0 <= drive_config_params.stall_protection_current_threshold <= 65535, f"{drive_config_params.stall_protection_current_threshold=} is out of range [0, 65535]"
        assert 0 <= drive_config_params.stall_protection_detection_time_threshold <= 65535, f"{drive_config_params.stall_protection_detection_time_threshold=} is out of range [0, 65535]"
        assert 0 <= drive_config_params.position_arrival_window <= 65535, f"{drive_config_params.position_arrival_window=} is out of range [0, 65535]"

        storage_flag = 0x01 if do_store else 0x00
        microstep_enabled_byte = 0x01 if drive_config_params.microstep_interpolation else 0x00
        stall_protection_enabled_byte = 0x01 if drive_config_params.stall_protection else 0x00
        auto_screen_off_enabled_byte = 0x01 if drive_config_params.stall_protection else 0x00
        payload1 = bytearray([0x48, 0xD1, storage_flag])
        payload1.extend(bytearray([drive_config_params.mot_type]))
        payload1.extend(bytearray([drive_config_params.pulse_port_control_mode]))
        payload1.extend(bytearray([drive_config_params.communication_port_multiplexing_mode]))
        payload1.extend(bytearray([drive_config_params.en_pin_active_level]))
        payload1.extend(bytearray([drive_config_params.dir_pin_effective_direction]))
        payload2 = bytearray([0x48])
        payload2.extend(bytearray([drive_config_params.microstep]))
        payload2.extend(bytearray([microstep_enabled_byte]))
        payload2.extend(bytearray([auto_screen_off_enabled_byte]))
        payload2.extend(drive_config_params.open_loop_mode_operating_current.to_bytes(2, "big", signed=False))
        payload2.extend(drive_config_params.closed_loop_mode_maximum_stall_current.to_bytes(2, "big", signed=False))
        payload3 = bytearray([0x48])
        payload3.extend(drive_config_params.closed_loop_mode_maximum_output_voltage.to_bytes(2, "big", signed=False))
        payload3.extend(bytearray([drive_config_params.serial_baud_rate]))
        payload3.extend(bytearray([drive_config_params.CAN_communication_rate]))
        payload3.extend(bytearray([0x01])) # Deprecated param
        payload3.extend(bytearray([drive_config_params.communication_checksum_method]))
        payload3.extend(bytearray([drive_config_params.control_command_response]))
        payload4 = bytearray([0x48])
        payload4.extend(bytearray([stall_protection_enabled_byte]))
        payload4.extend(drive_config_params.stall_protection_speed_threshold.to_bytes(2, "big", signed=False))
        payload4.extend(drive_config_params.stall_protection_current_threshold.to_bytes(2, "big", signed=False))
        payload4.extend(drive_config_params.stall_protection_detection_time_threshold.to_bytes(2, "big", signed=False))
        payload5 = bytearray([0x48])
        payload5.extend(drive_config_params.position_arrival_window.to_bytes(2, "big", signed=False))
        payload5.extend(self.checksum.get(ZDTEmmV50Handler.combined_payloads(payload1, payload2, payload3, payload4, payload5))) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload1)
        self.can_interface.send_cmd((self.mot_id<<8) + 1, payload2)
        self.can_interface.send_cmd((self.mot_id<<8) + 2, payload3)
        self.can_interface.send_cmd((self.mot_id<<8) + 3, payload4)
        self.can_interface.send_cmd((self.mot_id<<8) + 4, payload5)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x48 and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def cmd_edit_position_PID_parameters(self, do_store: bool, kp: int, ki: int, kd: int) -> ZDTReturnCode:
        """Command 0x4A: Edit Pos Loop PID Parameters
        Modify the position loop PID parameters.
        
        Args: 
            do_store (bool): If False, parameters are not saved (lost on power off), if True: parameters are saved
            kp (int in [0, 4294967295]): Kp value.
            ki (int in [0, 4294967295]): Ki value.
            kd (int in [0, 4294967295]): Kd value.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""
        assert 0 <= kp <= 4294967295, f"{kp=} is out of range [0, 4294967295]"
        assert 0 <= ki <= 4294967295, f"{ki=} is out of range [0, 4294967295]"
        assert 0 <= kd <= 4294967295, f"{kd=} is out of range [0, 4294967295]"

        storage_flag = 0x01 if do_store else 0x00
        payload = bytearray([0x4A, 0xC3, storage_flag])
        payload.extend(kp.to_bytes(4, "big", signed=False))
        payload.extend(ki.to_bytes(4, "big", signed=False))
        payload.extend(kd.to_bytes(4, "big", signed=False))

        payload1 = bytearray(payload[:8])
        payload2 = bytearray([0x4A]) + bytearray(payload[8:15])
        payload3 = bytearray([0x4A]) + bytearray(payload[15:])
        payload3.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload1)
        self.can_interface.send_cmd((self.mot_id<<8) + 1, payload2)
        self.can_interface.send_cmd((self.mot_id<<8) + 2, payload3)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x4A and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
    
    def cmd_edit_homing_parameters(self, do_store: bool, homing_params: ZDTHomingParameters) -> ZDTReturnCode:
        """Command 0x4C: Set Homing Parameters
        Modify Origin Homing Parameters.
        
        Args: 
            do_store (bool): If False, parameters are not saved (lost on power off), if True: parameters are saved
            homing_params (ZDTHomingParameters): Homing parameters to set.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received.
        
        homing_params (ZDTHomingParameters), Will be None if error:
            - mode (Literal["Nearest", "Directional Nearest", "Collision", "Limit Switch"]): see below for modes description.
            - direction (Literal["Clockwise", "CounterClockwise"]): direction "Clockwise" or "CounterClockwise".
            - speed (int in [0, 65535]): homing speed in RPM.
            - timeout (int in [0, 4294967295]): homing timeout in ms.
            - detection_speed (int in [0, 65535]): Limitless Collision Homing Detection Speed in RPM.
            - detection_current (int in [0, 65535]): Limitless Collision Homing Detection Current in mA.
            - detection_time (int in [0, 65535]): Limitless Collision Homing Detection Time in ms.
            - enable_pwr_on_auto_trigger (bool): Enable Power-On Auto-Trigger Homing Function. False is not enabled, True is enabled.
        
        Homing modes:
            - "Nearest": Go to homing 0-position the nearest way (can go clockwise or counterclockwise). Will rotate 1 turn max.
            - "Directional Nearest": Rotates in given direction up to reaching 0-position (nearest but one way). Will rotate 1 turn max.
            - "Collision": The motor will rotate until it detects a collision (see criteria below). Can rotate without limits.
            - "Limit Switch": The motor will rotate until the external limit switch is triggered. Please refer to ZDT's documentation for the wiring. Can rotate without limits."""
        match homing_params.mode:
            case "Nearest":
                homing_mode_byte = 0x00
            case "Directional Nearest":
                homing_mode_byte = 0x01
            case "Collision":
                homing_mode_byte = 0x02
            case "Limit Switch":
                homing_mode_byte = 0x03
            case _:
                raise AssertionError(f"{homing_params.mode=} is not supported !")
        homing_direction_byte: int
        match homing_params.direction:
            case "Clockwise":
                homing_direction_byte = 0x00
            case "CounterClockwise":
                homing_direction_byte = 0x01
            case _:
                raise AssertionError(f'{homing_params.direction=} should have been either "Clockwise" or "CounterClockwise"')
        assert 0 <= homing_params.speed <= 65535, f"{homing_params.speed=} is out of range [0, 65535]"
        assert 0 <= homing_params.timeout <= 4294967295, f"{homing_params.mode=} is out of range [0, 4294967295]"
        assert 0 <= homing_params.detection_speed <= 65535, f"{homing_params.detection_speed=} is out of range [0, 65535]"
        assert 0 <= homing_params.detection_current <= 65535, f"{homing_params.detection_current=} is out of range [0, 65535]"
        assert 0 <= homing_params.detection_time <= 65535, f"{homing_params.detection_time=} is out of range [0, 65535]"

        storage_flag = 0x01 if do_store else 0x00
        enable_pwr_on_auto_trigger_byte = 0x01 if homing_params.enable_pwr_on_auto_trigger else 0x00
        payload = bytearray([0x4C, 0xAE, storage_flag])
        payload.extend(bytearray([homing_mode_byte]))
        payload.extend(bytearray([homing_direction_byte]))
        payload.extend(homing_params.speed.to_bytes(2, "big", signed=False))
        payload.extend(homing_params.timeout.to_bytes(4, "big", signed=False))
        payload.extend(homing_params.detection_speed.to_bytes(2, "big", signed=False))
        payload.extend(homing_params.detection_current.to_bytes(2, "big", signed=False))
        payload.extend(homing_params.detection_time.to_bytes(2, "big", signed=False))
        payload.extend(bytearray([enable_pwr_on_auto_trigger_byte]))

        payload1 = bytearray(payload[:8])
        payload2 = bytearray([0x4C]) + bytearray(payload[8:15])
        payload3 = bytearray([0x4C]) + bytearray(payload[15:])
        payload3.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload1)
        self.can_interface.send_cmd((self.mot_id<<8) + 1, payload2)
        self.can_interface.send_cmd((self.mot_id<<8) + 2, payload3)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x4C and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def cmd_allow_divide10_on_com_speed_commands(self, do_store: bool, do_divide10: bool) -> ZDTReturnCode:
        """Command 0x4F: Allow /10 on Communication Control Speed commands
        Modify Communication Control Input Speed to Reduce by 10x (S_Vel_IS Menu Option).
        
        Args: 
            do_store (bool): If False, parameters are not saved (lost on power off), if True: parameters are saved
            do_divide10 (boom): If True activates the 10x speed division, if False disables it 

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""

        storage_flag = 0x01 if do_store else 0x00
        do_divide10_flag = 0x01 if do_divide10 else 0x00
        payload = bytearray([0x4F, 0x71, storage_flag, do_divide10_flag])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x4F and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
    
    def cmd_edit_microstep(self, do_store: bool, microstep_value: int) -> ZDTReturnCode:
        """Command 0x84: Edit Microstep Value
        Modify Any Microstep
        
        Args: 
            do_store (bool): If False, parameters are not saved (lost on power off), if True: parameters are saved
            microstep_value (int in [0,256]): New microstep value. If 0 or 256, the effective value is 256.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""
        assert 0 <= microstep_value <= 256, f"{microstep_value=} is out of range [0, 256]"
        storage_flag = 0x01 if do_store else 0x00
        microstep_value_byte = 0 if microstep_value == 256 else microstep_value # allow 256 as input

        payload = bytearray([0x84, 0x8A, storage_flag, microstep_value_byte])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x84 and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def cmd_set_0_position_homing(self, do_store: bool) -> ZDTReturnCode:
        """Command 0x93: Set 0 Position Homing
        Set the Zero Position for Single-Turn Homing at current position.
        
        Args: 
            do_store (bool): If False, parameters are not saved (lost on power off), if True: parameters are saved

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""
        storage_flag = 0x01 if do_store else 0x00

        payload = bytearray([0x93, 0x88, storage_flag])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x93 and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def cmd_trigger_homing(self, homing_mode: Literal["Nearest", "Directional Nearest", "Collision", "Limit Switch"], m_m_sync_flag: bool = False) -> ZDTReturnCode:
        """Command 0x9A: Trigger Homing
        Trigger Homing.
        
        Args: 
            homing_mode (Literal["Nearest", "Directional Nearest", "Collision", "Limit Switch"]): homing mode to trigger. See below for mode descriptions.
            m_m_sync_flag (bool, optional): Multi-Machine Synchronization Flag. If True, the motors will wait for 0xFF Multi-Machine Synchronized Motion command to move. Defaults to False.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.CONDITIONS_NOT_MET if conditions not met
        - ZDTReturnCode.NONE if no return received.
        
        Homing modes:
            - "Nearest": Go to homing 0-position the nearest way (can go clockwise or counterclockwise). Will rotate 1 turn max.
            - "Directional Nearest": Rotates in given direction up to reaching 0-position (nearest but one way). Will rotate 1 turn max.
            - "Collision": The motor will rotate until it detects a collision (see criteria below). Can rotate without limits.
            - "Limit Switch": The motor will rotate until the external limit switch is triggered. Please refer to ZDT's documentation for the wiring. Can rotate without limits.
        
        Detection Criteria (for "Collision" mode):
            Motor speed < collision homing detection speed + motor phase current > collision 
            homing detection current + duration > collision homing detection time
            (Note: Detection thresholds can be modified on the host computer or screen)
            
        Notes homing parameters (position, speed, ...) can be set via commands such as 0x4C Set Homing Parameters"""
        homing_mode_byte: int
        match homing_mode:
            case "Nearest":
                homing_mode_byte = 0x00
            case "Directional Nearest":
                homing_mode_byte = 0x01
            case "Collision":
                homing_mode_byte = 0x02
            case "Limit Switch":
                homing_mode_byte = 0x03
            case _:
                raise AssertionError(f"{homing_mode=} is not supported !")

        m_m_sync_flag_byte = 0x01 if m_m_sync_flag else 0x00
        payload = bytearray([0x9A, homing_mode_byte, m_m_sync_flag_byte])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x9A and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0x9A and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
    
    def cmd_interrupt_homing(self) -> ZDTReturnCode:
        """Command 0x9C: Interrupt Homing.
        Force Interrupt and Exit Homing Operation.
        
        Args: None

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.CONDITIONS_NOT_MET if conditions not met
        - ZDTReturnCode.NONE if no return received."""

        payload = bytearray([0x9C, 0x48])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x9C and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0x9C and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
    

    def cmd_edit_ID_address(self, do_store: bool, target_id: int) -> ZDTReturnCode:
        """Command 0xAE: Edit ID Address
        Edit ID Address of target device. Note: seems to always return ZDTReturnCode.NONE

        Args:
            do_store (bool): If False, parameters are not saved (lost on power off), if True: parameters are saved
            target_id (int in [0,255]): New ID to set.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""
        assert 0 <= target_id <= 255, f"{target_id=} is out of range [0, 255]"
        storage_flag = 0x01 if do_store else 0x00

        payload = bytearray([0xAE, 0x4B, storage_flag, target_id])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0x4B and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
    
    def cmd_motor_enable_control(self, enable: bool = True, m_m_sync_flag: bool = False) -> ZDTReturnCode:
        """Command 0xF3: Motor Enable Control.
        Enables control of the motor through CAN.

        Args:
            enable (bool, optional): set to True to enable motor, False to disable motor. Defaults to True
            m_m_sync_flag (bool, optional): Multi-Machine Synchronization Flag. If True, the motors will wait for 0xFF Multi-Machine Synchronized Motion command to move. Defaults to False.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""
        enable_status = 0x01 if enable else 0x00
        m_m_sync_flag_byte = 0x01 if m_m_sync_flag else 0x00
        payload = bytearray([0xF3, 0xAB, enable_status, m_m_sync_flag_byte])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0xF3 and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data):
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
        
    def cmd_speed_mode_control(self, speed: int, acceleration_level: int, m_m_sync_flag: bool = False) -> ZDTReturnCode:
        """Command 0xF6: Speed Mode Control
        Speed Mode Control.

        Args:
            speed (int in [-65535, 65535]): speed in RPM (the absolute value will be used).
            acceleration_level (int in [0,255]): acceleration level, read more below
            m_m_sync_flag (bool, optional): Multi-Machine Synchronization Flag. If True, the motors will wait for 0xFF Multi-Machine Synchronized Motion command to move. Defaults to False.
        
        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.CONDITIONS_NOT_MET if conditions not met
        - ZDTReturnCode.NONE if no return received.

        About acceleration level (source: ZDT_EmmV5.0Rev1.3 datasheet):
            - if 0, will run at full speed right away
            - if >0, the relationship with acceleration is : t2-t1=(256-acc)*50(μs)t2-t1=(256-acc)-50(μs), Vt2=Vt1+1Vt2=Vt1+1RPM"""
        assert -65535 <= speed <= 65535, f"speed {speed} is out of range [-65535, 65535]"
        assert 0 <= acceleration_level <= 255, f"acceleration_level {acceleration_level} is out of range [0, 255]"
        direction = 0x01 if speed < 0 else 0x00 # direction as sign of pulse count
        speed = abs(speed)
        m_m_sync_flag_byte = 0x01 if m_m_sync_flag else 0x00

        payload = bytearray([0xF6, direction])
        payload.extend(speed.to_bytes(2, "big", signed=False))
        payload.extend(bytearray([acceleration_level]))
        payload.extend(bytearray([m_m_sync_flag_byte]))
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0xF6 and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0xF6 and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
        
    def cmd_auto_run_on_power_on(self, store_clear_flag: int, speed: int, acceleration_level: int, do_en_pin_control: bool) -> ZDTReturnCode:
        """Command 0xF7: Auto-Run on Power-On
        Store a Set of Speed Mode Parameters (Direction, Speed, Acceleration), Auto-Run on Power-On

        Args:
            store_clear_flag (int in [0, 255]): Store/Clear flag. TODO: 0x01 might be store, but is 0x00 clear ?...
            speed (int in [-65535, 65535]): (Signed) Speed in RPM.
            acceleration_level (int in [0,255]): acceleration level, read more below
            do_en_pin_control (bool): If True, the auto-run routine can be controlled to start and stop via the En pin. If False, disables it.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.CONDITIONS_NOT_MET if conditions not met
        - ZDTReturnCode.NONE if no return received.

        About acceleration level (source: ZDT_EmmV5.0Rev1.3 datasheet):
            - if 0, will run at full speed right away
            - if >0, the relationship with acceleration is : t2-t1=(256-acc)*50(μs)t2-t1=(256-acc)-50(μs), Vt2=Vt1+1Vt2=Vt1+1RPM"""
        assert -65535 <= speed <= 65535, f"speed {speed} is out of range [-65535, 65535]"
        assert 0 <= acceleration_level <= 255, f"acceleration_level {acceleration_level} is out of range [0, 255]"
        
        direction = 0x01 if speed < 0 else 0x00 # direction as sign of pulse count
        speed = abs(speed)
        do_en_pin_control_byte = 0x01 if do_en_pin_control else 0x00

        payload = bytearray([0xF7, 0x1C, store_clear_flag, direction])
        payload.extend(speed.to_bytes(2, "big", signed=False))
        payload.extend(bytearray([acceleration_level]))
        payload.extend(bytearray([do_en_pin_control_byte]))

        payload1 = bytearray(payload[:8])
        payload2 = bytearray([0xF7]) + bytearray(payload[8:])
        payload2.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload1)
        self.can_interface.send_cmd((self.mot_id<<8) + 1, payload2)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0xF7 and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0xF7 and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
    
    def cmd_position_mode_control(self, pulse_count: int, speed: int, acceleration_level: int, is_relative: bool = True, m_m_sync_flag: bool = False) -> ZDTReturnCode:
        """Command 0xFD: Position Mode Control
        Send position-mode control command. 

        Args:
            pulse_count (int in [-4294967295, 4294967295]): pulse count for position control. If positive, will go in default (0x00) direction, if negative the other way.
            speed (int in [-65535, 65535]): speed in RPM (the absolute value will be used).
            acceleration_level (int in [0,255]): acceleration level, read more below
            is_relative (bool, optional): Relative position control if True, Absolute position control if False. Defaults to True. 
            m_m_sync_flag (bool, optional): Multi-Machine Synchronization Flag. If True, the motors will wait for 0xFF Multi-Machine Synchronized Motion command to move. Defaults to False.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.CONDITIONS_NOT_MET if conditions not met
        - ZDTReturnCode.NONE if no return received.

        About acceleration level (source: ZDT_EmmV5.0Rev1.3 datasheet):
            - if 0, will run at full speed right away
            - if >0, the relationship with acceleration is : t2-t1=(256-acc)*50(μs)t2-t1=(256-acc)-50(μs), Vt2=Vt1+1Vt2=Vt1+1RPM"""
        assert -4294967295 <= pulse_count <= 4294967295, f"pulse_count {pulse_count} is out of range [-4294967295, 4294967295]"
        assert -65535 <= speed <= 65535, f"speed {speed} is out of range [-65535, 65535]"
        assert 0 <= acceleration_level <= 255, f"acceleration_level {acceleration_level} is out of range [0, 255]"
        direction = 0x01 if pulse_count < 0 else 0x00 # direction as sign of pulse count
        speed, pulse_count = abs(speed), abs(pulse_count)
        relative = 0x00 if is_relative else 0x01
        m_m_sync_flag_byte = 0x01 if m_m_sync_flag else 0x00

        payload = bytearray([0xFD, direction])
        payload.extend(speed.to_bytes(2, "big", signed=False))
        payload.extend(bytearray([acceleration_level]))
        payload.extend(pulse_count.to_bytes(4, "big", signed=False))
        payload.extend(bytearray([relative]))
        payload.extend(bytearray([m_m_sync_flag_byte]))

        payload1 = bytearray(payload[:8])
        payload2 = bytearray([0xFD]) + bytearray(payload[8:])
        payload2.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload1)
        self.can_interface.send_cmd((self.mot_id<<8) + 1, payload2)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0xFD and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0xFD and recv_data[1] in (0x02, 0x9F) and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK, 0x02: received cmd, 0x9F: reached position
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def cmd_immediate_stop(self, m_m_sync_flag: bool = False) -> ZDTReturnCode:
        """Command 0xFE: Immediate Stop
        Stop the motor immediately.
        
        Args:
            m_m_sync_flag (bool, optional): Multi-Machine Synchronization Flag. If True, the motors will wait for 0xFF Multi-Machine Synchronized Motion command to move. Defaults to False.

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.CONDITIONS_NOT_MET if conditions not met
        - ZDTReturnCode.NONE if no return received."""
        m_m_sync_flag_byte = 0x01 if m_m_sync_flag else 0x00
        payload = bytearray([0xFE, 0x98, m_m_sync_flag_byte])
        payload.extend(self.checksum.get(payload)) # append checksum
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd((self.mot_id<<8), payload)

        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), self.recv_timeout)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0xFE and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0xFE and recv_data[1] == 0x02 and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE
    

    def cmd_M_M_synchronized_motion(self, do_broadcast: bool) -> ZDTReturnCode: # TODO: broadcast does not work + change checksum if broadcast
        """Command 0xFF: Multi-Machine Synchronized Motion
        Start Multi-Machine Synchronized Motion. Please refer to the documentation.
        Basically, send control commands (speed, position,...) with multi machine synchronized motion flags to True, the motors will wait for that command to be sent. 
        
        Args: 
            do_broadcast (bool): If True, will broadcast that command (send to ID 0x00) instead of self.mot_id, allowing all the motors to receive the signal to go simultaneously. 

        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.CONDITIONS_NOT_MET if conditions not met
        - ZDTReturnCode.NONE if no return received."""
        payload = bytearray([0xFF, 0x66])
        payload.extend(self.checksum.get(payload)) # append checksum
        
        effective_id = (self.mot_id<<8)
        if do_broadcast:
            effective_id = 0x00 # Force broadcast # ID inserted in payload, checksum appended by interface
        self.can_interface.clear_queues_of(self.mot_id) # Clear related queues
        self.can_interface.send_cmd(effective_id, payload)

        can_id, recv_data = self.can_interface.receive_cmd_from(0x01, self.recv_timeout) # Only ID 1 should answer
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0xFF and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0xFF and recv_data[1] == 0x02 and self._check_checksum(bytearray([effective_id]) + recv_data): # OK, with effective_id
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE

    def receive_reached_message(self, timeout: float | None = 5.0):
        """Wait for "Reached" return message from the device, after 0xFD Position Mode Control cmd
        Wait for "Reached" return message from the device, after 0xFD Position Mode Control cmd.

        Args:
            timeout (float, optional): Maximum time to wait for the "Reached" message. If None, is set to self.timeout. Defaults to 5 seconds
        
        Returns (ZDTReturnCode): 
        - ZDTReturnCode.OK if success
        - ZDTReturnCode.ERROR if error code 00 EE xx
        - ZDTReturnCode.NONE if no return received."""

        timeout_ = timeout if timeout is not None else self.recv_timeout


        can_id, recv_data = self.can_interface.receive_cmd_from((self.mot_id<<8), timeout_)
        if not can_id: # Timeout
            return ZDTReturnCode.NONE
        if recv_data[0] == 0x00 and recv_data[1] == 0xEE: # manual check if error
            return ZDTReturnCode.ERROR
        if recv_data[0] == 0xFD and recv_data[1] == 0xE2: # manual check if conditions not met
            return ZDTReturnCode.CONDITIONS_NOT_MET
        if recv_data[0] == 0xFD and recv_data[1] == 0x9F and self._check_checksum(bytearray([self.mot_id]) + recv_data): # OK, 0x02: received cmd, 0x9F: reached position
            return ZDTReturnCode.OK
        return ZDTReturnCode.NONE