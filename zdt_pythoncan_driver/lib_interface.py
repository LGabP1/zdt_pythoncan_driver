"""
Defines the low level inteface, taking input commands and sending or retrieving corresponding data.
"""

from abc import ABC, abstractmethod
from typing import Optional
from can import Bus, BusABC, Message
from logging import Logger

from .lib_bytes import Checksums, ChecksumABC, override

class CANBusOpenedException(Exception):
    """Bus is already open !"""
    def __init__(self, channel: str, interface: str, *args):
        super().__init__(*args)
        self.channel = channel
        self.interface = interface

    def __str__(self):
        return f"Bus (channel={self.channel}, interface={self.interface}) already opened: {super().__str__()}"

class CANReceiveException(Exception):
    pass

class CANNotOpenException(Exception):
    pass

class CANInterfaceGeneric(ABC):
    """Base interface"""
    
    @abstractmethod 
    def send(self, can_id: int, data: bytearray, checksum: ChecksumABC = Checksums.CS0x6B) -> None:
        """Sends given data with given checksum to given can_id. No return expected.
        
        Args:
            can_id (int): CAN arbitration id, will use extended ids.
            data (bytearray): data to send.
            checksum (Checksum): checksum to use. Defaults to 0x6B checksum (Checksum.CS0x6B)."""
        raise NotImplementedError()
    
    @abstractmethod 
    def recv(self, timeout: float, checksum: ChecksumABC = Checksums.CS0x6B) -> tuple[bytearray | None, bool]:
        """Receive one complete data array and whether the checksum matches.
        
        Args:
            timeout (float): timeout
            checksum (Checksum): checksum to use. Defaults to 0x6B checksum (Checksum.CS0x6B).

        Returns: tuple[bytearray | None, bool] where
            bytearray | None : packet content, or None if no packets received
            bool : True if checksum matches, False otherwise"""
        raise NotImplementedError()
    
    @abstractmethod 
    def open(self) -> None:
        """Open CAN channel."""
        raise NotImplementedError()
    
    @abstractmethod 
    def close(self) -> None:
        """Close CAN channel."""
        raise NotImplementedError()

    @abstractmethod 
    def is_open(self) -> bool:
        """Returns whether the connection with the CAN bus is active."""
        raise NotImplementedError()
    
    def __enter__(self) -> None: # Called when entering context manager
        pass
    
    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        if self.is_open(): # Close connection if still active
            self.close()
    
class CANInterface(CANInterfaceGeneric):
    """CAN Interface implementation."""

    def __init__(self, interface: str, channel: str, bitrate: int, logger: Optional[Logger] = None):
        """CAN Interface implementation.
        
        Args:
            interface (str): interface to use
            channel (str): channel to use
            bitrate (int): bitrate to use
            logger (Logger, optional): optional logger"""
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.logger = logger
        self._bus: None | BusABC = None
        self.logger = logger
    
    @override
    def open(self) -> None:
        """Open CAN channel."""
        if self.logger:
            self.logger.debug(f"Trying to open CAN at channel={self.channel}, interface={self.interface}, bitrate={self.bitrate}...")
        
        if self._bus is None:
            self._bus = Bus(self.channel, self.interface, bitrate=self.bitrate, data_bitrate=self.bitrate, fd=False)
        else:
            raise CANBusOpenedException()
        
        if self.logger:
            self.logger.info(f"Successfully opened CAN at channel={self.channel}, interface={self.interface}, bitrate={self.bitrate}.")


    @override
    def close(self) -> None:
        """Close CAN channel. If not openend will do nothing."""
        if self.logger:
            self.logger.debug(f"Trying to close CAN at channel={self.channel}, interface={self.interface}, bitrate={self.bitrate}...")
        if self._bus is not None:
            self._bus.shutdown()
        if self.logger:
            self.logger.debug(f"Successfully closed CAN at channel={self.channel}, interface={self.interface}, bitrate={self.bitrate}...")

    @override
    def is_open(self) -> bool:
        """Returns whether the connection with the CAN bus is active."""
        # raise NotImplementedError("") # TODO
        return True # TODO

    @override
    def send(self, mot_id: int, data: bytearray, checksum: ChecksumABC = Checksums.CS0x6B, timeout: float = 1.0) -> None:
        """Sends given data with given checksum to given motor id mot_id.
        For big enough packets, will split the command in accordance with ZDT's datasheet.
        
        The checksum and mot_id are handled automatically, thus: 1. do not provide the mot_id in data, 2. do not add the checksum at the end of data.
        
        Args:
            mot_id (int): Motor arbitration id, can_id being mot_id<<8 and will use extended ids.
            data (bytearray): data to send.
            checksum (Checksum): checksum to use. Defaults to 0x6B checksum (Checksum.CS0x6B).
            timeout (float, optional): send timeout for each packet. """
        if self._bus is None:
            raise CANNotOpenException("CAN bus is not openened !")
        count = 0 
        remaining_payload = data.copy()
        checksum_value = checksum.get(remaining_payload) # Get checksul for the whole packet
        command_prefix = remaining_payload[0] # keep track command
        while len(remaining_payload) >= 7: # all except last packet
            current_payload = remaining_payload[:7]
            msg = Message(arbitration_id=(mot_id<<8) + count, data=current_payload, is_extended_id=True, is_fd=False)
            self._bus.send(msg, timeout=timeout)

            remaining_payload = remaining_payload[7:]
            remaining_payload.insert(0, command_prefix) # insert command prefix at the beginning again
            count += 1
            if self.logger:
                self.logger.debug(f"Sent message id:{hex(msg.arbitration_id)}, data:{msg.data.hex(" ")}, dl:{msg.dlc}")
        # Last packet
        current_payload = remaining_payload + bytearray(checksum_value) # append checksum at the end

        msg = Message(arbitration_id=(mot_id<<8) + count, data=current_payload, is_extended_id=True, is_fd=False)
        self._bus.send(msg, timeout=timeout)
        if self.logger:
            self.logger.debug(f"Sent message id:{hex(msg.arbitration_id)}, data:{msg.data.hex(" ")}, dl:{msg.dlc}")

    @override
    def recv(self, timeout: float, checksum = Checksums.CS0x6B) -> tuple[bytearray, int, bool]:
        """Receive one complete data array and whether the checksum matches.
        Will combine received packets into one data array according to ZDT's datasheet.

        Warning: Assumes that only one ZDT device communicates during this functions' execution.
        
        Args:
            timeout (float): timeout
            checksum (Checksum): checksum to use. Defaults to 0x6B checksum (Checksum.CS0x6B).

        Returns tuple[bytearray | None, bool] where :
            bytearray : packet content, or empty if no packets received
            int : can_id of device, None if no packets received
            bool : True if checksum matches, False otherwise. Will be False if no packets received or a timeout is raised."""
        if self._bus is None:
            raise CANNotOpenException("CAN bus is not openened !")
        received_data = bytearray([])
        can_id: int = None
        def _is_last_message(data: bytearray) -> bool: # tells if it is last message
            if len(data) < 8:
                return True
            elif len(data) > 8: 
                raise ValueError(f"Length mismatch for {data}: {len(data)=}")
            else:
                cs_match = received_data[-1] == checksum.get(received_data[:-1])[0]
                return cs_match # if checksum matches, returns True: it is last message (likely)

        while True:
            recv_msg: Message = self._bus.recv(timeout)
            if self.logger:
                if recv_msg:
                    self.logger.debug(f"Received message id:{hex(recv_msg.arbitration_id)}, data:{recv_msg.data.hex(" ")}, dl:{recv_msg.dlc}")
                else:
                    self.logger.debug("Received None")
            if recv_msg:
                if can_id is None:
                    can_id = recv_msg.arbitration_id
                new_data = recv_msg.data
                if len(received_data) > 1: # if not first message:
                    new_data = new_data[1:] # remove first byte (command)
                received_data.extend(new_data)

                if _is_last_message(recv_msg.data):
                    break
            else:
                return (received_data, can_id, False)
        cs_match = received_data[-1] == checksum.get(received_data[:-1])[0]
        return (received_data, can_id>>8, cs_match)
    