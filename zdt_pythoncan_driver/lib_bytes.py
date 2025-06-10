"""
Defines checksums computing objects
"""
from abc import ABC, abstractmethod
try:
    from typing import override
except ImportError:
    def override(funcobj): # dummy decorator just for annotation
        return funcobj

class ChecksumABC(ABC):
    """Describes one checksum"""
    @staticmethod
    @abstractmethod
    def get(bytes: bytearray) -> bytearray:
        """Compute that checksum """
        raise NotImplementedError()
    def __call__(self, bytes: bytearray):
        """Allows calling class instances."""
        return self.get(bytes)
class CS0x6B(ChecksumABC):
    """Checksum description for 0x6B checksum."""
    @staticmethod
    @override
    def get(bytes: bytearray) -> bytearray:
        return bytearray([0x6B])
class CSXOR(ChecksumABC):
    """Checksum description for XOR checksum."""
    @staticmethod
    @override
    def get(bytes: bytearray) -> bytearray:
        checksum = 0
        for byte in bytes:
            checksum ^= byte
        return bytearray([checksum]) # Should be 1 byte 

class CSCRC8(ChecksumABC):
    """Checksum description for CRC8 checksum. """
    @staticmethod
    @override
    def get(bytes: bytearray) -> bytearray:
        crc = 0x00
        for byte in bytes:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        return bytearray([crc])
class CSModbus(ChecksumABC):
    """Checksum description for Modbus checksum"""
    @staticmethod
    @override
    def get(bytes: bytearray) -> bytearray:
        raise NotImplementedError("Modbus checksum is not implemented")

class Checksums:
    """Enumerate supported checksums."""
    CS0x6B = CS0x6B()
    CSXOR = CSXOR()
    CSCRC8 = CSCRC8()
    CSModbus = CSModbus()