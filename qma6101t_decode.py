from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

from qst_common import (
    DecodedTransaction,
    I2cAssembler,
    SpiAssembler,
    convert_acceleration,
    decode_register_fields,
    load_registers,
    signed_i16,
)


CHIP_ID = 0x00
ACC_DATA_START = 0x01
ACC_DATA_END = 0x06
RANGE = 0x0F
FIFO_DATA = 0x3F


REGISTERS = load_registers("qma6101t_registers.json")

REGISTERS.setdefault(
    CHIP_ID,
    {
        "name": "CHIP_ID",
        "access": "RO",
        "width": 1,
        "description": "QMA6101T chip identity",
        "roles": ["identity"],
        "fields": [
            {
                "name": "chip_id",
                "bits": "7:0",
                "description": "0x0A typically for QMA6101T",
                "roles": ["identity_value"],
            }
        ],
    },
)


class Qma6101tDecoder:
    register_mask = 0x7F

    def __init__(self) -> None:
        self.accel_fs_override: Optional[float] = None
        self.accel_unit = "g"

    def set_scale_override(self, accel_fs_g: Optional[float]) -> None:
        self.accel_fs_override = accel_fs_g

    def set_output_unit(self, accel_unit: str) -> None:
        self.accel_unit = accel_unit

    def decode_spi(self, mosi: Sequence[int], miso: Sequence[int]) -> DecodedTransaction:
        mosi_bytes = [value & 0xFF for value in mosi]
        miso_bytes = [value & 0xFF for value in miso]
        if not mosi_bytes:
            return DecodedTransaction(
                bus="SPI",
                operation="UNKNOWN",
                register=None,
                register_name="NO_COMMAND",
                data=[],
                status="No MOSI command byte",
                raw_mosi=mosi_bytes,
                raw_miso=miso_bytes,
            )
        command = mosi_bytes[0]
        operation = "READ" if command & 0x80 else "WRITE"
        register = command & self.register_mask
        data = miso_bytes[1:] if operation == "READ" else mosi_bytes[1:]
        transaction = self.decode_register("SPI", operation, register, data)
        transaction.raw_mosi = mosi_bytes
        transaction.raw_miso = miso_bytes
        if operation == "READ" and len(miso_bytes) <= 1:
            transaction.status = "Read transaction has no MISO payload"
        return transaction

    def decode_i2c(
        self,
        device_address: int,
        operation: str,
        register: Optional[int],
        data: Sequence[int],
    ) -> DecodedTransaction:
        if register is None:
            return DecodedTransaction(
                bus="I2C",
                operation=operation,
                register=None,
                register_name="UNKNOWN_POINTER",
                data=[value & 0xFF for value in data],
                device_address=device_address,
                status="Read without a known register pointer",
            )
        return self.decode_register(
            "I2C", operation, register & self.register_mask, data, device_address
        )

    def decode_register(
        self,
        bus: str,
        operation: str,
        register: int,
        data: Sequence[int],
        device_address: Optional[int] = None,
    ) -> DecodedTransaction:
        values = [value & 0xFF for value in data]
        definition = REGISTERS.get(register, {})
        register_name = definition.get("name", f"REG_0x{register:02X}")
        status = "OK" if definition else "Register is not present in the QMA6101T map"
        access = definition.get("access", "")
        if operation == "WRITE" and access == "RO":
            status = "Write to read-only register"
        elif operation == "READ" and access == "WO":
            status = "Read from write-only register"

        transaction = DecodedTransaction(
            bus=bus,
            operation=operation,
            register=register,
            register_name=register_name,
            data=values,
            device_address=device_address,
            status=status,
        )
        if register not in (ACC_DATA_START, FIFO_DATA):  # individual data regs or fifo handled separately
            transaction.fields = decode_register_fields(REGISTERS, register, values)
        transaction.derived = self._derive_values(register, operation, values)
        if operation in ("READ", "WRITE"):
            self._observe_configuration(register, values)
        return transaction

    def advance_i2c_pointer(self, pointer: int, count: int) -> int:
        if (pointer & self.register_mask) >= ACC_DATA_START and (pointer & self.register_mask) <= ACC_DATA_END:
            return FIFO_DATA  # stay at FIFO after data read for continuous mode
        return (pointer + count) & self.register_mask

    def _derive_values(
        self, register: int, operation: str, values: Sequence[int]
    ) -> List[str]:
        if register == CHIP_ID and operation == "READ" and values:
            if values[0] == 0x0A:
                return [f"CHIP_ID=0x{values[0]:02X} (QMA6101T)"]
            return [f"CHIP_ID=0x{values[0]:02X}"]
        if register in range(ACC_DATA_START, ACC_DATA_END + 1) and len(values) == 6:
            # Full 6-byte sample (X/Y/Z)
            return self._decode_samples(values, "ACC_DATA")
        if register in range(ACC_DATA_START, ACC_DATA_END + 1) and len(values) >= 2:
            # Individual register read (1 byte)
            return [f"Raw value = 0x{values[0]:02X} (low byte only; read 6 bytes for XYZ vector)"]
        if register == FIFO_DATA and operation == "READ":
            return self._decode_samples(values, "FIFO")
        return []

    def _decode_samples(self, values: Sequence[int], source: str) -> List[str]:
        complete = len(values) // 6
        remainder = len(values) % 6
        if complete == 0:
            return [f"{source} partial sample ({len(values)}/6 bytes)"]
        summary = f"{source} {complete} XYZ sample(s)"
        if remainder:
            summary += f", trailing {remainder} byte(s)"
        # Decode 14-bit signed (low 2 bits discarded, like QMA6100P)
        raw = [signed_i16(values, offset, "little") >> 2 for offset in (0, 2, 4)]
        full_scale = self.accel_fs_override
        if full_scale is None:
            vector = f"Araw14=[{raw[0]}, {raw[1]}, {raw[2]}]"
        else:
            lsb_per_g = 8192.0 / full_scale
            converted = [
                convert_acceleration(axis / lsb_per_g, self.accel_unit) for axis in raw
            ]
            vector = "A=[" + ", ".join(f"{value:.4f}" for value, _ in converted)
            vector += f"] {converted[0][1]}"
        return [summary, vector]

    def _observe_configuration(self, register: int, values: Sequence[int]) -> None:
        for offset, value in enumerate(values):
            if ((register + offset) & self.register_mask) != RANGE:
                continue
            # Simple range mapping for QMA6101T (adjust based on actual)
            self.accel_fs_override = {
                0x1: 2.0,
                0x2: 4.0,
                0x4: 8.0,
                0x8: 16.0,
                0xF: 32.0,
            }.get(value & 0x0F, self.accel_fs_override)


__all__ = ["I2cAssembler", "Qma6101tDecoder", "SpiAssembler"]
