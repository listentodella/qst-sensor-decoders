from saleae.analyzers import AnalyzerFrame, ChoicesSetting, HighLevelAnalyzer, NumberSetting

from qmi8660_decode import I2cAssembler as Qmi8660I2cAssembler
from qmi8660_decode import Qmi8660Decoder, SpiAssembler as Qmi8660SpiAssembler
from qmi8658_decode import I2cAssembler as Qmi8658I2cAssembler
from qmi8658_decode import Qmi8658Decoder, SpiAssembler as Qmi8658SpiAssembler
from qma6100p_decode import I2cAssembler as Qma6100pI2cAssembler
from qma6100p_decode import Qma6100pDecoder, SpiAssembler as Qma6100pSpiAssembler
from qma6101t_decode import I2cAssembler as Qma6101tI2cAssembler
from qma6101t_decode import Qma6101tDecoder, SpiAssembler as Qma6101tSpiAssembler


def _scale_value(setting):
    text = str(setting)
    if text == "Auto":
        return None
    try:
        return float(text.split()[0])
    except (TypeError, ValueError):
        return None


def _compatible_unit(settings, name, default):
    value = settings.get(name, default)
    return "m/s²" if value == "m/s^2" else value


class Qmi8660Hla(HighLevelAnalyzer):
    i2c_address = ChoicesSetting(
        label="I2C address", choices=("0x6A or 0x6B", "0x6A", "0x6B", "Any")
    )
    spi_gap_us = NumberSetting(
        label="SPI transaction gap without Enable (us)", min_value=1, max_value=1_000_000
    )
    accel_full_scale = ChoicesSetting(
        label="Accelerometer full scale", choices=("Auto", "4 g", "8 g", "16 g", "32 g")
    )
    accel_unit = ChoicesSetting(label="Accel unit", choices=("g", "mg", "m/s²"))
    gyro_full_scale = ChoicesSetting(
        label="Gyroscope full scale",
        choices=("Auto", "128 dps", "256 dps", "512 dps", "1024 dps", "2048 dps", "4096 dps"),
    )
    gyro_unit = ChoicesSetting(label="Gyro unit", choices=("dps", "rad/s"))
    fifo_layout = ChoicesSetting(
        label="FIFO layout",
        choices=(
            "Auto", "Gyro XYZ + Accel XYZ + Temp", "Gyro XYZ + Accel XYZ",
            "Gyro XYZ + Temp", "Accel XYZ + Temp", "Gyro XYZ", "Accel XYZ",
        ),
    )
    result_types = {
        "qmi8660": {"format": "{{data.Bus}}{{data.Address}} {{data.Op}} {{data.Register}} {{data.Hex}} {{data.Detail}} {{data.Status}}"},
        "qmi8660_sensor": {"format": "{{data.Bus}}{{data.Address}} DATA {{data.Detail}}"},
        "qmi8660_fifo": {"format": "{{data.Bus}} FIFO {{data.Detail}}"},
        "qmi8660_interrupt": {"format": "{{data.Bus}} IRQ {{data.Detail}}"},
    }

    def __new__(cls, settings, *args, **kwargs):
        compatible = dict(settings)
        compatible["accel_unit"] = _compatible_unit(compatible, "accel_unit", "g")
        compatible.setdefault("gyro_unit", "dps")
        compatible.setdefault("fifo_layout", "Auto")
        return super().__new__(cls, compatible, *args, **kwargs)

    def __init__(self):
        self.decoder = Qmi8660Decoder()
        self.i2c = Qmi8660I2cAssembler(self.decoder)
        self.spi = Qmi8660SpiAssembler(self.decoder)

    def decode(self, frame: AnalyzerFrame):
        self._apply_settings()
        frame_type = str(frame.type).lower()
        data = frame.data or {}
        if frame_type in ("start", "address", "data", "stop"):
            emission = self.i2c.feed(frame_type, data, frame.start_time, frame.end_time)
        elif frame_type in ("enable", "disable", "result", "error"):
            emission = self.spi.feed(frame_type, data, frame.start_time, frame.end_time)
        else:
            return None
        if emission is None:
            return None
        transaction = emission.transaction
        if transaction.page_before == 0 and transaction.register == 0x57 and transaction.operation == "READ":
            result_type = "qmi8660_fifo"
        elif transaction.operation == "READ" and self.decoder.is_sensor_data_register(
            transaction.page_before, transaction.register
        ):
            result_type = "qmi8660_sensor"
        elif transaction.page_before == 0 and transaction.register is not None and 0x58 <= transaction.register <= 0x5B and transaction.operation == "READ":
            result_type = "qmi8660_interrupt"
        else:
            result_type = "qmi8660"
        return AnalyzerFrame(result_type, emission.start_time, emission.end_time, transaction.frame_data())

    def _apply_settings(self):
        self.i2c.set_address_mode(str(self.i2c_address))
        self.spi.gap_us = float(self.spi_gap_us)
        self.decoder.set_scale_overrides(_scale_value(self.accel_full_scale), _scale_value(self.gyro_full_scale))
        self.decoder.set_output_units(str(self.accel_unit), str(self.gyro_unit))
        self.decoder.set_fifo_layout_override(str(self.fifo_layout))


class Qmi8658Hla(HighLevelAnalyzer):
    i2c_address = ChoicesSetting(
        label="I2C address", choices=("0x6A or 0x6B", "0x6A", "0x6B", "Any")
    )
    spi_gap_us = NumberSetting(
        label="SPI transaction gap without Enable (us)", min_value=1, max_value=1_000_000
    )
    accel_full_scale = ChoicesSetting(
        label="Accelerometer full scale", choices=("Auto", "2 g", "4 g", "8 g", "16 g")
    )
    accel_unit = ChoicesSetting(label="Accel unit", choices=("g", "mg", "m/s²"))
    gyro_full_scale = ChoicesSetting(
        label="Gyroscope full scale",
        choices=("Auto", "16 dps", "32 dps", "64 dps", "128 dps", "256 dps", "512 dps", "1024 dps", "2048 dps"),
    )
    gyro_unit = ChoicesSetting(label="Gyro unit", choices=("dps", "rad/s"))
    data_byte_order = ChoicesSetting(label="Sensor data byte order", choices=("Auto", "Little endian", "Big endian"))
    fifo_layout = ChoicesSetting(label="FIFO layout", choices=("Auto", "Accel XYZ + Gyro XYZ", "Accel XYZ", "Gyro XYZ"))
    result_types = {
        "qmi8658": {"format": "{{data.Bus}}{{data.Address}} {{data.Op}} {{data.Register}} {{data.Hex}} {{data.Detail}} {{data.Status}}"},
        "qmi8658_sensor": {"format": "{{data.Bus}}{{data.Address}} DATA {{data.Detail}}"},
        "qmi8658_fifo": {"format": "{{data.Bus}} FIFO {{data.Detail}}"},
        "qmi8658_status": {"format": "{{data.Bus}} STATUS {{data.Detail}}"},
    }

    def __new__(cls, settings, *args, **kwargs):
        compatible = dict(settings)
        compatible["accel_unit"] = _compatible_unit(compatible, "accel_unit", "g")
        compatible.setdefault("gyro_unit", "dps")
        compatible.setdefault("data_byte_order", "Auto")
        compatible.setdefault("fifo_layout", "Auto")
        return super().__new__(cls, compatible, *args, **kwargs)

    def __init__(self):
        self.decoder = Qmi8658Decoder()
        self.i2c = Qmi8658I2cAssembler(self.decoder, (0x6A, 0x6B))
        self.spi = Qmi8658SpiAssembler(self.decoder)

    def decode(self, frame: AnalyzerFrame):
        self._apply_settings()
        frame_type = str(frame.type).lower()
        data = frame.data or {}
        if frame_type in ("start", "address", "data", "stop"):
            emission = self.i2c.feed(frame_type, data, frame.start_time, frame.end_time)
        elif frame_type in ("enable", "disable", "result", "error"):
            emission = self.spi.feed(frame_type, data, frame.start_time, frame.end_time)
        else:
            return None
        if emission is None:
            return None
        transaction = emission.transaction
        if transaction.operation == "READ" and transaction.register == 0x17:
            result_type = "qmi8658_fifo"
        elif transaction.operation == "READ" and transaction.register is not None and self.decoder.is_sensor_data_register(transaction.register):
            if not transaction.derived:
                return None
            result_type = "qmi8658_sensor"
        elif transaction.operation == "READ" and transaction.register is not None and self.decoder.is_event_status_register(transaction.register):
            result_type = "qmi8658_status"
        else:
            result_type = "qmi8658"
        return AnalyzerFrame(result_type, emission.start_time, emission.end_time, transaction.frame_data())

    def _apply_settings(self):
        self.i2c.set_address_mode(str(self.i2c_address))
        self.spi.gap_us = float(self.spi_gap_us)
        self.decoder.set_scale_overrides(_scale_value(self.accel_full_scale), _scale_value(self.gyro_full_scale))
        self.decoder.set_output_units(str(self.accel_unit), str(self.gyro_unit))
        self.decoder.set_byte_order_override(str(self.data_byte_order))
        self.decoder.set_fifo_layout_override(str(self.fifo_layout))


class Qma6100pHla(HighLevelAnalyzer):
    i2c_address = ChoicesSetting(label="I2C address", choices=("0x12 or 0x13", "0x12", "0x13", "Any"))
    spi_gap_us = NumberSetting(label="SPI transaction gap without Enable (us)", min_value=1, max_value=1_000_000)
    accel_full_scale = ChoicesSetting(label="Accelerometer full scale", choices=("Auto", "2 g", "4 g", "8 g", "16 g", "32 g"))
    accel_unit = ChoicesSetting(label="Accel unit", choices=("g", "mg", "m/s²"))
    result_types = {
        "qma6100p": {"format": "{{data.Bus}}{{data.Address}} {{data.Op}} {{data.Register}} {{data.Hex}} {{data.Detail}} {{data.Status}}"},
        "qma6100p_fifo": {"format": "{{data.Bus}} FIFO {{data.Detail}}"},
        "qma6100p_accel": {"format": "{{data.Bus}} DATA {{data.Detail}}"},
    }

    def __new__(cls, settings, *args, **kwargs):
        compatible = dict(settings)
        compatible["accel_unit"] = _compatible_unit(compatible, "accel_unit", "g")
        return super().__new__(cls, compatible, *args, **kwargs)

    def __init__(self):
        self.decoder = Qma6100pDecoder()
        self.i2c = Qma6100pI2cAssembler(self.decoder, (0x12, 0x13))
        self.spi = Qma6100pSpiAssembler(self.decoder)

    def decode(self, frame: AnalyzerFrame):
        self._apply_settings()
        frame_type = str(frame.type).lower()
        data = frame.data or {}
        if frame_type in ("start", "address", "data", "stop"):
            emission = self.i2c.feed(frame_type, data, frame.start_time, frame.end_time)
        elif frame_type in ("enable", "disable", "result", "error"):
            emission = self.spi.feed(frame_type, data, frame.start_time, frame.end_time)
        else:
            return None
        if emission is None:
            return None
        transaction = emission.transaction
        if transaction.operation == "READ" and transaction.register == 0x3F:
            result_type = "qma6100p_fifo"
        elif transaction.operation == "READ" and transaction.register == 0x01:
            result_type = "qma6100p_accel"
        else:
            result_type = "qma6100p"
        return AnalyzerFrame(result_type, emission.start_time, emission.end_time, transaction.frame_data())

    def _apply_settings(self):
        self.i2c.set_address_mode(str(self.i2c_address))
        self.spi.gap_us = float(self.spi_gap_us)
        self.decoder.set_scale_override(_scale_value(self.accel_full_scale))
        self.decoder.set_output_unit(str(self.accel_unit))


class Qma6101tHla(HighLevelAnalyzer):
    i2c_address = ChoicesSetting(label="I2C address", choices=("0x12 or 0x13", "0x12", "0x13", "Any"))
    spi_gap_us = NumberSetting(label="SPI transaction gap without Enable (us)", min_value=1, max_value=1_000_000)
    accel_full_scale = ChoicesSetting(label="Accelerometer full scale", choices=("Auto", "2 g", "4 g", "8 g", "16 g", "32 g"))
    accel_unit = ChoicesSetting(label="Accel unit", choices=("g", "mg", "m/s²"))
    result_types = {
        "qma6101t": {"format": "{{data.Bus}}{{data.Address}} {{data.Op}} {{data.Register}} {{data.Hex}} {{data.Detail}} {{data.Status}}"},
        "qma6101t_fifo": {"format": "{{data.Bus}} FIFO {{data.Detail}}"},
        "qma6101t_accel": {"format": "{{data.Bus}} DATA {{data.Detail}}"},
    }

    def __new__(cls, settings, *args, **kwargs):
        compatible = dict(settings)
        compatible["accel_unit"] = _compatible_unit(compatible, "accel_unit", "g")
        return super().__new__(cls, compatible, *args, **kwargs)

    def __init__(self):
        self.decoder = Qma6101tDecoder()
        self.i2c = Qma6101tI2cAssembler(self.decoder, (0x12, 0x13))
        self.spi = Qma6101tSpiAssembler(self.decoder)

    def decode(self, frame: AnalyzerFrame):
        self._apply_settings()
        frame_type = str(frame.type).lower()
        data = frame.data or {}
        if frame_type in ("start", "address", "data", "stop"):
            emission = self.i2c.feed(frame_type, data, frame.start_time, frame.end_time)
        elif frame_type in ("enable", "disable", "result", "error"):
            emission = self.spi.feed(frame_type, data, frame.start_time, frame.end_time)
        else:
            return None
        if emission is None:
            return None
        transaction = emission.transaction
        if transaction.operation == "READ" and transaction.register == 0x3F:
            result_type = "qma6101t_fifo"
        elif transaction.operation == "READ" and transaction.register in range(0x01, 0x07):
            result_type = "qma6101t_accel"
        else:
            result_type = "qma6101t"
        return AnalyzerFrame(result_type, emission.start_time, emission.end_time, transaction.frame_data())

    def _apply_settings(self):
        self.i2c.set_address_mode(str(self.i2c_address))
        self.spi.gap_us = float(self.spi_gap_us)
        self.decoder.set_scale_override(_scale_value(self.accel_full_scale))
        self.decoder.set_output_unit(str(self.accel_unit))
