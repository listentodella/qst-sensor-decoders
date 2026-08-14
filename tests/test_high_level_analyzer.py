import importlib
import sys
import types
import unittest


class _Setting:
    def __init__(self, **_kwargs):
        pass


class _ChoicesSetting(_Setting):
    def __init__(self, choices, **kwargs):
        super().__init__(**kwargs)
        self.choices = choices


class _NumberSetting(_Setting):
    pass


class _HighLevelAnalyzer:
    def __new__(cls, settings, *_args, **_kwargs):
        obj = object.__new__(cls)
        for name, setting in vars(cls).items():
            if not isinstance(setting, _Setting):
                continue
            if name not in settings:
                raise RuntimeError(f"Missing setting: {name}")
            if isinstance(setting, _ChoicesSetting) and settings[name] not in setting.choices:
                raise RuntimeError(f"Invalid choice: {name}")
            setattr(obj, name, settings[name])
        return obj


class _AnalyzerFrame:
    def __init__(self, frame_type, start_time, end_time, data=None):
        self.type = frame_type
        self.start_time = start_time
        self.end_time = end_time
        self.data = data


class HighLevelAnalyzerCompatibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        saleae_module = types.ModuleType("saleae")
        analyzers_module = types.ModuleType("saleae.analyzers")
        analyzers_module.AnalyzerFrame = _AnalyzerFrame
        analyzers_module.ChoicesSetting = _ChoicesSetting
        analyzers_module.HighLevelAnalyzer = _HighLevelAnalyzer
        analyzers_module.NumberSetting = _NumberSetting
        cls.previous_saleae = sys.modules.get("saleae")
        cls.previous_analyzers = sys.modules.get("saleae.analyzers")
        sys.modules["saleae"] = saleae_module
        sys.modules["saleae.analyzers"] = analyzers_module
        sys.modules.pop("HighLevelAnalyzer", None)
        cls.module = importlib.import_module("HighLevelAnalyzer")

    @classmethod
    def tearDownClass(cls):
        sys.modules.pop("HighLevelAnalyzer", None)
        if cls.previous_saleae is None:
            sys.modules.pop("saleae", None)
        else:
            sys.modules["saleae"] = cls.previous_saleae
        if cls.previous_analyzers is None:
            sys.modules.pop("saleae.analyzers", None)
        else:
            sys.modules["saleae.analyzers"] = cls.previous_analyzers

    def test_old_saved_settings_default_new_fifo_layout_to_auto(self):
        legacy_settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "8 g",
            "gyro_full_scale": "2048 dps",
        }
        analyzer = self.module.Qmi8660Hla.__new__(self.module.Qmi8660Hla, legacy_settings)
        self.assertEqual(analyzer.fifo_layout, "Auto")
        self.assertEqual(analyzer.accel_unit, "g")
        self.assertEqual(analyzer.gyro_unit, "dps")

    def test_full_scale_setting_values_are_parsed(self):
        self.assertEqual(self.module._scale_value("8 g"), 8.0)
        self.assertEqual(self.module._scale_value("2048 dps"), 2048.0)
        self.assertIsNone(self.module._scale_value("Auto"))

    def test_hla_applies_selected_scales_and_units_to_decoder(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "8 g",
            "accel_unit": "m/s²",
            "gyro_full_scale": "2048 dps",
            "gyro_unit": "rad/s",
            "fifo_layout": "Gyro XYZ + Accel XYZ",
        }
        analyzer = self.module.Qmi8660Hla.__new__(self.module.Qmi8660Hla, settings)
        self.module.Qmi8660Hla.__init__(analyzer)
        analyzer._apply_settings()
        self.assertEqual(analyzer.decoder.accel_fs_override, 8.0)
        self.assertEqual(analyzer.decoder.gyro_fs_override, 2048.0)
        self.assertEqual(analyzer.decoder.accel_unit, "m/s²")
        self.assertEqual(analyzer.decoder.gyro_unit, "rad/s")

    def test_qmi8658_hla_converts_accel_register_burst(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "8 g",
            "accel_unit": "mg",
            "gyro_full_scale": "2048 dps",
            "gyro_unit": "rad/s",
            "data_byte_order": "Little endian",
            "fifo_layout": "Auto",
        }
        analyzer = self.module.Qmi8658Hla.__new__(self.module.Qmi8658Hla, settings)
        self.module.Qmi8658Hla.__init__(analyzer)
        frames = [
            _AnalyzerFrame("enable", 0.0, 0.1, {}),
            _AnalyzerFrame(
                "result",
                0.1,
                0.2,
                {
                    "mosi": [0xB5, 0, 0, 0, 0, 0, 0],
                    "miso": [0, 0x00, 0x20, 0x00, 0xE0, 0x00, 0x10],
                },
            ),
            _AnalyzerFrame("disable", 0.2, 0.3, {}),
        ]
        output = None
        for frame in frames:
            output = analyzer.decode(frame) or output
        self.assertEqual(output.type, "qmi8658_sensor")
        self.assertIn("A=[2000.0000, -2000.0000, 1000.0000] mg", output.data["Detail"])
        self.assertNotIn("8192", output.data["Detail"])
        self.assertNotIn("Araw", output.data["Detail"])
        sensor_format = self.module.Qmi8658Hla.result_types["qmi8658_sensor"]["format"]
        self.assertEqual(
            sensor_format,
            "{{data.Bus}}{{data.Address}} DATA {{data.Detail}}",
        )
        self.assertNotIn("{{data.Hex}}", sensor_format)
        self.assertNotIn("{{data.Register}}", sensor_format)

    def test_qmi8660_hla_marks_normal_sensor_data_as_data(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "16 g",
            "accel_unit": "g",
            "gyro_full_scale": "2048 dps",
            "gyro_unit": "dps",
            "fifo_layout": "Auto",
        }
        analyzer = self.module.Qmi8660Hla.__new__(self.module.Qmi8660Hla, settings)
        self.module.Qmi8660Hla.__init__(analyzer)
        payload = [0x00, 0x20] * 7
        frames = [
            _AnalyzerFrame("enable", 0.0, 0.1, {}),
            _AnalyzerFrame(
                "result",
                0.1,
                0.2,
                {"mosi": [0xE0] + [0] * 14, "miso": [0] + payload},
            ),
            _AnalyzerFrame("disable", 0.2, 0.3, {}),
        ]
        output = None
        for frame in frames:
            output = analyzer.decode(frame) or output
        self.assertEqual(output.type, "qmi8660_sensor")
        self.assertEqual(
            self.module.Qmi8660Hla.result_types[output.type]["format"],
            "{{data.Bus}}{{data.Address}} DATA {{data.Detail}}",
        )

    def test_qmi8660_configuration_read_is_not_marked_as_data(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "Auto",
            "gyro_full_scale": "Auto",
            "fifo_layout": "Auto",
        }
        analyzer = self.module.Qmi8660Hla.__new__(self.module.Qmi8660Hla, settings)
        self.module.Qmi8660Hla.__init__(analyzer)
        frames = [
            _AnalyzerFrame("enable", 0.0, 0.1, {}),
            _AnalyzerFrame(
                "result", 0.1, 0.2,
                {"mosi": [0xB7, 0], "miso": [0, 0x02]},
            ),
            _AnalyzerFrame("disable", 0.2, 0.3, {}),
        ]
        output = None
        for frame in frames:
            output = analyzer.decode(frame) or output
        self.assertEqual(output.type, "qmi8660")

    def test_qma6100p_hla_marks_accel_and_fifo_sources(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "8 g",
            "accel_unit": "g",
        }
        analyzer = self.module.Qma6100pHla.__new__(self.module.Qma6100pHla, settings)
        self.module.Qma6100pHla.__init__(analyzer)

        def decode_transaction(command, payload):
            frames = [
                _AnalyzerFrame("enable", 0.0, 0.1, {}),
                _AnalyzerFrame(
                    "result", 0.1, 0.2,
                    {"mosi": [command] + [0] * len(payload), "miso": [0] + payload},
                ),
                _AnalyzerFrame("disable", 0.2, 0.3, {}),
            ]
            output = None
            for frame in frames:
                output = analyzer.decode(frame) or output
            return output

        payload = [0x00, 0x40, 0x00, 0xC0, 0x00, 0x20]
        accel = decode_transaction(0x81, payload)
        self.assertEqual(accel.type, "qma6100p_accel")
        self.assertEqual(
            self.module.Qma6100pHla.result_types[accel.type]["format"],
            "{{data.Bus}} DATA {{data.Detail}}",
        )
        fifo = decode_transaction(0xBF, payload)
        self.assertEqual(fifo.type, "qma6100p_fifo")
        self.assertEqual(
            self.module.Qma6100pHla.result_types[fifo.type]["format"],
            "{{data.Bus}} FIFO {{data.Detail}}",
        )

    def test_qmi8658_hla_suppresses_individual_axis_bubble(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "8 g",
            "accel_unit": "g",
            "gyro_full_scale": "2048 dps",
            "gyro_unit": "dps",
            "data_byte_order": "Little endian",
            "fifo_layout": "Auto",
        }
        analyzer = self.module.Qmi8658Hla.__new__(self.module.Qmi8658Hla, settings)
        self.module.Qmi8658Hla.__init__(analyzer)
        frames = [
            _AnalyzerFrame("enable", 0.0, 0.1, {}),
            _AnalyzerFrame(
                "result",
                0.1,
                0.2,
                {"mosi": [0xB5, 0, 0], "miso": [0, 0x00, 0x20]},
            ),
            _AnalyzerFrame("disable", 0.2, 0.3, {}),
        ]
        self.assertTrue(all(analyzer.decode(frame) is None for frame in frames))

    def test_qmi8658_hla_fifo_bubble_contains_all_decoded_frames(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "16 g",
            "accel_unit": "g",
            "gyro_full_scale": "2048 dps",
            "gyro_unit": "dps",
            "data_byte_order": "Little endian",
            "fifo_layout": "Accel XYZ + Gyro XYZ",
        }
        analyzer = self.module.Qmi8658Hla.__new__(self.module.Qmi8658Hla, settings)
        self.module.Qmi8658Hla.__init__(analyzer)
        frame = [
            0x00, 0x10, 0x00, 0xF0, 0x00, 0x08,
            0x00, 0x20, 0x00, 0xE0, 0x00, 0x10,
        ]
        frames = [
            _AnalyzerFrame("enable", 0.0, 0.1, {}),
            _AnalyzerFrame(
                "result",
                0.1,
                0.2,
                {"mosi": [0x97] + [0] * 24, "miso": [0] + frame + frame},
            ),
            _AnalyzerFrame("disable", 0.2, 0.3, {}),
        ]
        output = None
        for frame_event in frames:
            output = analyzer.decode(frame_event) or output
        self.assertEqual(output.type, "qmi8658_fifo")
        self.assertIn("24 B, F=2, 12 B/F", output.data["Detail"])
        self.assertIn("A₁=[2.0000, -2.0000, 1.0000] g", output.data["Detail"])
        self.assertIn("G₂=[512.0000, -512.0000, 256.0000] dps", output.data["Detail"])

    def test_qmi8660_hla_fifo_bubble_contains_statistics_and_all_frames(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "16 g",
            "accel_unit": "g",
            "gyro_full_scale": "4096 dps",
            "gyro_unit": "dps",
            "fifo_layout": "Gyro XYZ + Accel XYZ",
        }
        analyzer = self.module.Qmi8660Hla.__new__(self.module.Qmi8660Hla, settings)
        self.module.Qmi8660Hla.__init__(analyzer)
        frame = [
            0x00, 0x40, 0x00, 0xC0, 0x00, 0x20,
            0x00, 0x10, 0x00, 0xF0, 0x00, 0x08,
        ]
        frames = [
            _AnalyzerFrame("enable", 0.0, 0.1, {}),
            _AnalyzerFrame(
                "result",
                0.1,
                0.2,
                {"mosi": [0xD7] + [0] * 24, "miso": [0] + frame + frame},
            ),
            _AnalyzerFrame("disable", 0.2, 0.3, {}),
        ]
        output = None
        for frame_event in frames:
            output = analyzer.decode(frame_event) or output
        self.assertEqual(output.type, "qmi8660_fifo")
        self.assertIn("24 B, F=2, 12 B/F", output.data["Detail"])
        self.assertIn("A₁=[2.0000, -2.0000, 1.0000] g", output.data["Detail"])
        self.assertIn("G₂=[2048.0000, -2048.0000, 1024.0000] dps", output.data["Detail"])

    def test_fifo_result_types_render_the_complete_detail_field(self):
        for analyzer in (self.module.Qmi8658Hla, self.module.Qmi8660Hla):
            fifo_type = next(name for name in analyzer.result_types if name.endswith("_fifo"))
            fifo_format = analyzer.result_types[fifo_type]["format"]
            self.assertIn("{{data.Detail}}", fifo_format)

    def test_qmi8658_single_fifo_frame_keeps_statistics_and_subscripts(self):
        settings = {
            "i2c_address": "Any", "spi_gap_us": 1,
            "accel_full_scale": "16 g", "accel_unit": "g",
            "gyro_full_scale": "2048 dps", "gyro_unit": "dps",
            "data_byte_order": "Little endian",
            "fifo_layout": "Accel XYZ + Gyro XYZ",
        }
        analyzer = self.module.Qmi8658Hla.__new__(self.module.Qmi8658Hla, settings)
        self.module.Qmi8658Hla.__init__(analyzer)
        payload = [
            0x00, 0x10, 0x00, 0xF0, 0x00, 0x08,
            0x00, 0x20, 0x00, 0xE0, 0x00, 0x10,
        ]
        frames = [
            _AnalyzerFrame("enable", 0.0, 0.1, {}),
            _AnalyzerFrame("result", 0.1, 0.2, {"mosi": [0x97] + [0] * 12, "miso": [0] + payload}),
            _AnalyzerFrame("disable", 0.2, 0.3, {}),
        ]
        output = None
        for frame in frames:
            output = analyzer.decode(frame) or output
        self.assertTrue(output.data["Detail"].startswith("12 B, F=1, 12 B/F; A₁="))
        self.assertIn("G₁=[512.0000, -512.0000, 256.0000] dps", output.data["Detail"])

    def test_qmi8660_single_fifo_frame_keeps_statistics_and_subscripts(self):
        settings = {
            "i2c_address": "Any", "spi_gap_us": 1,
            "accel_full_scale": "16 g", "accel_unit": "g",
            "gyro_full_scale": "4096 dps", "gyro_unit": "dps",
            "fifo_layout": "Gyro XYZ + Accel XYZ",
        }
        analyzer = self.module.Qmi8660Hla.__new__(self.module.Qmi8660Hla, settings)
        self.module.Qmi8660Hla.__init__(analyzer)
        payload = [
            0x00, 0x40, 0x00, 0xC0, 0x00, 0x20,
            0x00, 0x10, 0x00, 0xF0, 0x00, 0x08,
        ]
        frames = [
            _AnalyzerFrame("enable", 0.0, 0.1, {}),
            _AnalyzerFrame("result", 0.1, 0.2, {"mosi": [0xD7] + [0] * 12, "miso": [0] + payload}),
            _AnalyzerFrame("disable", 0.2, 0.3, {}),
        ]
        output = None
        for frame in frames:
            output = analyzer.decode(frame) or output
        self.assertTrue(output.data["Detail"].startswith("12 B, F=1, 12 B/F; A₁="))
        self.assertIn("G₁=[2048.0000, -2048.0000, 1024.0000] dps", output.data["Detail"])

    def test_all_analyzers_expose_only_applicable_unit_choices(self):
        self.assertEqual(self.module.Qmi8660Hla.accel_unit.choices, ("g", "mg", "m/s²"))
        self.assertEqual(self.module.Qmi8660Hla.gyro_unit.choices, ("dps", "rad/s"))
        self.assertEqual(self.module.Qmi8658Hla.accel_unit.choices, ("g", "mg", "m/s²"))
        self.assertEqual(self.module.Qmi8658Hla.gyro_unit.choices, ("dps", "rad/s"))
        self.assertEqual(self.module.Qma6100pHla.accel_unit.choices, ("g", "mg", "m/s²"))
        self.assertFalse(hasattr(self.module.Qma6100pHla, "gyro_unit"))

    def test_ascii_si_unit_from_saved_settings_is_migrated(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "8 g",
            "accel_unit": "m/s^2",
            "gyro_full_scale": "2048 dps",
            "gyro_unit": "dps",
            "fifo_layout": "Auto",
        }
        analyzer = self.module.Qmi8660Hla.__new__(self.module.Qmi8660Hla, settings)
        self.assertEqual(analyzer.accel_unit, "m/s²")

    def test_explicit_fifo_layout_is_preserved(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "8 g",
            "gyro_full_scale": "2048 dps",
            "fifo_layout": "Gyro XYZ + Accel XYZ + Temp",
        }
        analyzer = self.module.Qmi8660Hla.__new__(self.module.Qmi8660Hla, settings)
        self.assertEqual(analyzer.fifo_layout, "Gyro XYZ + Accel XYZ + Temp")

    def test_interrupt_read_uses_compact_irq_result_type(self):
        settings = {
            "i2c_address": "Any",
            "spi_gap_us": 1,
            "accel_full_scale": "Auto",
            "gyro_full_scale": "Auto",
            "fifo_layout": "Auto",
        }
        analyzer = self.module.Qmi8660Hla.__new__(self.module.Qmi8660Hla, settings)
        self.module.Qmi8660Hla.__init__(analyzer)
        frames = [
            _AnalyzerFrame("enable", 0.0, 0.1, {}),
            _AnalyzerFrame(
                "result",
                0.1,
                0.2,
                {"mosi": [0xD8, 0, 0], "miso": [0, 0x40, 0x02]},
            ),
            _AnalyzerFrame("disable", 0.2, 0.3, {}),
        ]
        output = None
        for frame in frames:
            output = analyzer.decode(frame) or output
        self.assertEqual(output.type, "qmi8660_interrupt")
        self.assertTrue(output.data["Detail"].startswith("TRIGGERED: fifo_watermark"))


if __name__ == "__main__":
    unittest.main()
