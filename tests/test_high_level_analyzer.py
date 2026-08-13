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

    def test_all_analyzers_expose_only_applicable_unit_choices(self):
        self.assertEqual(self.module.Qmi8660Hla.accel_unit.choices, ("g", "mg", "m/s²"))
        self.assertEqual(self.module.Qmi8660Hla.gyro_unit.choices, ("dps", "rad/s"))
        self.assertEqual(self.module.Qmi8658aHla.accel_unit.choices, ("g", "mg", "m/s²"))
        self.assertEqual(self.module.Qmi8658aHla.gyro_unit.choices, ("dps", "rad/s"))
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
