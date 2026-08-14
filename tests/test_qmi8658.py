import unittest

from qmi8658_decode import I2cAssembler, Qmi8658Decoder, SpiAssembler


class Qmi8658DecoderTests(unittest.TestCase):
    def test_spi_who_am_i(self):
        decoder = Qmi8658Decoder()
        decoded = decoder.decode_spi([0x80, 0], [0, 0x05])
        self.assertEqual(decoded.operation, "READ")
        self.assertEqual(decoded.register_name, "WHO_AM_I")
        self.assertIn("matched", decoded.derived[0])

    def test_ranges_and_little_endian_data_all(self):
        decoder = Qmi8658Decoder()
        decoder.decode_spi([0x02, 0x00], [0, 0])
        decoder.decode_spi([0x03, 0x20], [0, 0])  # 8 g
        decoder.decode_spi([0x04, 0x60], [0, 0])  # 1024 dps
        payload = [
            0x00, 0x19,
            0x00, 0x40, 0x00, 0xC0, 0x00, 0x20,
            0x00, 0x40, 0x00, 0xC0, 0x00, 0x20,
        ]
        decoded = decoder.decode_spi([0xB3] + [0] * 14, [0] + payload)
        self.assertEqual(decoded.register_name, "DATA_ALL")
        self.assertEqual(decoded.derived[0], "T=25.000 C")
        self.assertEqual(decoded.derived[1], "A=[4.0000, -4.0000, 2.0000] g")
        self.assertEqual(decoded.derived[2], "G=[512.0000, -512.0000, 256.0000] dps")

    def test_data_all_converts_selected_units(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(8, 360)
        decoder.set_output_units("mg", "rad/s")
        payload = [
            0x00, 0x00,
            0x00, 0x20, 0x00, 0xE0, 0x00, 0x10,
            0x00, 0x40, 0x00, 0xC0, 0x00, 0x20,
        ]
        decoded = decoder.decode_spi([0xB3] + [0] * 14, [0] + payload)
        self.assertEqual(decoded.derived[1], "A=[2000.0000, -2000.0000, 1000.0000] mg")
        self.assertEqual(decoded.derived[2], "G=[3.1416, -3.1416, 1.5708] rad/s")

    def test_accel_only_burst_has_qmi8660_style_physical_values(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(8, 2048)
        decoder.set_output_units("m/s²", "dps")
        payload = [0x00, 0x20, 0x00, 0xE0, 0x00, 0x10]
        decoded = decoder.decode_spi([0xB5] + [0] * 6, [0] + payload)
        self.assertEqual(decoded.register_name, "AX_L")
        self.assertEqual(decoded.derived[0], "A=[19.6133, -19.6133, 9.8066] m/s²")
        self.assertEqual(decoded.fields, [])

    def test_gyro_only_burst_has_qmi8660_style_physical_values(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(8, 2048)
        decoder.set_output_units("g", "rad/s")
        payload = [0x00, 0x20, 0x00, 0xE0, 0x00, 0x10]
        decoded = decoder.decode_spi([0xBB] + [0] * 6, [0] + payload)
        self.assertEqual(decoded.register_name, "GX_L")
        self.assertEqual(decoded.derived[0], "G=[8.9361, -8.9361, 4.4680] rad/s")

    def test_single_axis_read_is_not_emitted_as_a_vector(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(8, 2048)
        decoded = decoder.decode_i2c(0x6A, "READ", 0x35, [0x00, 0x20])
        self.assertEqual(decoded.derived, [])
        self.assertEqual(decoded.fields, [])

    def test_incomplete_axis_read_is_not_decoded(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(8, 2048)
        decoded = decoder.decode_spi([0xB5, 0], [0, 0x20])
        self.assertEqual(decoded.derived, [])
        self.assertEqual(decoded.fields, [])

    def test_sensor_vector_without_scale_does_not_show_raw_values(self):
        decoder = Qmi8658Decoder()
        payload = [0x00, 0x20, 0x00, 0xE0, 0x00, 0x10]
        decoded = decoder.decode_spi([0xB5] + [0] * 6, [0] + payload)
        self.assertEqual(decoded.derived, ["A: set accelerometer full scale"])
        self.assertNotIn("8192", decoded.derived[0])
        self.assertNotIn("raw", decoded.derived[0].lower())

    def test_ctrl1_big_endian_data(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(8, 1024)
        decoder.decode_spi([0x02, 0x20], [0, 0])
        payload = [
            0x19, 0x00,
            0x40, 0x00, 0xC0, 0x00, 0x20, 0x00,
            0x40, 0x00, 0xC0, 0x00, 0x20, 0x00,
        ]
        decoded = decoder.decode_spi([0xB3] + [0] * 14, [0] + payload)
        self.assertEqual(decoded.derived[1], "A=[4.0000, -4.0000, 2.0000] g")

    def test_fifo_order_is_accel_then_gyro(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(16, 2048)
        decoder.decode_spi([0x08, 0x03], [0, 0])
        payload = [
            0x00, 0x10, 0x00, 0xF0, 0x00, 0x08,
            0x00, 0x20, 0x00, 0xE0, 0x00, 0x10,
        ]
        decoded = decoder.decode_spi([0x97] + [0] * 12, [0] + payload)
        self.assertEqual(
            decoded.derived[0],
            "12 B, F=1, 12 B/F",
        )
        self.assertEqual(decoded.derived[1], "A₁=[2.0000, -2.0000, 1.0000] g, G₁=[512.0000, -512.0000, 256.0000] dps")

    def test_fifo_summary_preserves_multi_frame_byte_statistics(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(16, 2048)
        decoder.set_fifo_layout_override("Accel XYZ + Gyro XYZ")
        frame = [
            0x00, 0x10, 0x00, 0xF0, 0x00, 0x08,
            0x00, 0x20, 0x00, 0xE0, 0x00, 0x10,
        ]
        decoded = decoder.decode_spi([0x97] + [0] * 24, [0] + frame + frame)
        self.assertEqual(
            decoded.derived[0],
            "24 B, F=2, 12 B/F",
        )
        self.assertEqual(
            decoded.derived[1],
            "A₁=[2.0000, -2.0000, 1.0000] g, G₁=[512.0000, -512.0000, 256.0000] dps;"
            "A₂=[2.0000, -2.0000, 1.0000] g, G₂=[512.0000, -512.0000, 256.0000] dps",
        )

    def test_fifo_summary_reports_trailing_bytes(self):
        decoder = Qmi8658Decoder()
        decoder.set_fifo_layout_override("Accel XYZ")
        decoded = decoder.decode_spi([0x97] + [0] * 8, [0] + [0] * 8)
        self.assertEqual(
            decoded.derived[0],
            "8 B, F=1, 6 B/F, tail=2 B",
        )

    def test_auto_does_not_guess_layout_without_configuration(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(16, 2048)
        payload = [
            0x00, 0x10, 0x00, 0xF0, 0x00, 0x08,
            0x00, 0x20, 0x00, 0xE0, 0x00, 0x10,
        ]
        decoded = decoder.decode_spi([0x97] + [0] * 12, [0] + payload)
        self.assertEqual(decoded.derived, ["12 B, F=?"])

    def test_auto_leaves_ambiguous_single_sensor_payload_unidentified(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(16, 2048)
        decoded = decoder.decode_spi([0x97] + [0] * 6, [0] + [0] * 6)
        self.assertEqual(decoded.derived, ["6 B, F=?"])

    def test_fifo_converts_to_meters_per_second_squared(self):
        decoder = Qmi8658Decoder()
        decoder.set_scale_overrides(16, 2048)
        decoder.set_output_units("m/s²", "dps")
        decoder.decode_spi([0x08, 0x01], [0, 0])
        payload = [0x00, 0x10, 0x00, 0xF0, 0x00, 0x08]
        decoded = decoder.decode_spi([0x97] + [0] * 6, [0] + payload)
        self.assertEqual(decoded.derived[1], "A₁=[19.6133, -19.6133, 9.8066] m/s²")

    def test_fifo_pointer_does_not_increment(self):
        decoder = Qmi8658Decoder()
        self.assertEqual(decoder.advance_i2c_pointer(0x17, 24), 0x17)
        self.assertEqual(decoder.advance_i2c_pointer(0x33, 14), 0x41)

    def test_interrupt_status_prioritizes_active_event(self):
        decoder = Qmi8658Decoder()
        decoded = decoder.decode_spi([0xAD, 0], [0, 0x81])
        self.assertEqual(decoded.fields[0], "TRIGGERED: ctrl9_done, data_ready")
        self.assertIn("inactive: locked", decoded.fields[1])

    def test_i2c_repeated_start(self):
        decoder = Qmi8658Decoder()
        i2c = I2cAssembler(decoder, (0x6A, 0x6B))
        emission = None
        for frame in [
            ("start", {}, 0.0, 0.1),
            ("address", {"address": [0x6A], "read": False}, 0.1, 0.2),
            ("data", {"data": [0x00]}, 0.2, 0.3),
            ("start", {}, 0.3, 0.4),
            ("address", {"address": [0x6A], "read": True}, 0.4, 0.5),
            ("data", {"data": [0x05], "ack": False}, 0.5, 0.6),
            ("stop", {}, 0.6, 0.7),
        ]:
            emission = i2c.feed(*frame) or emission
        self.assertIsNotNone(emission)
        self.assertEqual(emission.transaction.register_name, "WHO_AM_I")
        self.assertEqual(emission.transaction.device_address, 0x6A)

    def test_spi_assembler_uses_enable_boundary(self):
        decoder = Qmi8658Decoder()
        spi = SpiAssembler(decoder)
        self.assertIsNone(spi.feed("enable", {}, 0.0, 0.1))
        self.assertIsNone(
            spi.feed("result", {"mosi": [0x80, 0], "miso": [0, 0x05]}, 0.1, 0.2)
        )
        emission = spi.feed("disable", {}, 0.2, 0.3)
        self.assertEqual(emission.transaction.register_name, "WHO_AM_I")


if __name__ == "__main__":
    unittest.main()
