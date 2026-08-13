# QST Sensor Decoders

Saleae Logic 2 High Level Analyzer extensions for QST motion sensors. The
package exposes three independent analyzers in the Logic 2 analyzer menu:

- `QMI8660`
- `QMI8658A`
- `QMA6100P`

Each analyzer accepts frames from Logic 2's built-in I2C or SPI analyzer and
decodes register transactions, status fields, FIFO payloads, and physical
acceleration/gyro values where the selected device provides them.

Acceleration output can be displayed in `g`, `mg`, or `m/s²`. Gyroscope
output can be displayed in `dps` or `rad/s`. The SI conversions use standard
gravity (`1 g = 9.80665 m/s²`) and `1 dps = pi / 180 rad/s`. Unit selection
applies to both normal sensor-data registers and FIFO samples.

## Installation

Install this folder as a Logic 2 extension using the Extensions panel. After
installation, add the analyzer matching the exact sensor on the board. The
three entries share a package but do not share decoder state, so selecting a
model does not require a second model setting.

## Development

The extension has no runtime dependency on Ruby, RSEQ, or third-party Python
packages. Register JSON files are generated inputs and are shipped with the
extension.

Run the test suite with:

```sh
python3 -m unittest discover -s tests -p 'test_*.py'
```

Register databases can be regenerated from the matching QST/RSEQ YAML sources
with the project's register generator. Generated JSON should be reviewed and
committed together with decoder changes.

## Supported buses

I2C addresses default to the device-specific values and can be restricted or
set to `Any` in analyzer settings. SPI uses the device read-bit convention and
the analyzer's Enable signal when available; the configurable transaction gap
is a fallback for captures without Enable.

## License

MIT. See `LICENSE`.
