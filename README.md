# PAC193X Arduino Library

This is an Arduino library for the PAC193x series of I2C/SMBus power/energy monitor ICs made by Microchip. It supports the [PAC1931](https://www.microchip.com/wwwproducts/en/PAC1931), [PAC1932](https://www.microchip.com/wwwproducts/en/PAC1932), [PAC1933](https://www.microchip.com/wwwproducts/en/PAC1933), and [PAC1934](https://www.microchip.com/wwwproducts/en/PAC1934) devices, which have 1-4 channels respectively. These chips can measure voltage, current, power, and accumulated power values. The voltage and current measurements can be read as instantaneous values or averages over the last 8 readings.

## IN-DEVELOPMENT WARNING

This library is currently in development, and hasn't actually been tested on hardware. I don't currently own a PAC193x IC and I'm writing all of this based on reference code and the datasheet. I should hopefully have access to a PAC1934 soon for testing purposes.

## Why this library?

Microchip wrote an Arduino library for these chips, but it is... not very good. It has limited support for the chip's functionality, doesn't support basic things like bipolar measurement (negative voltage/current readings) or having different shunt resistances per channel, does everything synchronously, uses extremely excessive sleep times during measurement, and makes an awful lot of assumptions about your configuration.

Features of this library include:

- Unipolar (positive only) and bipolar (positive or negative) measurement options for both voltage and current, per channel
- Automatic detection of unipolar/bipolar power measurement based on voltage and current polarity configuration
- Voltage/current readings as both instantaneous and mean average (last 8 samples)
- High-precision power measurement using the chip's internal computation
- Power accumulator and counter, which can be used to compute energy consumption (just integrate w.r.t. time)
- Support for different current measurement shunt resistances per channel
- Power accumulator overflow detection
- Asynchronous and synchronous refresh methods
- Optional status code parameter for all calls that interact with the chip, to aid debugging and error detection
- Practical sampling rate of up to 950Hz (each refresh operation takes 1ms in hardware; the library waits 1.05ms for safety)

I also put the effort in to ensure that the device registers are nicely documented in code, to make it easier to follow what's being done and avoid having to refer back to the datasheet all the time.

## Planned Features

In rough order of priority:

- [ ] Support for full 1024Hz sample rate
- [ ] One-shot measurement mode (sleep mode)
- [ ] External measurement trigger support (via SLOW register)
- [ ] Expose power-on-reset (POR) bit
- [ ] Expose SLOW status bits
- [ ] ALERT pin support
- [ ] Enable/disable SMBus timeout

## Contributing

I'd be very happy to receive issue reports and PRs!

For PRs, please try to stick to the same coding style, and include usage comments before all function definitions in the header file.

## License

The source code of this project is released under [MIT License](LICENSE).

## Code of Conduct

All contributors to this project are expected to agree with and abide by the [code of conduct](CONDUCT.md). Please familiarise yourself with its contents.