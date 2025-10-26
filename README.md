```
 ⢎⡑⡎⠑⢹⠁⡇ ⣏⡉⣎⣵⣎⣵⣎⣵ ⡏⢱⣏⡱⡇⡇⢸⣏⡉⣏⡱
 ⠢⠜⠣⠝⠸ ⠧⠤⠤⠜⠫⠜⠫⠜⠫⠜ ⠧⠜⠇⠱⠇⠸⠃⠧⠤⠇⠱
 an I2C driver for Embedded HAL
```

Embedded HAL I2C driver for the [SGTL5000 Audio Chip](https://www.nxp.com/products/audio/audio-converters/ultra-low-power-audio-codec:SGTL5000). 

The SGTL5000 is a low-power stereo codec designed to provide a comprehensive audio solution for portable products that require line-in, mic-in, line-out, headphone-out and digital I/O.

Featuring:

- Ultra-low-power with very high performance and functionality
- Capless headphone and an integrated PLL to allow clock reuse within the system that helps achieve a lower overall system cost.

> [!CAUTION]
> This project is actively being developed with frequent breaking changes. APIs may shift, features are incomplete, and stability is not guaranteed. Use at your own risk and expect regular updates that might require code adjustments. Have fun!

> [!IMPORTANT]
> **Hi Squeaky Things** can happen at any time. This driver let the [_Little Weirdo_](https://github.com/hi-squeaky-things/little-weirdo) squeak, squuuueak, squeeeeeaak, squeaaaaaaaaak!

## Examples

In the example folder you find two ESP32 examples:

1. loopback test (listen to input and stream it to the output).
2. play a wav-file continuously.

## Credits

- Inspiration is taken from the teensy implementation of the SGTL5000 driver.
- [Small Braille ASCII Font](https://patorjk.com/software/taag/#p=display&f=Small+Braille&t=LITTLE+WEIRDO&x=rainbow1&v=1&h=1&w=80&we=false)

