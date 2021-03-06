# bcm2835-rs
- Rust Bindings for bcm2835 C library by Mike McCauley for Broadcom BCM 2835 as used in Raspberry PI (https://www.airspayce.com/mikem/bcm2835/)
- Bindings are incomplete (its under development).

## Installation and Usage
- Require libbcm2835 shared library to be installed on the Raspberry PI.
- [Download](http://www.airspayce.com/mikem/bcm2835/bcm2835-1.68.tar.gz) the latest version of the library, say bcm2835-1.xx.tar.gz, then:
```bash
tar zxvf bcm2835-1.xx.tar.gz
cd bcm2835-1.xx
./configure
make
sudo make check
sudo make install
```
- Clone this repo and then:
```bash
cd bcm2835-rs
cargo run --example blink
```
- connect a LED to pin 11(GPIO 17) of your Raspberry PI and see it blinking.

## License
[GPL v3.0](https://choosealicense.com/licenses/gpl-3.0/)