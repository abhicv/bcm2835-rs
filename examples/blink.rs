
use bcm2835_rs as bcm2835;
use bcm2835_rs::{RPiGPIO, GPIOFSel, HIGH, LOW};

fn main() {
	if bcm2835::init() == 0 {
		return;
	}

	let LED = RPiGPIO::Pin11; //pin 17 of Raspberry PI

	bcm2835::gpio_fsel(LED, GPIOFSel::Output);

	loop {
		bcm2835::gpio_write(LED, HIGH);
		bcm2835::delay(500);

		bcm2835::gpio_write(LED, LOW);
		bcm2835::delay(500);
	}

	bcm2835::close();
	return;
}
