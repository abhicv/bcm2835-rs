
use bcm2835_rs as bcm2835;
use bcm2835_rs::{RPiGPIO, GPIOFunctionSelect, HIGH, LOW};

fn main() {
	if bcm2835::init() == 0 {
		return;
	}

	let LED = RPiGPIO::P11; //pin 17 of raspberry pi

	bcm2835::gpio_fsel(LED, GPIOFunctionSelect::Output);

	loop {
		bcm2835::gpio_write(LED, HIGH);
		bcm2835::delay(500);

		bcm2835::gpio_write(LED, LOW);
		bcm2835::delay(500);
	}

	bcm2835::close();
	return;
}
