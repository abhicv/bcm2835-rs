use bcm2835_rs as bcm2835;

fn main() {
	if bcm2835::init() == 0 {
		return;
	}

	bcm2835::gpio_fsel(bcm2835::RPiGPIO::P11, bcm2835::GPIOFunctionSelect::Input);

	loop {
		bcm2835::gpio_write(bcm2835::RPiGPIO::P11, bcm2835::HIGH);
		bcm2835::delay(500);

		bcm2835::gpio_write(bcm2835::RPiGPIO::P11, bcm2835::HIGH);
		bcm2835::delay(500);
	}

	bcm2835::close();
	return;
}
