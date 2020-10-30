//  Based on blink example program for bcm2835 library by Mike McCauley
//https://www.airspayce.com/mikem/bcm2835/blink_8c-example.html

use bcm2835_rs as bcm2835;
use bcm2835_rs::{RPiGPIO, GPIOFSel, HIGH, LOW};

fn main() {
	if bcm2835::init() == 0 {
		return;
	}

	// Blinks on RPi Plug P1 pin 11 (which is GPIO pin 17)
	let pin = RPiGPIO::Pin11; //pin 17 of Raspberry PI

    // Set the pin to be an output
	bcm2835::gpio_fsel(pin, GPIOFSel::Output);

	//Blink
	loop {

		// Turn it on
		bcm2835::gpio_write(pin, HIGH);

		//Wait a bit
		bcm2835::delay(500);

		// Turn it off
		bcm2835::gpio_write(pin, LOW);

		//Wait a bit
		bcm2835::delay(500);
	}

	bcm2835::close();
	return;
}
