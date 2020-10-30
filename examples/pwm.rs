//  Based on PWM example program for bcm2835 library by Mike McCauley
//  https://www.airspayce.com/mikem/bcm2835/pwm_8c-example.html


// Connect an LED between GPIO18 (pin 12) and GND to observe the LED changing in brightness

use bcm2835_rs as bcm2835;
use bcm2835_rs::{RPiGPIO, GPIOFSel, PWMClockDivider};

fn main() {
    let init_status =  bcm2835::init();
	match init_status {
		Err(0) => {
			panic!("bcm2835 initialization failed");
		},
		_ => ()
	}


    // PWM output on RPi Plug P1 pin 12 (which is GPIO pin 18) in alt fun 5.
    let pin = RPiGPIO::Pin12;

    // and it is controlled by PWM channel 0
    let pwm_channel = 0;

    // This controls the max range of the PWM signal
    let range = 1024;

    // Set the output pin to Alt Fun 5, to allow PWM channel 0 to be output there
    bcm2835::gpio_fsel(pin, GPIOFSel::Alt5);

    // Clock divider is set to 16.
    // With a divider of 16 and a RANGE of 1024, in MARKSPACE mode,
    // the pulse repetition frequency will be
    // 1.2MHz/1024 = 1171.875Hz, suitable for driving a DC motor with PWM
    bcm2835::pwm_set_clock(PWMClockDivider::Divider1024);
    bcm2835::pwm_set_mode(pwm_channel, 1, 1);
    bcm2835::pwm_set_range(pwm_channel, range);

    // Vary the PWM m/s ratio between 1/RANGE and (RANGE-1)/RANGE
    // over the course of a a few seconds
    let mut direction : i32 = 1;
    let mut data : i32 = 0;

    loop {
        if data == 1 {
            direction = 1; //Switch to increasing
        }
        else if data == (range - 1) as i32 {
            direction = -1; //Switch to decreasing
        }
        data += direction;
        bcm2825::pwm_set_data(pwm_channel, data as u32);
        bcm2825::delay(1);
    }

    bcm2835::close().unwrap();
}
