use bcm2835_rs as bcm2835;
use bcm2835::{RPiGPIO, GPIOFSel, PWMClockDivider};

//NOTE(abhicv): Connect an servo signal pin to GPIO18 (pin 12)
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
    let range = 6000;

    // Set the output pin to Alt Fun 5, to allow PWM channel 0 to be output there
    bcm2835::gpio_fsel(pin, GPIOFSel::Alt5);

    // Clock divider is set to 64.
    // With a divider of 64 and a RANGE of 6000, in MARKSPACE mode,
    // the pulse repetition frequency will be
    // 300kHz/6000 = 50Hz, suitable for the servo motor
    bcm2835::pwm_set_clock(PWMClockDivider::Divider64);
    bcm2835::pwm_set_mode(pwm_channel, 1, 1);
    bcm2835::pwm_set_range(pwm_channel, range);

    loop {
        bcm2835::pwm_set_data(pwm_channel, 200); //min 5% duty cycle
        bcm2835::delay(200);
        bcm2835::pwm_set_data(pwm_channel, 450); //mid 7.5% duty cycle
        bcm2835::delay(200);
        bcm2835::pwm_set_data(pwm_channel, 600); //max 10% duty cycle
        bcm2835::delay(200);
    }

    bcm2835::close().unwrap();
}
