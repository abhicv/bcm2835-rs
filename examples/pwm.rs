
use bcm2835_rs as bcm2835;
use bcm2835_rs::{RPiGPIO, GPIOFSel, PWMClockDivider};

fn main() {
    if bcm2835::init() == 0 {
	println!("Could not initialize bcm2835");
	return;
    }

    let pin = RPiGPIO::Pin12;
    let pwm_channel = 0;
    let range = 1024;

    bcm2835::gpio_fsel(pin, GPIOFSel::Alt5);

    bcm2835::pwm_set_clock(PWMClockDivider::Divider1024);
    bcm2835::pwm_set_mode(pwm_channel, 1, 1);
    bcm2835::pwm_set_range(pwm_channel, range);
	
    loop {
        for i in 0..(range - 1) {
            bcm2835::pwm_set_data(pwm_channel, i);
            bcm2835::delay(1);
        }

        for i in (range - 1)..0 {
	    bcm2835::pwm_set_data(pwm_channel, i);
	    bcm2835::delay(1);    
        }
    }

    bcm2835::close();
}
