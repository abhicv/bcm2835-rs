use bcm2835_rs as bcm2835;
use bcm2835_rs::{RPiGPIO, GPIOFSel, PWMClockDivider};

fn main() {
    if bcm2835::init() == 0 {
		return;
    }

    let servo_signal = RPiGPIO::Pin12;
    let pwm_channel = 0;
    let range = 383;

    //needed freq is 50Hz
    // clk = 19.6Mhz / 1024
    //range = clk / 50 = 383

    bcm2835::gpio_fsel(servo_signal, GPIOFSel::Alt5);

    bcm2835::pwm_set_clock(PWMClockDivider::Divider1024);
    bcm2835::pwm_set_mode(pwm_channel, 1, 1);
    bcm2835::pwm_set_range(pwm_channel, range);

    for i in 0..38 {
        bcm2835::pwm_set_data(pwm_channel, i);
        bcm2835::delay(1);
    }

    bcm2835::close();
}