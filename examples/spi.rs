//  Based on SPI example program for bcm2835 library by Mike McCauley
//https://www.airspayce.com/mikem/bcm2835/spi_8c-example.html

use bcm2835::{SPIBitOrder, SPIChipSelect, SPIClockDivider, SPIMode, LOW};
use bcm2835_rs as bcm2835;

fn main() {
    let init_status = bcm2835::init();
    match init_status {
        Err(0) => {
            panic!("bcm2835 initialization failed!");
        }
        _ => (),
    }

    let spi_begin_status = bcm2835::spi_begin();
    match spi_begin_status {
        Err(0) => {
            panic!("SPI begin failed! Are you running as root??");
        }
        _ => (),
    }

    bcm2835::spi_set_bit_order(SPIBitOrder::LsbFirst); // The default
    bcm2835::spi_set_data_mode(SPIMode::Mode0); // The default
    bcm2835::spi_set_clock_divider(SPIClockDivider::Divider65536); // The default
    bcm2835::spi_chip_select(SPIChipSelect::Cs0); // The default
    bcm2835::spi_set_chip_select_polarity(SPIChipSelect::Cs0, LOW); // the default

    // Send a byte to the slave and simultaneously read a byte back from the slave
    // If you tie MISO to MOSI, you should read back what was sent
    let send_data = 0x23;
    let read_data = bcm2835::spi_transfer(send_data);

    println!("Sent to SPI : {:?}", send_data);
    println!("Read back from SPI: {:?}", read_data);

    if send_data != read_data {
        println!("Do you have the loopback from MOSI to MISO connected?");
    }

    bcm2835::spi_end();
    bcm2835::close().unwrap();
}
