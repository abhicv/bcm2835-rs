use bcm2835-rs as bcm2835;
use bcm2835::{SPIBitOrder, SPIMode, SPIChipSelect, SPIClockDivider};

fn main() {
	if bcm2835::init() == 0 {
        println!("bcm2835 init failed. Are you running as root?");
		return;
    }

    if bcm2835::spi_begin() == 0 {
        println!("bcm2835 spi begin failed. Are you running as root??\n");
        return;
    }

    bcm2835::spi_setBitOrder(SPIBitOrder::Lsbfirst);      // The default
    bcm2835::spi_setDataMode(SPIMode::Mode0);                   // The default
    bcm2835::spi_setClockDivider(SPIClockDivider::Divider65536); // The default
    bcm2835::spi_chipSelect(SPIChipSelect::Cs0);                      // The default
    bcm2835::spi_setChipSelectPolarity(SPIChipSelect::Cs0, bcm2835::LOW);      // the default

    // Send a byte to the slave and simultaneously read a byte back from the slave
    // If you tie MISO to MOSI, you should read back what was sent
    let send_data = 0x23;
    let read_data = bcma2835::spi_transfer(send_data);

    println!("Sent to SPI: {:?}. Read back from SPI: {:?}", send_data, read_data);
    if (send_data != read_data) {
        println!("Do you have the loopback from MOSI to MISO connected?");
    }
    bcm2835::spi_end();
    bcm2835::close();
	return;
}