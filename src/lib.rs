use bcm2835_sys::*;
use enum_primitive::*;
use std::ffi::CString;

pub const HIGH : u8 = 0x1;
pub const LOW : u8 = 0x0;

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum RegisterBase {
        ST   =  bcm2835RegisterBase_BCM2835_REGBASE_ST,
        GPIO =  bcm2835RegisterBase_BCM2835_REGBASE_GPIO,
        PWM  =  bcm2835RegisterBase_BCM2835_REGBASE_PWM,
        CLK  =  bcm2835RegisterBase_BCM2835_REGBASE_CLK,
        PADS =  bcm2835RegisterBase_BCM2835_REGBASE_PADS,
        SPI0 =  bcm2835RegisterBase_BCM2835_REGBASE_SPI0,
        BSC0 =  bcm2835RegisterBase_BCM2835_REGBASE_BSC0,
        BSC1 =  bcm2835RegisterBase_BCM2835_REGBASE_BSC1,
        AUX  =	bcm2835RegisterBase_BCM2835_REGBASE_AUX,
        SPI1 =	bcm2835RegisterBase_BCM2835_REGBASE_SPI1,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum GPIOFSel {
        Input  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_INPT,
        Output  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_OUTP,
        Alt0  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT0,
        Alt1  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT1,
        Alt2  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT2,
        Alt3  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT3,
        Alt4  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT4,
        Alt5  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT5,
        //Mask: bcm2835FunctionSelect_BCM2835_GPIO_FSEL_MASK,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum PudControl {
        PudOff = bcm2835PUDControl_BCM2835_GPIO_PUD_OFF,
        PudDown = bcm2835PUDControl_BCM2835_GPIO_PUD_DOWN,
        PudUp  = bcm2835PUDControl_BCM2835_GPIO_PUD_UP,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum PadGroup {
        Gpio0_27  = bcm2835PadGroup_BCM2835_PAD_GROUP_GPIO_0_27,
        Gpio28_45 = bcm2835PadGroup_BCM2835_PAD_GROUP_GPIO_28_45,
        Gpio46_53 = bcm2835PadGroup_BCM2835_PAD_GROUP_GPIO_46_53,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum RPiGPIO {
        Pin03 = RPiGPIOPin_RPI_GPIO_P1_03,
        Pin05 = RPiGPIOPin_RPI_GPIO_P1_05,
        Pin07 = RPiGPIOPin_RPI_GPIO_P1_07,
        Pin08 = RPiGPIOPin_RPI_GPIO_P1_08,
        Pin10 = RPiGPIOPin_RPI_GPIO_P1_10,
        Pin11 = RPiGPIOPin_RPI_GPIO_P1_11,
        Pin12 = RPiGPIOPin_RPI_GPIO_P1_12,
        Pin13 = RPiGPIOPin_RPI_GPIO_P1_13,
        Pin15 = RPiGPIOPin_RPI_GPIO_P1_15,
        Pin16 = RPiGPIOPin_RPI_GPIO_P1_16,
        Pin18 = RPiGPIOPin_RPI_GPIO_P1_18,
        Pin19 = RPiGPIOPin_RPI_GPIO_P1_19,
        Pin21 = RPiGPIOPin_RPI_GPIO_P1_21,
        Pin22 = RPiGPIOPin_RPI_GPIO_P1_22,
        Pin23 = RPiGPIOPin_RPI_GPIO_P1_23,
        Pin24 = RPiGPIOPin_RPI_GPIO_P1_24,
        Pin26 = RPiGPIOPin_RPI_GPIO_P1_26,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum SPIBitOrder {
        LsbFirst = bcm2835SPIBitOrder_BCM2835_SPI_BIT_ORDER_LSBFIRST,
        MsbFirst = bcm2835SPIBitOrder_BCM2835_SPI_BIT_ORDER_MSBFIRST,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum SPIMode {
        Mode0 = bcm2835SPIMode_BCM2835_SPI_MODE0,
        Mode1 = bcm2835SPIMode_BCM2835_SPI_MODE1,
        Mode2 = bcm2835SPIMode_BCM2835_SPI_MODE2,
        Mode3 = bcm2835SPIMode_BCM2835_SPI_MODE3,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum SPIChipSelect {
        Cs0 = bcm2835SPIChipSelect_BCM2835_SPI_CS0,
        Cs1 = bcm2835SPIChipSelect_BCM2835_SPI_CS1,
        Cs2 = bcm2835SPIChipSelect_BCM2835_SPI_CS2,
        CsNone  = bcm2835SPIChipSelect_BCM2835_SPI_CS_NONE,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum SPIClockDivider {
        Divider65536 = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_65536,
        Divider32768 = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_32768,
        Divider16384 = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_16384,
        Divider8192  = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_8192,
        Divider4096  = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_4096,
        Divider2048  = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_2048,
        Divider1024  = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_1024,
        Divider512   = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_512,
        Divider256   = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_256,
        Divider128   = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_128,
        Divider64    = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_64,
        Divider32    = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_32,
        Divider16    = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_16,
        Divider8     = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_8,
        Divider4     = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_4,
        Divider2     = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_2,
        Divider1     = bcm2835SPIClockDivider_BCM2835_SPI_CLOCK_DIVIDER_1,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum I2CClockDivider {
        Divider2500 = bcm2835I2CClockDivider_BCM2835_I2C_CLOCK_DIVIDER_2500,
        Divider626  = bcm2835I2CClockDivider_BCM2835_I2C_CLOCK_DIVIDER_626,
        Divider150  = bcm2835I2CClockDivider_BCM2835_I2C_CLOCK_DIVIDER_150,
        Divider148  = bcm2835I2CClockDivider_BCM2835_I2C_CLOCK_DIVIDER_148,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum I2CReasonCode {
        ErrorOk = bcm2835I2CReasonCodes_BCM2835_I2C_REASON_OK,
        ErrorNack = bcm2835I2CReasonCodes_BCM2835_I2C_REASON_ERROR_NACK,
        ErrorClkt = bcm2835I2CReasonCodes_BCM2835_I2C_REASON_ERROR_CLKT,
        ErrorData = bcm2835I2CReasonCodes_BCM2835_I2C_REASON_ERROR_DATA,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum PWMClockDivider {
        Divider2048  = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_2048,
        Divider1024  = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_1024,
        Divider512   = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_512,
        Divider256   = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_256,
        Divider128   = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_128,
        Divider64    = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_64,
        Divider32    = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_32,
        Divider16    = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_16,
        Divider8     = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_8,
        Divider4     = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_4,
        Divider2     = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_2,
        Divider1     = bcm2835PWMClockDivider_BCM2835_PWM_CLOCK_DIVIDER_1,
    }
}

pub fn init() -> u8 {
    unsafe {
        match bcm2835_init() {
            0 => 0,
            _ => 1,
        }
    }
}

pub fn close() -> u8 {
    unsafe {
        match bcm2835_close() {
            0 => 0,
            _ => 1,
        }
    }
}

pub fn set_debug(debug : u8) {
    unsafe {
        bcm2835_set_debug(debug);
    }
}

pub fn version() -> u32 {
    unsafe {
        bcm2835_version()
    }
}

pub fn regbase(regbase : RegisterBase) -> &'static mut u32 {
    unsafe {
        let r = bcm2835_regbase(regbase as u8);
        return &mut *r;
    }
}

pub fn peri_read(paddr : &mut u32) -> u32 {
    unsafe {
        bcm2835_peri_read(paddr)
    }
}

pub fn peri_read_nb(paddr : &mut u32) -> u32 {
    unsafe {
        bcm2835_peri_read_nb(paddr)
    }
}

pub fn peri_write(paddr : &mut u32, value : u32) {
    unsafe {
        bcm2835_peri_write(paddr, value);
    }
}

pub fn peri_write_nb(paddr : &mut u32, value : u32) {
    unsafe {
        bcm2835_peri_write_nb(paddr, value);
    }
}

pub fn peri_set_bits(paddr : &mut u32, value : u32, mask : u32) {
    unsafe {
        bcm2835_peri_set_bits(paddr, value, mask);
    }
}

pub fn gpio_fsel(pin : RPiGPIO, mode : GPIOFSel) {
    unsafe {
        bcm2835_gpio_fsel(pin as u8, mode as u8);
    }
}

pub fn gpio_set(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_set(pin as u8);
    }
}

pub fn gpio_clr(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_clr(pin as u8);
    }
}

pub fn gpio_set_multi(mask : u32) {
    unsafe {
        bcm2835_gpio_set_multi(mask);
    }
}

pub fn gpio_clr_multi(mask : u32) {
    unsafe {
        bcm2835_gpio_clr_multi(mask);
    }
}

pub fn gpio_lev(pin : RPiGPIO) -> u8 {
    unsafe {
        bcm2835_gpio_lev(pin as u8)
    }
}

pub fn gpio_eds(pin : RPiGPIO) -> u8 {
    unsafe {
        bcm2835_gpio_eds(pin as u8)
    }
}

pub fn gpio_eds_multi(mask : u32) -> u32 {
    unsafe {
        bcm2835_gpio_eds_multi(mask)
    }
}

pub fn gpio_set_eds(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_set_eds(pin as u8);
    }
}

pub fn gpio_set_eds_multi(mask : u32) {
    unsafe {
        bcm2835_gpio_set_eds_multi(mask);
    }
}

pub fn gpio_ren(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_ren(pin as u8);
    }
}

pub fn gpio_clr_ren(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_clr_ren(pin as u8);
    }
}

pub fn gpio_fen(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_fen(pin as u8);
    }
}

pub fn gpio_clr_fen(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_clr_fen(pin as u8);
    }
}

pub fn gpio_hen(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_hen(pin as u8);
    }
}

pub fn gpio_clr_hen(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_clr_hen(pin as u8);
    }
}

pub fn gpio_len(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_len(pin as u8);
    }
}

pub fn gpio_clr_len(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_clr_len(pin as u8);
    }
}

pub fn gpio_aren(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_aren(pin as u8);
    }
}

pub fn gpio_clr_aren(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_clr_aren(pin as u8);
    }
}

pub fn gpio_afen(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_afen(pin as u8);
    }
}

pub fn gpio_clr_afen(pin : RPiGPIO) {
    unsafe {
        bcm2835_gpio_clr_afen(pin as u8);
    }
}

pub fn gpio_pud(pud : PudControl) {
    unsafe {
        bcm2835_gpio_pud(pud as u8);
    }
}

pub fn gpio_pudclk(pin : RPiGPIO, on : u8) {
    unsafe {
        bcm2835_gpio_pudclk(pin as u8, on);
    }
}

pub fn gpio_pad(group : PadGroup) -> u32 {
    unsafe {
        bcm2835_gpio_pad(group as u8)
    }
}

pub fn gpio_set_pad(group : PadGroup, control : u32) {
    unsafe {
        bcm2835_gpio_set_pad(group as u8, control);
    }
}

pub fn delay(millis : u32) {
    unsafe {
        bcm2835_delay(millis);
    }
}

pub fn delay_microseconds(micros : u64) {
    unsafe {
        bcm2835_delayMicroseconds(micros);
    }
}

pub fn gpio_write(pin : RPiGPIO, on : u8) {
    unsafe {
        bcm2835_gpio_write(pin as u8, on);
    }
}

pub fn gpio_write_multi(mask : u32 , on : u8) {
    unsafe {
        bcm2835_gpio_write_multi(mask , on);
    }
}

pub fn gpio_write_mask(value : u32, mask : u32) {
    unsafe {
        bcm2835_gpio_write_mask(value, mask);
    }
}

pub fn gpio_set_pud(pin : RPiGPIO, pud : PudControl) {
    unsafe {
        bcm2835_gpio_set_pud(pin as u8, pud as u8);
    }
}

pub fn gpio_get_pud(pin : RPiGPIO) -> Option<PudControl> {
    unsafe {
        let pud = bcm2835_gpio_get_pud(pin as u8);
        PudControl::from_u8(pud)
    }
}

pub fn spi_begin() -> u8 {
    unsafe {
        match bcm2835_spi_begin() {
            0 => 0,
            _ => 1,
        }
    }
}

pub fn spi_end() {
    unsafe {
        bcm2835_spi_end();
    }
}

pub fn spi_set_bit_order(order : SPIBitOrder) {
    unsafe {
        bcm2835_spi_setBitOrder(order as u8);
    }
}

pub fn spi_set_clock_divider(divider : SPIClockDivider) {
    unsafe {
        bcm2835_spi_setClockDivider(divider as u16);
    }
}

pub fn spi_set_speed_hz(speed_hz : u32) {
    unsafe {
        bcm2835_spi_set_speed_hz(speed_hz);
    }
}

pub fn spi_set_data_mode(mode : SPIMode) {
    unsafe {
        bcm2835_spi_setDataMode(mode as u8);
    }
}

pub fn spi_chip_select(cs : SPIChipSelect) {
    unsafe {
        bcm2835_spi_chipSelect(cs as u8);
    }
}

pub fn spi_set_chip_select_polarity(cs : SPIChipSelect, active : u8) {
    unsafe {
        bcm2835_spi_setChipSelectPolarity(cs as u8, active);
    }
}

pub fn spi_transfer(value : u8) -> u8{
    unsafe {
        bcm2835_spi_transfer(value)
    }
}

pub fn spi_transfernb(tbuf : &[u8]) -> String {
    unsafe {
        let tbuf_raw = CString::new(tbuf).unwrap().into_raw();
        let rbuf_raw = CString::new(tbuf).unwrap().into_raw();

        bcm2835_spi_transfernb(tbuf_raw, rbuf_raw, tbuf.len() as u32);
        let rbuf_cstr = CString::from_raw(rbuf_raw);

        return rbuf_cstr.into_string().unwrap();
    }
}



