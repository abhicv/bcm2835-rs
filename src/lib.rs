use bcm2835_sys::*;
use enum_primitive::*;

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

    pub enum GPIOFunctionSelect {
        Input  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_INPT,
        Output  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_OUTP,
        Alt0  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT0,
        Alt1  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT1,
        Alt2  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT2,
        Alt3  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT3,
        Alt4  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT4,
        Alt5  = bcm2835FunctionSelect_BCM2835_GPIO_FSEL_ALT5,
        //MASK: bcm2835FunctionSelect_BCM2835_GPIO_FSEL_MASK,
    }
}

enum_from_primitive! {
#[derive(Debug, Copy, Clone, PartialEq)]
#[repr(u32)]

    pub enum GPIOPUDControl {
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
        P03 = RPiGPIOPin_RPI_GPIO_P1_03,
        P05 = RPiGPIOPin_RPI_GPIO_P1_05,
        P07 = RPiGPIOPin_RPI_GPIO_P1_07,
        P08 = RPiGPIOPin_RPI_GPIO_P1_08,
        P10 = RPiGPIOPin_RPI_GPIO_P1_10,
        P11 = RPiGPIOPin_RPI_GPIO_P1_11,
        P12 = RPiGPIOPin_RPI_GPIO_P1_12,
        P13 = RPiGPIOPin_RPI_GPIO_P1_13,
        P15 = RPiGPIOPin_RPI_GPIO_P1_15,
        P16 = RPiGPIOPin_RPI_GPIO_P1_16,
        P18 = RPiGPIOPin_RPI_GPIO_P1_18,
        P19 = RPiGPIOPin_RPI_GPIO_P1_19,
        P21 = RPiGPIOPin_RPI_GPIO_P1_21,
        P22 = RPiGPIOPin_RPI_GPIO_P1_22,
        P23 = RPiGPIOPin_RPI_GPIO_P1_23,
        P24 = RPiGPIOPin_RPI_GPIO_P1_24,
        P26 = RPiGPIOPin_RPI_GPIO_P1_26,
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


