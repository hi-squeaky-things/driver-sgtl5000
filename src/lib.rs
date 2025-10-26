#![no_std]
//!
//! Embedded Hal I2C Driver for the NXP SGTL5000 codec chip
//!
//! Example usage:
//! ``` no_run
//! use driver_sgtl5000::SGTL5000;
//!
//! // Make sure you provide the NXP SGTL5000 with a MCLK signal first before using I2C.
//! fn main() {
//!     
//!     ...
//!
//!     // Setup the I2C peripherals.
//!     // Depending  on your target system and selected pins, this must be adapted to you needs
//!     let mut config = Config::default();
//!     let mut i2c = I2c::new(peripherals.I2C0, config)
//!         .with_sda(peripherals.GPIO17)
//!         .with_scl(peripherals.GPIO18);
//!
//!     // Initialize the driver using the I2C peripheral
//!     let mut sgtl500 = SGTL5000::new(i2c);
//!     // Check if the codec is ready!
//!     sgtl500.ready();
//!     // Initalized the codec
//!     sgtl500.init();
//!     // Do stuff with the codec
//!     sgtl500.headphone_volume(60);
//!     sgtl500.headphone_unmute();
//!
//!     ...
//! }
//! ```
//!
use embedded_hal::i2c::I2c;
use registers::*;

#[derive(Debug, Copy, Clone)]
pub struct SGTL5000<I2C> {
    i2c: I2C,
    address: u8,
}

pub mod registers {

    pub const SGTL5000_I2C_ADDR_CS_LOW: u8 = 0x0A;
    pub const SGTL5000_I2C_ADDR_CS_HIGH: u8 = 0x2A;
    pub const SGTL5000_CHIP_ID: u8 = 0xA0;

    pub const CHIP_ID: u16 = 0x0000;
    pub const CHIP_DIG_POWER: u16 = 0x0002;
    pub const CHIP_CLK_CTRL: u16 = 0x0004;
    pub const CHIP_I2S_CTRL: u16 = 0x0006;
    pub const CHIP_SSS_CTRL: u16 = 0x000A;
    pub const CHIP_ADCDAC_CTRL: u16 = 0x000E;
    pub const CHIP_DAC_VOL: u16 = 0x0010;
    pub const CHIP_PAD_STRENGTH: u16 = 0x0014;
    pub const CHIP_ANA_ADC_CTRL: u16 = 0x0020;
    pub const CHIP_ANA_HP_CTRL: u16 = 0x0022;

    ///
    pub const CHIP_ANA_CTRL: u16 = 0x0024;
    pub const CHIP_ANA_CTRL_ADC_MUTE_BIT: u8 = 0;
    pub const CHIP_ANA_CTRL_SELECT_ADC_BIT: u8 = 2;
    pub const CHIP_ANA_CTRL_MUTE_HP_BIT: u8 = 4;
    pub const CHIP_ANA_CTRL_SELECT_HP_INPUT_BIT: u8 = 6;
    pub const CHIP_ANA_CTRL_MUTE_LO_BIT: u8 = 8;

    pub const CHIP_LINREG_CTRL: u16 = 0x0026;
    pub const CHIP_REF_CTRL: u16 = 0x0028;
    pub const CHIP_MIC_CTRL: u16 = 0x002A;
    pub const CHIP_LINE_OUT_CTRL: u16 = 0x002C;
    pub const CHIP_LINE_OUT_VOL: u16 = 0x002E;
    pub const CHIP_ANA_POWER: u16 = 0x0030;
    pub const CHIP_PLL_CTRL: u16 = 0x0032;
    pub const CHIP_CLK_TOP_CTRL: u16 = 0x0034;
    pub const SHIP_ANA_STATUS: u16 = 0x0036;
    pub const CHIP_ANA_TEST1: u16 = 0x0038;
    pub const CHIP_ANA_TEST2: u16 = 0x003A;
    pub const CHIP_SHORT_CTRL: u16 = 0x003C;
    pub const DAP_CONTROL: u16 = 0x0100;
    pub const DAP_PEQ: u16 = 0x0102;
    pub const DAP_BASS_ENHANCE: u16 = 0x0104;
    pub const DAP_BASS_ENHANCE_CTRL: u16 = 0x0106;
    pub const DAP_AUDIO_EQ: u16 = 0x0108;
    pub const DAP_SGTL_SURROUND: u16 = 0x010A;
    pub const DAP_FILTER_COEF_ACCESS: u16 = 0x010C;
    pub const DAP_COEF_WR_B0_MSB: u16 = 0x010E;
    pub const DAP_COEF_WR_B0_LSB: u16 = 0x0110;
    pub const DAP_AUDIO_EQ_BASS_BAND0: u16 = 0x0116;
    pub const DAP_AUDIO_EQ_BAND1: u16 = 0x0118;
    pub const DAP_AUDIO_EQ_BAND2: u16 = 0x011A;
    pub const DAP_AUDIO_EQ_BAND3: u16 = 0x011C;
    pub const DAP_AUDIO_EQ_TREBLE_BAND4: u16 = 0x011E;
    pub const DAP_MAIN_CHAN: u16 = 0x0120;
    pub const DAP_MIX_CHAN: u16 = 0x0122;
    pub const DAP_AVC_CTRL: u16 = 0x0124;
    pub const DAP_AVC_THRESHOLD: u16 = 0x0126;
    pub const DAP_AVC_ATTACK: u16 = 0x0128;
    pub const DAP_AVC_DECAY: u16 = 0x012A;
    pub const DAP_COEF_WR_B1_MSB: u16 = 0x012C;
    pub const DAP_COEF_WR_B1_LSB: u16 = 0x012E;
    pub const DAP_COEF_WR_B2_MSB: u16 = 0x0130;
    pub const DAP_COEF_WR_B2_LSB: u16 = 0x0132;
    pub const DAP_COEF_WR_A1_MSB: u16 = 0x0134;
    pub const DAP_COEF_WR_A1_LSB: u16 = 0x0136;
    pub const DAP_COEF_WR_A2_MSB: u16 = 0x0138;
    pub const DAP_COEF_WR_A2_LSB: u16 = 0x013A;
}

pub enum AdcInputSources {
    Microphone = 0,
    LineIn = 1,
}

pub enum HeadphoneInputSources {
    DAC = 0,
    LineIn = 1,
}

impl<I2C: I2c> SGTL5000<I2C> {
    /// Create new builder with a default I2C address of 0x0A (normal configuration)
    #[allow(clippy::new_ret_no_self)]
    pub fn new(i2c: I2C) -> Self {
        Self::new_custom_address(i2c, SGTL5000_I2C_ADDR_CS_LOW)
    }

    /// Create a new I2C interface with the alternate address 0x2A as specified in the datasheet (when CTRL_ADR0_CS is high).
    pub fn new_alternate_address(i2c: I2C) -> Self {
        Self::new_custom_address(i2c, SGTL5000_I2C_ADDR_CS_HIGH)
    }

    fn new_custom_address(i2c: I2C, address: u8) -> Self {
        SGTL5000 { i2c, address }
    }

    pub fn ready(&mut self) -> Result<bool, I2C::Error> {
        if self.get_chip_id()? == SGTL5000_CHIP_ID {
            return Ok(true);
        }
        Ok(false)
    }

    pub fn get_chip_id(&mut self) -> Result<u8, I2C::Error> {
        let chip_id_and_revision = self.read_configuration(CHIP_ID)?;
        Ok(chip_id_and_revision[0])
    }

    pub fn get_chip_revision(&mut self) -> Result<u8, I2C::Error> {
        let chip_id_and_revision = self.read_configuration(CHIP_ID)?;
        Ok(chip_id_and_revision[1])
    }

    pub fn init(&mut self) -> Result<bool, I2C::Error> {
        // VDDD is externally driven with 1.8V
        self.update_configuration(CHIP_ANA_POWER, 0x42, 0x60)?;
        // VDDA & VDDIO both over 3.1V
        self.update_configuration(CHIP_LINREG_CTRL, 0x00, 0x6C)?;
        // VAG=1.575, normal ramp, +12.5% bias current
        self.update_configuration(CHIP_REF_CTRL, 0x01, 0xF2)?;
        // LO_VAGCNTRL=1.65V, OUT_CURRENT=0.54mA
        self.update_configuration(CHIP_LINE_OUT_CTRL, 0x0F, 0x22)?;
        // allow up to 125mA
        self.update_configuration(CHIP_SHORT_CTRL, 0x44, 0x46)?;
        // enable zero cross detectors
        self.update_configuration(CHIP_ANA_CTRL, 0b0000_0001, 0b0011_0011)?;

        //SGTL is I2S Slave, so we can power up: lineout, hp, adc, dac
        self.update_configuration(CHIP_ANA_POWER, 0x6A, 0xFF)?;

        Ok(true)
    }

    pub fn power_up(&mut self) -> Result<bool, I2C::Error> {
        // power up all digital stuff
        self.update_configuration(CHIP_DIG_POWER, 0x00, 0x73)?;
        self.update_configuration(CHIP_LINE_OUT_VOL, 0x1D, 0x1D)?;

        // ADC --> IS2_OUT, I2S_IN --> DAC, ADC --> DAP (not enabled), ADC --> DAP MIXER (not enabled)
        self.update_configuration(CHIP_SSS_CTRL, 0b_00_00_00_00, 0b_00_01_00_00)?;

        // unmute dac
        self.update_configuration(CHIP_ADCDAC_CTRL, 0x00, 0x00)?;

        // digital gain, 0dB
        self.update_configuration(CHIP_DAC_VOL, 0x3C, 0x3C)?;

        // headphone volume (lowest level)
        self.update_configuration(CHIP_ANA_HP_CTRL, 0x7F, 0x7F)?;

        // enable analog with ZCD
        self.update_configuration(CHIP_ANA_CTRL, 0b0000_0000, 0b0000_0010)?;

        self.init_clock_and_i2s()?;
        Ok(true)
    }

    fn init_clock_and_i2s(&mut self) -> Result<bool, I2C::Error> {
        // 44.1 kHz, 256*Fs
        self.update_configuration(CHIP_CLK_CTRL, 0x00, 0x04)?;
        // SCLK=64*Fs, 16bit, I2S format
        self.update_configuration(CHIP_I2S_CTRL, 0x00, 0x30)?;
        Ok(true)
    }
    fn update_configuration(
        &mut self,
        register: u16,
        upper: u8,
        lower: u8,
    ) -> Result<bool, I2C::Error> {
        self.i2c.write(
            self.address,
            &[
                ((register >> 8) & 0xFF) as u8,
                (register & 0xFF) as u8,
                upper,
                lower,
            ],
        )?;
        Ok(true)
    }

    fn modify_configuration_bit(
        &mut self,
        register: u16,
        position: u8,
        value: bool,
    ) -> Result<bool, I2C::Error> {
        let mut analog_control_configuration = self.read_configuration(register)?;
        self.set_configuration_bit(&mut analog_control_configuration, position, value);
        self.update_configuration(
            register,
            analog_control_configuration[0],
            analog_control_configuration[1],
        )?;
        Ok(true)
    }

    fn read_configuration(&mut self, register: u16) -> Result<[u8; 2], I2C::Error> {
        let mut read_buffer: [u8; 2] = [0; 2];
        self.i2c.write_read(
            self.address,
            &[((register >> 8) & 0xFF) as u8, (register & 0xFF) as u8],
            &mut read_buffer,
        )?;
        Ok(read_buffer)
    }

    fn set_configuration_bit(&mut self, configuration: &mut [u8; 2], position: u8, value: bool) {
        if value {
            if position < 8 {
                configuration[1] = configuration[1] | 1 << position;
            } else {
                configuration[0] = configuration[0] | 1 << (position - 8);
            }
        } else {
            if position < 8 {
                configuration[1] = configuration[1] & !(1 << position);
            } else {
                configuration[1] = configuration[1] & !(1 << (position - 8));
            }
        }
    }

    /// volume 0 --> 127
    pub fn headphone_volume(&mut self, value: u8) -> Result<bool, I2C::Error> {
        let hp_volume = match value {
            0x00 => 0x7F,
            0x80..=u8::MAX => 0x00,
            0x01..=0x7F => 0x80 - value,
        };
        self.update_configuration(CHIP_ANA_HP_CTRL, hp_volume, hp_volume)?;
        Ok(true)
    }

    pub fn mute_headphone(&mut self) -> Result<bool, I2C::Error> {
        self.modify_configuration_bit(CHIP_ANA_CTRL, CHIP_ANA_CTRL_MUTE_HP_BIT, true)?;
        Ok(true)
    }

    pub fn unmute_headphone(&mut self) -> Result<bool, I2C::Error> {
        self.modify_configuration_bit(CHIP_ANA_CTRL, CHIP_ANA_CTRL_MUTE_HP_BIT, false)?;
        Ok(true)
    }

    pub fn mute_lineout(&mut self) -> Result<bool, I2C::Error> {
        self.modify_configuration_bit(CHIP_ANA_CTRL, CHIP_ANA_CTRL_MUTE_LO_BIT, true)?;
        Ok(true)
    }

    pub fn unmute_linout(&mut self) -> Result<bool, I2C::Error> {
        self.modify_configuration_bit(CHIP_ANA_CTRL, CHIP_ANA_CTRL_MUTE_LO_BIT, false)?;
        Ok(true)
    }

    pub fn select_adc_input(&mut self, input: AdcInputSources) -> Result<bool, I2C::Error> {
        self.modify_configuration_bit(
            CHIP_ANA_CTRL,
            CHIP_ANA_CTRL_SELECT_ADC_BIT,
            (input as u8) == 1,
        )?;
        Ok(true)
    }
    pub fn select_headphone_input(
        &mut self,
        input: HeadphoneInputSources,
    ) -> Result<bool, I2C::Error> {
        self.modify_configuration_bit(
            CHIP_ANA_CTRL,
            CHIP_ANA_CTRL_SELECT_HP_INPUT_BIT,
            (input as u8) == 1,
        )?;
        Ok(true)
    }

    // 0, 10, 20, 30 DB (0, 10, 100, 1000 x amplification)
    pub fn set_microphone_gain(&mut self, db: u8) -> Result<bool, I2C::Error> {
        let mut _set_db_level = 0x0;
        if db <= 30 {
            _set_db_level = db / 10;
        }

        //fixed bias resistor = 2KOhm, ,biasvolt = 3v, variable pre-gain.
        //// mic on board =   self.update_configuration(CHIP_MIC_CTRL,0b00_00_00_11 , 0b00_10_00_10)?;
        //self.update_configuration(CHIP_MIC_CTRL,0b00_00_00_11 , 0b00_10_00_10)?;
        self.update_configuration(CHIP_MIC_CTRL, 0b00_00_00_11, 0b01_11_00_10)?;
        Ok(true)
    }

    pub fn enable_audio_processing(&mut self) -> Result<bool, I2C::Error> {
        // DAP --> IS2_DOUT,

        self.update_configuration(CHIP_SSS_CTRL, 0b00_00_00_00, 0b01_11_00_00)?;
        self.update_configuration(DAP_CONTROL, 0x00, 0b00_0_0_000_1)?;

        //        self.update_configuration(DAP_AVC_CTRL, 0b00_01_00_01, 0b00_0_0000_1)?;
        Ok(true)
    }

    pub fn enable_bass_enhance(&mut self) -> Result<bool, I2C::Error> {
        self.update_configuration(DAP_BASS_ENHANCE, 0b0000000_0, 0b0_111_000_1)?;
        self.update_configuration(DAP_BASS_ENHANCE_CTRL, 0b0_011_1111, 0b0_0000111)?;
        Ok(true)
    }

    pub fn disable_audio_processing(&mut self) -> Result<bool, I2C::Error> {
        self.update_configuration(DAP_CONTROL, 0x00, 0x00)?;
        self.update_configuration(CHIP_SSS_CTRL, 0x00, 0x10)?;

        Ok(true)
    }
}
