#![no_std]
#![no_main]

use driver_sgtl5000::SGTL5000;
use esp_backtrace as _;
use esp_hal::{
    delay::Delay,dma_circular_buffers, gpio::NoPin, i2c::master::{Config, I2c}, i2s::master::{DataFormat, I2s, Standard}, peripheral::{self, Peripheral}, time::RateExtU32, xtensa_lx_rt::entry
};
use esp_hal::{rtc_cntl::Rtc};
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    const SAMPLE_RATE: u32 = 44100;
    const AMOUNT_OF_SAMPLES_BUFFERING: usize = 1100;
    const BUFFER_SIZE: usize = 4 * AMOUNT_OF_SAMPLES_BUFFERING;

    // init CPU
    let mut config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::_240MHz);
    let peripherals = esp_hal::init(config);
    let rtc = Rtc::new(peripherals.LPWR);

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
        dma_circular_buffers!(BUFFER_SIZE, BUFFER_SIZE);

    let dma_channel = peripherals.DMA_CH0;

    let mut i2s = I2s::new(
        peripherals.I2S0,
        Standard::Philips,
        DataFormat::Data16Channel16,
        SAMPLE_RATE.Hz(),
        dma_channel,
        rx_descriptors,
        tx_descriptors,
    );

    i2s = i2s.with_mclk(peripherals.GPIO38);
    let mut i2s_tx = i2s
        .i2s_tx
        .with_bclk(peripherals.GPIO40)
        .with_ws(peripherals.GPIO39)
        .with_dout(peripherals.GPIO42)
        .build();


    let mut i2s_rx = i2s.i2s_rx
        .with_bclk(NoPin)
        .with_ws(NoPin)
        .with_din(peripherals.GPIO41)
        .build();

    let mut config = Config::default();
    let mut i2c = I2c::new(peripherals.I2C0, config).unwrap()
    .with_sda(peripherals.GPIO47)
            .with_scl(peripherals.GPIO48);

    let mut sgtl500 = SGTL5000::new(i2c);
    
    println!("Check if the SGTL5000 is ready and respond to commands\n");
    println!(
        "--> SGTL5000 :: Chip Ready {:?}",
        sgtl500.ready().unwrap()
    );

    println!("Initialize the SGTL5000\n");
    println!(
        "--> SGTL5000 :: Chip Revision {:#x}",
        sgtl500.get_chip_revision().unwrap()
    );
    println!("--> SGTL5000 :: Codec setup {:?}", sgtl500.init());
    Delay::new().delay_millis(100);
    sgtl500.power_up();

    Delay::new().delay_millis(1);
    sgtl500.set_microphone_gain(0);
    Delay::new().delay_millis(1);
    sgtl500.select_adc_input(driver_sgtl5000::AdcInputSources::Microphone);
    Delay::new().delay_millis(1);
   
    
    println!(
        "--> SGTL5000 ::Volume to 40 {:?}",
        sgtl500.headphone_volume(70)
    );
   
    Delay::new().delay_millis(1);
    println!("start!");
    //sgtl500.disable_audio_processing();
    //sgtl500.enable_bass_enhance();

    Delay::new().delay_millis(1000);

    let mut transfer = i2s_tx.write_dma_circular(tx_buffer).unwrap();
    let mut receiver = i2s_rx.read_dma_circular(rx_buffer).unwrap();
   let mut audio_sample = [0u8; SAMPLE_RATE as usize * 4];
  //  let mut audio_sample = include_bytes!("sample.raw");
    let mut buffer_receiver = [0u8; BUFFER_SIZE];
    let mut buffer_transfer = [0u8; BUFFER_SIZE];
    


    let mut counter = 0;
    let mut counter2 = 0;
    let mut stop = false;
    loop {
        match transfer.available() {
            Ok(num_bytes) => {
                if (num_bytes > 0) {
                    let avail = usize::min(BUFFER_SIZE, num_bytes);
                    for i in 0..(avail / 4) {
                       // left
                        buffer_transfer[i * 4] = audio_sample[counter * 4];
                        buffer_transfer[i * 4 + 1] = audio_sample[counter * 4 + 1];
                        //right
                        buffer_transfer[i * 4 + 2] = audio_sample[counter * 4 + 2];
                        buffer_transfer[i * 4 + 3] = audio_sample[counter * 4 + 3];
                        counter = counter + 1;
                        if counter * 4 >= audio_sample.len() {
                            counter = 0;
                        }
                    }
                   // transfer.push(&buffer_transfer[0..avail]);
                   transfer.push(&buffer_transfer[0..avail]);
                }
            }
            Err(error) => {
                println!("TX Error: {:?}", error);
            }
        }

        match receiver.available() {
            Ok(num_bytes) => {
                if (num_bytes > 0) {
                   
                    receiver.pop(&mut buffer_receiver);
                    
                    if counter2 >= audio_sample.len() - num_bytes {
                        counter2 = 0;
                         stop = false;
                    }
                    if !stop {
                        for i in 0..(num_bytes) {
                            audio_sample[counter2 + i] = buffer_receiver[i];
                        }
                        counter2 = counter2 + num_bytes;
                 }   

                } 
            }
            Err(error) => {
                println!("RX Error: {:?}", error);
            }
        }


    }
}
