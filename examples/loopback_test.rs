#![no_std]
#![no_main]

use driver_sgtl5000::SGTL5000;
use esp_backtrace as _;
use esp_hal::i2s::master::Channels;
use esp_hal::rtc_cntl::Rtc;
use esp_hal::time::Rate;
use esp_hal::{
    delay::Delay,
    dma_circular_buffers,
    gpio::NoPin,
    i2c::master::I2c,
    i2s::master::{DataFormat, I2s},
    xtensa_lx_rt::entry,
};
use esp_println::{print, println};

#[entry]
fn main() -> ! {
    const SAMPLE_RATE: u32 = 44100;
    const AMOUNT_OF_SAMPLES_BUFFERING_TX: usize = 1100;
    const BUFFER_SIZE_TX: usize = 4 * AMOUNT_OF_SAMPLES_BUFFERING_TX;
    const BUFFER_SIZE_RX: usize = 1100;

    // init CPU
    let mut config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::_240MHz);
    let peripherals = esp_hal::init(config);

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) =
        dma_circular_buffers!(BUFFER_SIZE_RX, BUFFER_SIZE_TX);

    //
    let dma_channel = peripherals.DMA_CH0;

    let mut i2s = I2s::new(
        peripherals.I2S0,
        dma_channel,
        esp_hal::i2s::master::Config::new_tdm_philips()
            .with_sample_rate(Rate::from_hz(44100))
            .with_data_format(DataFormat::Data16Channel16)
            .with_channels(Channels::STEREO),
    )
    .unwrap();

    i2s = i2s.with_mclk(peripherals.GPIO42);

    let mut i2s_tx = i2s
        .i2s_tx
        .with_bclk(peripherals.GPIO40)
        .with_ws(peripherals.GPIO41)
        .with_dout(peripherals.GPIO38)
        .build(tx_descriptors);
    //

    let mut i2s_rx = i2s
        .i2s_rx
        .with_bclk(NoPin)
        .with_ws(NoPin)
        .with_din(peripherals.GPIO39)
        .build(rx_descriptors);

    let mut config = esp_hal::i2c::master::Config::default();
    let mut i2c = I2c::new(peripherals.I2C0, config)
        .unwrap()
        .with_sda(peripherals.GPIO47)
        .with_scl(peripherals.GPIO48);

    let mut sgtl500 = SGTL5000::new(i2c);

    println!("Check if he SGTL5000 is ready and respond to commands\n");
    println!("--> SGTL5000 :: Chip Ready {:?}", sgtl500.ready().unwrap());

    println!("Initialize the SGTL5000\n");
    println!(
        "--> SGTL5000 :: Chip Revision {:#x}",
        sgtl500.get_chip_revision().unwrap()
    );
    println!("--> SGTL5000 :: Codec setup {:?}", sgtl500.init());
    sgtl500.power_up();
    sgtl500.set_microphone_gain(0);
    sgtl500.select_adc_input(driver_sgtl5000::AdcInputSources::Microphone);
    sgtl500.headphone_volume(50);

    println!("start!");
    sgtl500.disable_audio_processing();



    //let mut transfer = i2s_tx.write_dma_circular(tx_buffer).unwrap();
    println!("receiver buffer {:?}", rx_buffer.len());
    let mut receiver = i2s_rx.read_dma(rx_buffer).unwrap();
       let result = receiver.wait();

     //let mut audio_sample = include_bytes!("sample.raw");
    let mut buffer_transfer = [0u8; BUFFER_SIZE_TX];

    let mut audio_sample = [0u8; BUFFER_SIZE_RX];

    let mut counter = 0;
    let mut counter2 = 0;
    let mut stop = false;
    println!("start {:?}", "done");



   // match result {
     //   Ok(_) => println!("Dome"),
       // Err(e) => println!(" Error {:?}", e)
  //  }
    println!("receiver buffer {:?}", rx_buffer);

    loop {
        
     /*    match receiver.available() {
            Ok(num_bytes) => {
                if (num_bytes > 0) {
                    receiver.pop(&mut buffer_receiver);

                    // Calculate how many complete 4-byte chunks we can process
                    let chunks_to_process = num_bytes / 4;
                    let remaining_bytes = num_bytes % 4;
                    if remaining_bytes > 0 {
                        println!("Warning: Received {} incomplete bytes", remaining_bytes);
                    }

                    if counter2 >= audio_sample.len() - (chunks_to_process * 4) {
                        counter2 = 0;
                        stop = false;
                    }

                    if !stop {
                        for i in 0..chunks_to_process {
                            // Store 4 bytes (2 bytes per channel)
                            let byte_index = counter2 + (i * 4);

                            // Channel 1 (first 2 bytes)
                            audio_sample[byte_index] = buffer_receiver[i * 4];
                            audio_sample[byte_index + 1] = buffer_receiver[i * 4 + 1];

                            // Channel 2 (next 2 bytes)
                            audio_sample[byte_index + 2] = buffer_receiver[i * 4 + 2];
                            audio_sample[byte_index + 3] = buffer_receiver[i * 4 + 3];
                        }
                        counter2 = counter2 + (chunks_to_process * 4);
                        // Handle remaining bytes (0-3 bytes)
                        /*   if remaining_bytes > 0 {
                            println!("Warning: Received {} incomplete bytes", remaining_bytes);

                            // Option 1: Store remaining bytes and combine with next chunk
                            if counter2 + remaining_bytes <= audio_sample.len() {
                                for i in 0..remaining_bytes {
                                    audio_sample[counter2 + i] = buffer_receiver[chunks_to_process * 4 + i];
                                }
                                counter2 += remaining_bytes;
                            }
                            // Option 2: Drop the incomplete chunk
                            // (No action needed - just don't store the remaining bytes)
                        }*/
                    }
                }
            }
            Err(error) => {
                println!("RX Error: {:?}", error);
            }
        }
        */
        /* 
        match transfer.available() {
            Ok(num_bytes) => {
                if (num_bytes > 0) {
                    let avail = usize::min(BUFFER_SIZE_TX, num_bytes);
                    for i in 0..(avail / 4) {
                        // left
                        buffer_transfer[i * 4] = rx_buffer[counter * 4];
                        buffer_transfer[i * 4 + 1] = rx_buffer[counter * 4 + 1];
                        //right
                        buffer_transfer[i * 4 + 2] = rx_buffer[counter * 4 + 2];
                        buffer_transfer[i * 4 + 3] = rx_buffer[counter * 4 + 3];
                        counter = counter + 1;
                        if counter * 4 >= rx_buffer.len() {
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
        */
    }
}
