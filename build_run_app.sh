cargo build --release --example stream_audio_file
espflash flash --monitor  target/xtensa-esp32s3-none-elf/release/examples/stream_audio_file
