cargo build --release --example loopback_test
espflash flash --monitor  target/xtensa-esp32s3-none-elf/release/examples/loopback_test
