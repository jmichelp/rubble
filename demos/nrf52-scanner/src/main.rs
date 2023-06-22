#![no_std]
#![no_main]

// We need to import this crate explicitly so we have a panic handler
use panic_rtt_target as _;

// Import the right HAL and PAC
#[cfg(feature = "52810")]
use nrf52810_hal as hal;
#[cfg(feature = "52811")]
use nrf52811_hal as hal;
#[cfg(feature = "52832")]
use nrf52832_hal as hal;
#[cfg(feature = "52833")]
use nrf52833_hal as hal;
#[cfg(feature = "52840")]
use nrf52840_hal as hal;

#[rtic::app(device = crate::hal::pac, peripherals = true)]
mod app {
    use core::mem::MaybeUninit;
    use rtt_target::{rprint, rprintln, rtt_init, set_print_channel};
    use rubble::beacon::{BeaconScanner, ScanCallback};
    use rubble::link::Metadata;
    use rubble::link::{ad_structure::AdStructure, filter::AllowAll, DeviceAddress, MIN_PDU_BUF};
    use rubble::time::{Duration, Timer};
    use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
    use rubble_nrf5x::timer::BleTimer;

    pub struct BeaconScanCallback;

    impl ScanCallback for BeaconScanCallback {
        fn beacon<'a, I>(&mut self, addr: DeviceAddress, data: I, metadata: Metadata)
        where
            I: Iterator<Item = AdStructure<'a>>,
        {
            rprint!("[{:?} ", metadata.timestamp.unwrap().ticks());
            if let Some(rssi) = metadata.rssi {
                rprint!("RSSI:{:?}dBm ", rssi);
            }
            rprint!("BDADDR:{:?} DATA:", addr);
            let mut first = true;
            for packet in data {
                rprint!("{}{:02x?}", if first { " " } else { " / " }, packet);
                first = false;
            }
            rprintln!("");
        }
    }

    #[shared]
    struct Shared {
        radio: BleRadio,
        ble_timer: BleTimer<crate::hal::pac::TIMER0>,
        scanner: BeaconScanner<BeaconScanCallback, AllowAll>,
    }

    #[local]
    struct Local {}

    #[init(local=[
        tx_buf: MaybeUninit<PacketBuffer> = MaybeUninit::uninit(),
        rx_buf: MaybeUninit<PacketBuffer> = MaybeUninit::uninit()
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rtt = rtt_init! {
            up: {
                0: {
                    size: 1024
                    mode: NoBlockTrim
                    name: "Rubble Logs"
                }
            }
        };
        set_print_channel(rtt.up.0);

        // On reset, the internal high frequency clock is already used, but we
        // also need to switch to the external HF oscillator. This is needed
        // for Bluetooth to work.
        let _clocks = crate::hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();

        // Initialize BLE timer
        let mut ble_timer = BleTimer::init(ctx.device.TIMER0);

        // Initialize radio
        let ble_rx_buf: &'static mut _ = ctx.local.rx_buf.write([0; MIN_PDU_BUF]);
        let ble_tx_buf: &'static mut _ = ctx.local.tx_buf.write([0; MIN_PDU_BUF]);
        let mut radio = BleRadio::new(ctx.device.RADIO, &ctx.device.FICR, ble_tx_buf, ble_rx_buf);

        // Set up beacon scanner for continuous scanning. The advertisement
        // channel that is being listened on (scan window) will be switched
        // every 500 ms.
        let mut scanner = BeaconScanner::new(BeaconScanCallback);
        let scanner_cmd = scanner.configure(ble_timer.now(), Duration::millis(500));

        // Reconfigure radio and timer
        radio.configure_receiver(scanner_cmd.radio);
        ble_timer.configure_interrupt(scanner_cmd.next_update);

        rprintln!("nRF52 scanner ready!");

        (
            Shared {
                radio,
                scanner,
                ble_timer,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(binds = RADIO, shared = [radio, scanner, ble_timer])]
    fn radio(ctx: radio::Context) {
        let timer = ctx.shared.ble_timer;
        let scanner = ctx.shared.scanner;
        let radio = ctx.shared.radio;

        (timer, scanner, radio).lock(|timer, scanner, radio| {
            if let Some(next_update) = radio.recv_beacon_interrupt(timer.now(), scanner) {
                timer.configure_interrupt(next_update);
            }
        });
    }

    #[task(binds = TIMER0, shared = [radio, ble_timer, scanner])]
    fn timer0(ctx: timer0::Context) {
        let timer = ctx.shared.ble_timer;
        let scanner = ctx.shared.scanner;
        let radio = ctx.shared.radio;

        (timer, scanner, radio).lock(|timer, scanner, radio| {
            // Clear interrupt
            if !timer.is_interrupt_pending() {
                return;
            }
            timer.clear_interrupt();

            // Update scanner (switch to next advertisement channel)
            let cmd = scanner.timer_update(timer.now());
            radio.configure_receiver(cmd.radio);
            timer.configure_interrupt(cmd.next_update);
        });
    }
}
