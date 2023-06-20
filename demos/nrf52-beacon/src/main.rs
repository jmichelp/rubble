#![no_std]
#![no_main]

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

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [TIMER1])]
mod app {
    // We need to import this crate explicitly so we have a panic handler
    use panic_halt as _;
    use core::mem::MaybeUninit;
    use rubble::beacon::Beacon;
    use rubble::link::{ad_structure::AdStructure, MIN_PDU_BUF};
    use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
    use rubble_nrf5x::utils::get_device_address;
    use dwt_systick_monotonic::*;

    #[monotonic(binds = DwtSystick, default = true)]
    type MyMono = DwtSystick<1000>;  // 1kHz = 1ms granularity

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        ble_tx_buf: &'static mut PacketBuffer,
        ble_rx_buf: &'static mut PacketBuffer,
        radio: BleRadio,
        beacon: Beacon,
    }

    #[init(local=[
        tx_buf: MaybeUninit<PacketBuffer> = MaybeUninit::uninit(),
        rx_buf: MaybeUninit<PacketBuffer> = MaybeUninit::uninit()
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // On reset, the internal high frequency clock is already used, but we
        // also need to switch to the external HF oscillator. This is needed
        // for Bluetooth to work.
        let _clocks = crate::hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();

        // Initialize (enable) the monotonic timer (CYCCNT)
        let mut core = ctx.core;
        core.DCB.enable_trace();
        core.DWT.enable_cycle_counter();

        let mono = DwtSystick::new(core.DCB, core.DWT, core.SYST, 64_000_000);

        // Determine device address
        let device_address = get_device_address();

        // Rubble currently requires an RX buffer even though the radio is only used as a TX-only beacon.
        let ble_rx_buf: &'static mut _ = ctx.local.rx_buf.write([0; MIN_PDU_BUF]);
        let ble_tx_buf: &'static mut _ = ctx.local.tx_buf.write([0; MIN_PDU_BUF]);
        let radio = BleRadio::new(
            ctx.device.RADIO,
            &ctx.device.FICR,
            ble_tx_buf,
            ble_rx_buf,
        );

        let beacon = Beacon::new(
            device_address,
            &[AdStructure::CompleteLocalName("Rusty Beacon (nRF52)")],
        )
        .unwrap();

        update::spawn_after(1.secs()).unwrap();

        (Shared {}, Local {
            ble_tx_buf,
            ble_rx_buf,
            radio,
            beacon
        }, init::Monotonics(mono))
    }

    /// Fire the beacon.
    #[task(local = [radio, beacon])]
    fn update(ctx: update::Context) {
        let beacon = ctx.shared.beacon;
        let radio = ctx.shared.radio;
        (beacon, radio).lock(|beacon, radio| {beacon.broadcast(radio)});

        update::spawn_after(333.millis()).unwrap(); // about 3 times per second
    }
}
