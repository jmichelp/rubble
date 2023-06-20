#![no_std]
#![no_main]

// We need to import this crate explicitly so we have a panic handler
use panic_probe as _;

mod attrs;
mod logger;

// Import the right HAL/PAC crate, depending on the target chip
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

#[rtic::app(device = crate::hal::pac, peripherals = true, dispatchers = [WDT])]
mod app {
    use core::mem::MaybeUninit;

    use rubble::{
        config::Config,
        l2cap::{BleChannelMap, L2CAPState},
        link::{
            ad_structure::AdStructure,
            queue::{PacketQueue, SimpleQueue},
            LinkLayer, Responder, MIN_PDU_BUF,
        },
        security::NoSecurity,
        time::{Duration, Timer},
    };
    use rubble_nrf5x::{
        radio::{BleRadio, PacketBuffer},
        timer::BleTimer,
        utils::get_device_address,
    };
    
    use bbqueue::Consumer;
    use core::sync::atomic::{compiler_fence, Ordering};
    use crate::hal::gpio::Level;
    use rtt_target::{rtt_init, UpChannel};
    use crate::logger::BUFFER_SIZE;

    pub enum AppConfig {}

    impl Config for AppConfig {
        type Timer = BleTimer<crate::hal::pac::TIMER0>;
        type Transmitter = BleRadio;
        type ChannelMapper = BleChannelMap<crate::attrs::DemoAttrs, NoSecurity>;
        type PacketQueue = &'static mut SimpleQueue;
    }

    #[local]
    struct Local {
        ble_r: Responder<AppConfig>,
        log_channel: UpChannel,
        log_sink: Consumer<'static, BUFFER_SIZE>,
    }

    #[shared]
    struct Shared {
        ble_ll: LinkLayer<AppConfig>,
        radio: BleRadio,
    }

    #[init(local = [
        tx_queue: SimpleQueue = SimpleQueue::new(),
        rx_queue: SimpleQueue = SimpleQueue::new(),
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
        let log_channel = rtt.up.0;

        // On reset, the internal high frequency clock is already used, but we
        // also need to switch to the external HF oscillator. This is needed
        // for Bluetooth to work.
        let _clocks = crate::hal::clocks::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();

        let ble_timer = BleTimer::init(ctx.device.TIMER0);

        let p0 = crate::hal::gpio::p0::Parts::new(ctx.device.P0);

        // Determine device address
        let device_address = get_device_address();

        let ble_rx_buf: &'static mut _ = ctx.local.rx_buf.write([0; MIN_PDU_BUF]);
        let ble_tx_buf: &'static mut _ = ctx.local.tx_buf.write([0; MIN_PDU_BUF]);
        let mut radio = BleRadio::new(
            ctx.device.RADIO,
            &ctx.device.FICR,
            ble_tx_buf,
            ble_rx_buf,
        );

        let log_sink = crate::logger::init(ble_timer.create_stamp_source());

        // Create TX/RX queues
        let (tx, tx_cons) = ctx.local.tx_queue.split();
        let (rx_prod, rx) = ctx.local.rx_queue.split();

        // Create the actual BLE stack objects
        let mut ble_ll = LinkLayer::<AppConfig>::new(device_address, ble_timer);

        // Assumes pin 17 corresponds to an LED.
        // On the NRF52DK board, this is LED 1.
        let ble_r = Responder::new(
            tx,
            rx,
            L2CAPState::new(BleChannelMap::with_attributes(crate::attrs::DemoAttrs::new(
                p0.p0_17.into_push_pull_output(Level::High).degrade(),
            ))),
        );

        // Send advertisement and set up regular interrupt
        let next_update = ble_ll
            .start_advertise(
                Duration::millis(200),
                &[AdStructure::CompleteLocalName("CONCVRRENS CERTA CELERIS")],
                &mut radio,
                tx_cons,
                rx_prod,
            )
            .unwrap();

        ble_ll.timer().configure_interrupt(next_update);

        (Shared {
            radio,
            ble_ll,
        }, Local {
            ble_r,
            log_channel,
            log_sink
        }, init::Monotonics())
    }

    #[task(binds = RADIO, shared = [radio, ble_ll], priority = 3)]
    fn radio(ctx: radio::Context) {
        let ble_ll = ctx.shared.ble_ll;
        let radio = ctx.shared.radio;
        (radio, ble_ll).lock(|radio, ble_ll| {
            if let Some(cmd) = radio.recv_interrupt(ble_ll.timer().now(), ble_ll) {
                radio.configure_receiver(cmd.radio);
                ble_ll.timer().configure_interrupt(cmd.next_update);

                if cmd.queued_work {
                    // If there's any lower-priority work to be done, ensure that happens.
                    // If we fail to spawn the task, it's already scheduled.
                    ble_worker::spawn().unwrap();
                }
            }
        });
    }

    #[task(binds = TIMER0, shared = [radio, ble_ll], priority = 3)]
    fn timer0(ctx: timer0::Context) {
        let radio = ctx.shared.radio;
        let ble_ll = ctx.shared.ble_ll;
        (radio, ble_ll).lock(|radio, ble_ll| {
            let timer = ble_ll.timer();
            if !timer.is_interrupt_pending() {
                return;
            }
            timer.clear_interrupt();
    
            let cmd = ble_ll.update_timer(radio);
            radio.configure_receiver(cmd.radio);
            ble_ll.timer().configure_interrupt(cmd.next_update);    

            if cmd.queued_work {
                // If there's any lower-priority work to be done, ensure that happens.
                // If we fail to spawn the task, it's already scheduled.
                ble_worker::spawn().unwrap();
            }
            });
    }

    #[idle(local = [log_sink, log_channel])]
    fn idle(ctx: idle::Context) -> ! {
        // Drain the logging buffer through the serial connection
        loop {
            if cfg!(feature = "log") {
                while let Ok(grant) = ctx.local.log_sink.read() {
                    ctx.local.log_channel.write(grant.buf());

                    let len = grant.buf().len();
                    grant.release(len);
                }
            } else {
                // Work around https://github.com/rust-lang/rust/issues/28728
                compiler_fence(Ordering::SeqCst);
            }
        }
    }

    #[task(local = [ble_r], priority = 2)]
    fn ble_worker(ctx: ble_worker::Context) {
        // Fully drain the packet queue
        while ctx.local.ble_r.has_work() {
            ctx.local.ble_r.process_one().unwrap();
        }
    }

}
