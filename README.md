Simple example using slint on a RP2040

```
#![no_std]
#![cfg_attr(not(feature = "simulator"), no_main)]

extern crate alloc;
use gc9a01::GC9A01;
use embedded_hal::digital::v2::OutputPin;

slint::include_modules!();

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    ui
}

#[cfg(feature = "simulator")]
fn main() -> Result<(), slint::PlatformError> {
    create_slint_app().run()
}

slint::slint!{
export global Logic := {
    property <int> value;
}
}

#[cfg(not(feature = "simulator"))]
#[rp_pico::entry]
fn main() -> ! {
    // Pull in any important traits
    use fugit::RateExtU32;
    use panic_halt as _;
    use rp_pico::hal;
    use rp_pico::hal::pac;
    use rp_pico::hal::prelude::*;
    use slint::platform::WindowEvent;

    // -------- Setup Allocator --------
    const HEAP_SIZE: usize = 200 * 1024;
    static mut HEAP: [u8; HEAP_SIZE] = [0; HEAP_SIZE];
    #[global_allocator]
    static ALLOCATOR: embedded_alloc::Heap = embedded_alloc::Heap::empty();
    unsafe { ALLOCATOR.init(&mut HEAP as *const u8 as usize, core::mem::size_of_val(&HEAP)) };

    // -------- Setup peripherials --------
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().raw());

    let sio = hal::sio::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);

    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);

    let _spi_sclk = pins.gpio10.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio11.into_mode::<hal::gpio::FunctionSpi>();

    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI1);
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        8.MHz(),
        &embedded_hal::spi::MODE_3,
    );

    let mut bl = pins.gpio16.into_push_pull_output();
    let mut rst = pins.gpio12.into_push_pull_output();
    let dc = pins.gpio8.into_push_pull_output();
    let cs = pins.gpio9.into_push_pull_output();

    // Initialize display
    bl.set_high().unwrap();
    rst.set_low().unwrap();
    delay.delay_ms(100);
    rst.set_high().unwrap();
    delay.delay_ms(100);

    let mut display = GC9A01 { spi, cs, dc };
    display.setup();

    // -------- Setup the Slint backend --------
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(alloc::boxed::Box::new(MyPlatform {
        window: window.clone(),
        timer,
    }))
    .unwrap();

    struct MyPlatform {
        window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
        timer: hal::Timer,
    }

    impl slint::platform::Platform for MyPlatform {
        fn create_window_adapter(&self) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
            Ok(self.window.clone())
        }
        fn duration_since_start(&self) -> core::time::Duration {
            core::time::Duration::from_micros(self.timer.get_counter())
        }
    }

    // -------- Configure the UI --------
    // (need to be done after the call to slint::platform::set_platform)
    let _ui = create_slint_app();
    _ui.global::<Logic>().set_counter(29);

    // -------- Event loop --------
    let mut line = [slint::platform::software_renderer::Rgb565Pixel(0); 320];
    loop {
        slint::platform::update_timers_and_animations();
        window.draw_if_needed(|renderer| {
            use embedded_graphics_core::prelude::*;
            struct DisplayWrapper<'a, T>(
                &'a mut T,
                &'a mut [slint::platform::software_renderer::Rgb565Pixel],
            );
            impl<T: DrawTarget<Color = embedded_graphics_core::pixelcolor::Rgb565>>
                slint::platform::software_renderer::LineBufferProvider for DisplayWrapper<'_, T>
            {
                type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;
                fn process_line(
                    &mut self,
                    line: usize,
                    range: core::ops::Range<usize>,
                    render_fn: impl FnOnce(&mut [Self::TargetPixel]),
                ) {
                    let rect = embedded_graphics_core::primitives::Rectangle::new(
                        Point::new(range.start as _, line as _),
                        Size::new(range.len() as _, 1),
                    );
                    render_fn(&mut self.1[range.clone()]);
                    self.0
                        .fill_contiguous(
                            &rect,
                            self.1[range.clone()].iter().map(|p| {
                                embedded_graphics_core::pixelcolor::raw::RawU16::new(p.0).into()
                            }),
                        )
                        .map_err(drop)
                        .unwrap();
                }
            }
            renderer.render_by_line(DisplayWrapper(&mut display, &mut line));
        });

        if window.has_active_animations() {
            continue;
        }


    }
}

```

