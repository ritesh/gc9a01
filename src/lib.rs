#![no_std]
#![no_main]

use embedded_graphics::{
    pixelcolor::{raw::RawU16, Rgb565, RgbColor},
    prelude::*,
};

use embedded_hal::digital::v2::{InputPin, OutputPin};
use embedded_hal::blocking::spi;

#[derive(Debug)]
pub struct CommError;

enum GC9A01Kind {
    GC9A01_START,
    GC9A01_CMD,
    GC9A01_DATA,
    GC9A01_DELAY,
    GC9A01_END
}

struct GC9A01Step {
    state: GC9A01Kind,
    value: u8
}

const  GC9A01_INVOFF: u8 = 0x20;
const  GC9A01_INVONL: u8 = 0x21;
const  GC9A01_DISPON: u8 = 0x29;
const  GC9A01_CASET: u8 = 0x2A;
const  GC9A01_RASET: u8 = 0x2B;
const  GC9A01_RAMWR: u8 = 0x2C;
const  GC9A01_COLMOD: u8 = 0x3A;
const  GC9A01_MADCTL: u8 = 0x36;
const  GC9A01_MADCTL_MY: u8 = 0x80;
const  GC9A01_MADCTL_MX: u8 = 0x40;
const  GC9A01_MADCTL_MV: u8 = 0x20;
const  GC9A01_MADCTL_RGB: u8 = 0x00;
const  GC9A01_DISFNCTRL: u8 = 0xB6;

const gc9a01_config: [GC9A01Step; 188] =
[
    GC9A01Step{ state: GC9A01Kind::GC9A01_START, value: 0x0},
    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xEF},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xEB},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x14},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xFE}, // Inter Register Enable1
    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xEF}, // Inter Register Enable2

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xEB},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x14},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x84},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x40},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x85},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xFF},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x86},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xFF},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x87},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xFF},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x88},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x0A},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x89},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x21},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x8A},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x8B},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x80},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x8C},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x01},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x8D},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x01},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x8E},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xFF},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x8F},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xFF},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: GC9A01_DISFNCTRL}, // Display Function Control
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: GC9A01_MADCTL}, // Memory Access Control
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x48}, // Set the display direction 0,1,2,3	four directions

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: GC9A01_COLMOD}, // COLMOD: Pixel Format Set
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x05}, // 16 Bits per pixel

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x90},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x08},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x08},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x08},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x08},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xBD},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x06},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xBC},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xFF},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x60},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x01},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x04},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xC3}, // Power Control 2
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x13},
    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xC4}, // Power Control 3
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x13},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xC9}, // Power Control 4
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x22},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xBE},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x11},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xE1},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x10},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x0E},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xDF},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x21},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x0C},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x02},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xF0}, // SET_GAMMA1
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x45},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x09},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x08},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x08},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x26},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x2A},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xF1}, // SET_GAMMA2
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x43},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x72},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x36},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x37},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x6F},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xF2}, // SET_GAMMA3
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x45},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x09},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x08},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x08},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x26},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x2A},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xF3}, // SET_GAMMA4
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x43},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x72},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x36},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x37},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x6F},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xED},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x1B},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x0B},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xAE},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x77},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xCD},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x63},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x70},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x07},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x07},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x04},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x0E},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x0F},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x09},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x07},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x08},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x03},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0xE8},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x34},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x62},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x18},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x0D},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x71},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xED},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x18},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x0F},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x71},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xEF},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x63},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x18},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x11},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x71},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xF1},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x18},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x13},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x71},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xF3},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x70},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x64},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x28},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x29},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xF1},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x01},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xF1},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x07},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x66},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x3C},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0xCD},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x67},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x45},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x45},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x10},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x67},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x3C},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x01},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x54},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x10},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x32},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x98},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x74},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x10},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x85},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x80},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x4E},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x00},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x98},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x3E},
    GC9A01Step{ state: GC9A01Kind::GC9A01_DATA, value: 0x07},

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x35}, // Tearing Effect Line ON
    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x21}, // Display Inversion ON

    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: 0x11}, // Sleep Out Mode
    GC9A01Step{ state: GC9A01Kind::GC9A01_DELAY, value: 120},
    GC9A01Step{ state: GC9A01Kind::GC9A01_CMD, value: GC9A01_DISPON}, // Display ON
    GC9A01Step{ state: GC9A01Kind::GC9A01_DELAY, value: 255},
    GC9A01Step{ state: GC9A01Kind::GC9A01_END, value: 0x0}
    ];

    /// GC9A01 driver
    pub struct GC9A01<SPI, CS, DC> {
        pub spi: SPI,
        pub cs: CS,
        pub dc: DC
    }

impl<SPI, CS, DC, E, PinError> GC9A01<SPI, CS, DC>
where
SPI: spi::Write<u8, Error=E>,
CS: OutputPin<Error = PinError>,
DC: OutputPin<Error = PinError>
{

    pub fn default(spi:SPI, cs:CS, dc:DC) -> Result<Self, E> {
        GC9A01::new(spi, cs, dc)
    }

    /// Takes a config object to initialize the adxl355 driver
    pub fn new(spi:SPI, cs:CS, dc: DC) -> Result<Self, E> {
        let mut gc9a01 = GC9A01 {
            spi,
            cs,
            dc,
        };

        Ok(gc9a01)
    }

    pub fn setup(&mut self)
    {
        for step in gc9a01_config.iter()
        {
            match step.state {
                GC9A01Kind::GC9A01_CMD => { 
                    self.gc9a01_command(step.value);
                }
                GC9A01Kind::GC9A01_DATA => { 
                    self.gc9a01_data(step.value);
                }
                GC9A01Kind::GC9A01_DELAY => { 
                }
                _ => { /* do nothing */ }
            }
        }
    }

    fn gc9a01_command(&mut self, cmd: u8)
    {
        self.cs.set_low().ok();
        self.dc.set_low().ok();
        self.spi.write(&[cmd]);
        self.cs.set_high().ok();
    }

    fn gc9a01_data(&mut self, data: u8)
    {
        self.cs.set_low().ok();
        self.dc.set_high().ok();
        self.spi.write(&[data]);
        self.cs.set_high().ok();
    }

    fn gc9a01_fillRect(&mut self, x: u16, y: u16, w: u16, h: u16, color: u16)
    {

        let mut _w: u16 = w;
        let mut _h: u16 = h;

        if (x + w - 1) > 240 {
            _w = 240 - x;
        }
        
        if (y + h - 1) > 240 {
            _h = 240 - y;
        }

        self.gc9a01_set_frame(x, y, x + _w - 1, y + _h - 1);

        let hi: u8 = (color >> 8) as u8;
        let lo: u8 = (color & 0xFF) as u8;

        for _y in 0.._h
        {
            for _x in 0.._w
            {
                self.gc9a01_data(hi);
                self.gc9a01_data(lo);
            }
        }
    }

    fn set_pixel(&mut self, x: u32, y: u32, color: u16) -> Result<(), CommError> {
        let hi: u8 = (color >> 8) as u8;
        let lo: u8 = (color & 0xFF) as u8;

        if x < 240 && x >= 0 && y >= 0 && y < 240 {

            self.gc9a01_set_frame(x as u16,y as u16,x as u16,y as u16);
            self.gc9a01_data(hi);
            self.gc9a01_data(lo);

        }

        Ok(())
    }

    fn gc9a01_set_frame(&mut self, x1: u16, y1: u16, x2: u16, y2: u16)
    {

        self.gc9a01_command(GC9A01_CASET); // Column addr set
        self.gc9a01_data((x1 >> 8) as u8);
        self.gc9a01_data((x1 & 0xFF) as u8);     // XSTART
        self.gc9a01_data((x2 >> 8) as u8);
        self.gc9a01_data((x2 & 0xFF) as u8);     // XEND

        self.gc9a01_command(GC9A01_RASET); // Row addr set
        self.gc9a01_data((y1 >> 8) as u8);
        self.gc9a01_data((y1 & 0xFF) as u8);     // YSTART
        self.gc9a01_data((y2 >> 8) as u8);
        self.gc9a01_data((y2 & 0xFF) as u8);     // YEND

        self.gc9a01_command(GC9A01_RAMWR);
    }

}

impl<SPI, CS, DC, E, PinError> OriginDimensions for GC9A01<SPI, CS, DC>
where
SPI: spi::Write<u8, Error=E>,
CS: OutputPin<Error = PinError>,
DC: OutputPin<Error = PinError>
{
    fn size(&self) -> Size {
        Size::new(240, 240)
    }
}

impl<SPI, CS, DC, E, PinError> DrawTarget for GC9A01<SPI, CS, DC>
where
SPI: spi::Write<u8, Error=E>,
CS: OutputPin<Error = PinError>,
DC: OutputPin<Error = PinError>
{
    type Color = Rgb565;
    type Error = CommError;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
        where
            I: IntoIterator<Item = Pixel<Self::Color>>,
        {
            for Pixel(coord, color) in pixels.into_iter() {
                self.set_pixel(coord.x as u32, coord.y as u32, RawU16::from(color).into_inner());
            }

            Ok(())
        }

}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
