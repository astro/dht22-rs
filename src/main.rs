#![no_std]
#![feature(used)]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
#[macro_use(exception, interrupt)]
extern crate stm32f429x;
extern crate embedded_hal;

use cortex_m::asm;
use stm32f429x::{Peripherals, CorePeripherals, GPIOC};
use embedded_hal::digital::*;

use core::fmt::Write;
use cortex_m_semihosting::hio;

struct DHT22Pin<'a> {
    gpio: &'a GPIOC,
}

const GPIO_MODER_INPUT: u8 = 0b00;
const GPIO_MODER_OUTPUT: u8 = 0b01;

impl<'a> DHT22Pin<'a> {
    pub fn new(p: &'a Peripherals) -> Self {
        let gpio = &p.GPIOC;

        DHT22Pin { gpio }
    }
}

impl<'a> IoPin for DHT22Pin<'a> {
    type Input = DHT22Input<'a>;
    type Output = DHT22Output<'a>;

    fn as_input<R, F>(&mut self, f: F) -> R
    where F: FnOnce(&Self::Input) -> R
    {
        self.gpio.moder.modify(|_, w| unsafe { w.moder0().bits(GPIO_MODER_INPUT) });
        f(&DHT22Input { gpio: self.gpio })
    }

    fn as_output<R, F>(&mut self, f: F) -> R
    where F: FnOnce(&mut Self::Output) -> R
    {
        self.gpio.moder.modify(|_, w| unsafe { w.moder0().bits(GPIO_MODER_OUTPUT) });
        f(&mut DHT22Output { gpio: self.gpio })
    }
}

struct DHT22Input<'a> {
    gpio: &'a GPIOC,
}

impl<'a> InputPin for DHT22Input<'a> {
    fn is_high(&self) -> bool {
        self.gpio.idr.read()
            .idr0()
            .bit_is_clear()
    }

    fn is_low(&self) -> bool {
        self.gpio.idr.read()
            .idr0()
            .bit_is_set()
    }
}

struct DHT22Output<'a> {
    gpio: &'a GPIOC,
}

impl<'a> OutputPin for DHT22Output<'a> {
    fn is_low(&self) -> bool {
        self.gpio.odr
            .read()
            .odr0().bit_is_set()
    }

    fn is_high(&self) -> bool {
        ! self.is_low()
    }

    fn set_low(&mut self) {
        self.gpio.bsrr
            .write(|w| w.br0().set_bit())
    }

    fn set_high(&mut self) {
        self.gpio.bsrr
            .write(|w| w.bs0().set_bit())
    }
}

struct PulseWidthDecoder<'p, P: 'p> {
    times: [usize; 40],
    total_time: usize,
    input: &'p P,
}

const TIMEOUT: usize = 10000;

impl<'p, P: InputPin> PulseWidthDecoder<'p, P> {
    // TODO: timeouts
    fn recv_from(input: &P) -> usize {
        // Wait
        let mut time = 0;
        while input.is_high() && time < TIMEOUT {
            time += 1;
        }
        // Count
        let mut time = 0;
        while ! input.is_high() && time < TIMEOUT {
            time += 1;
        }
        time
    }

    fn new(input: &'p P, times: [usize; 40]) -> Self {
        PulseWidthDecoder {
            times,
            input,
            total_time: 0,
        }
    }
    
    fn run(&mut self) -> bool {
        self.total_time = 0;
        for t in 0..self.times.len() {
            self.times[t] = Self::recv_from(&self.input);
            if self.times[t] >= TIMEOUT {
                return false;
            }
            self.total_time += self.times[t];
        }

        // Success
        true
    }

    fn to_bytes(&self) -> [u8; 5] {
        let mut result = [0; 5];
        let mut i = 0;
        let mut mask = 0x80;
        let avg_time = self.total_time / self.times.len();
        for bit in self.times
            .iter()
            .map(|time| *time >= avg_time)
        {
            if bit {
                result[i] |= mask;
            }
            mask >>= 1;
            if mask == 0 {
                mask = 0x80;
                i += 1;
            }
        }
        result
    }
}

#[derive(Debug)]
pub struct SensorData {
    /// Relative humidity
    pub humidity: f32,
    /// Degrees Celsius
    pub temperature: f32,
}

impl SensorData {
    pub fn from(data: [u8; 5]) -> Option<Self> {
        let checksum = data[0..4].iter()
            .fold(0, |sum, b| b + sum);
        if checksum == data[4] {
            let humidity =
                (((data[0] as u16) << 8) | (data[1] as u16)) as f32 / 10.0;
            let temperature =
                (((data[2] as i16) << 8) | (data[3] as i16)) as f32 / 10.0;
            Some(SensorData { humidity, temperature })
        } else {
            let mut stdout = hio::hstdout().unwrap();
            writeln!(stdout, "Expected checksum {:02X} but was {:02X}", checksum, data[4]).unwrap();

            None
        }
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn sensor_data_from_normal() {
        let d = SensorData::from([0b00000010, 0b10001100, 0b00000001, 0b01011111, 0b11101110]);
        assert!(d.is_some());
        let d = d.unwrap();
        assert_eq!(d.humidity, 65.2);
        assert_eq!(d.temperature, 35.1);
    }

    #[test]
    fn sensor_data_from_subzero() {
        let d = SensorData::from([0b00000010, 0b10001100, 0b00000001, 0b01011111, 0b11101110]);
        assert!(d.is_some());
        let d = d.unwrap();
        assert_eq!(d.temperature, -10.1);
    }
}


fn main() {
    let mut stdout = hio::hstdout().unwrap();

    let p = Peripherals::take().unwrap();
    // let mut cp = CorePeripherals::take().unwrap();

    p.RCC.ahb1enr.modify(|_, w| {
        w.gpiocen().set_bit()
    });

    let mut pin = DHT22Pin::new(&p);
    loop {
        pin.as_output(|out| {
            out.set_low();
            for _ in 0..20000 {
                asm::nop();
            }
            out.set_high();
        });

        pin.as_input(|input| {
            while ! input.is_high() {}
            while input.is_high() {}
            while ! input.is_high() {}

            let mut dec = PulseWidthDecoder::new(input, [0usize; 40]);
            if dec.run() {
                let avg_time = dec.total_time / dec.times.len();

                write!(stdout, "Times:").unwrap();
                for (i, time) in dec.times.iter().enumerate() {
                    if i > 0 && i % 8 == 0 {
                        write!(stdout, " |").unwrap();
                    }
                    write!(stdout, " {}", time).unwrap();
                }
                writeln!(stdout, " | AVG={}", avg_time).unwrap();
                writeln!(stdout, "Bytes: {:?}", dec.to_bytes()).unwrap();
                let sd = SensorData::from(dec.to_bytes());
                writeln!(stdout, "Data: {:?}", sd).unwrap();
            }
        });
    }
}
