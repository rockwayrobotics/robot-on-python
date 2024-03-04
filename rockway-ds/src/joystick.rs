use log::*;

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicI8, AtomicI16, AtomicU16, Ordering};
use std::{thread, time::Duration};

use hidapi::{HidApi, HidResult};

use ds::JoystickValue;


const POLL_PERIOD: u64 = 20; // milliseconds

const AXIS_COUNT: usize = 6;


#[derive(Copy, Clone, Default)]
pub struct JoystickData {
    pub buttons: u16, // bits 0..15 are the buttons
    pub axes: [i8; AXIS_COUNT],
    pub pov: i16,
    pub count: u8,
}

#[derive(Default)]
struct SharedJoystickData {
    pub buttons: AtomicU16, // bits 0..15 are the buttons
    pub axes: [AtomicI8; AXIS_COUNT],
    pub pov: AtomicI16,
}

#[derive(Copy, Clone, Default)]
enum HidType {
    #[default]
    Ps4, // the wireless controller, on USB
    // Xbox,
}

#[derive(Default)]
pub struct Joystick {
    hidtype: HidType,
    data: SharedJoystickData,
    count: AtomicU8,
    running: AtomicBool,
}

impl Joystick {
    pub fn new() -> Self {
        Self {
            hidtype: HidType::Ps4,
            data: SharedJoystickData::default(),
            count: AtomicU8::new(0),
            running: AtomicBool::new(false),
        }
    }

    pub fn count(&self) -> u8 {
        self.count.load(Ordering::Relaxed)
    }

    pub fn stop(&self) {
        self.running.store(false, Ordering::Relaxed);
    }

    pub fn get_data(&self) -> JoystickData {
        let data = &self.data;

        let mut axes = [0; AXIS_COUNT];
        for (i, x) in axes.iter_mut().enumerate() {
            *x = data.axes[i].load(Ordering::Relaxed);
        }

        JoystickData {
            buttons: data.buttons.load(Ordering::Relaxed),
            axes,
            pov: data.pov.load(Ordering::Relaxed),
            count: self.count.load(Ordering::Relaxed),
        }
    }

    pub fn get_report(&self) -> Vec<Vec<JoystickValue>> {
        let mut out = Vec::with_capacity(1);
        let mut inner = Vec::with_capacity(10+6+1);

        let mut buttons = self.data.buttons.load(Ordering::Relaxed);
        // For now, the ds crate is limited to button ids 1..=10 so ignore the rest.
        buttons &= 0b11_1111_111;
        while buttons != 0 {
            let i = buttons.trailing_zeros(); // 0..=16
            inner.push(JoystickValue::Button { id: i as u8 + 1, pressed: true });
            buttons &= !(1 << i);
        }

        for id in 0..self.data.axes.len() {
            let v = self.data.axes[id].load(Ordering::Relaxed);
            let value = v as f32 / 128f32;
            inner.push(JoystickValue::Axis { id: id as u8, value });
        }

        let angle = match self.data.pov.load(Ordering::Relaxed) {
            0b1000 => -1,
            x => x * 45,
        };
        inner.push(JoystickValue::POV { id: 0, angle });

        out.push(inner);
        out
    }

    pub fn run(&self) {
        self.running.store(true, Ordering::Relaxed);
        // println!("Printing all available hid devices:");

        // match HidApi::new() {
        //     Ok(api) => {
        //         for device in api.device_list() {
        //             println!(
        //                 "VID: {:04x}, PID: {:04x}, Serial: {}, Product name: {}, Interface: {}",
        //                 device.vendor_id(),
        //                 device.product_id(),
        //                 match device.serial_number() {
        //                     Some(s) => s,
        //                     _ => "<COULD NOT FETCH>",
        //                 },
        //                 match device.product_string() {
        //                     Some(s) => s,
        //                     _ => "<COULD NOT FETCH>",
        //                 },
        //                 device.interface_number()
        //             );
        //         }
        //     }
        //     Err(e) => {
        //         eprintln!("Error: {}", e);
        //     }
        // }

        let api = HidApi::new().expect("Failed to create API instance");

        let (vid, pid) = match self.hidtype {
            HidType::Ps4 => (0x054c, 0x09cc),
        };

        if let HidResult::Ok(stick) = api.open(vid, pid) {
            while self.running.load(Ordering::Relaxed) {
                let mut buf = [0u8; 256];
                let _res = stick.read(&mut buf[..]).unwrap();

                self.count.fetch_add(1, Ordering::Relaxed);

                // byte 5 bits: 0..3=POV, 4=Square, 5=Cross, 6=Circle, 7=Triangle
                // byte 6 bits: 0=L1, 1=R1, 2=L2, 3=R2, 4=Share, 5=Options, 6=L3, 7=R3
                // byte 7 bits: 0=Touchpad, 1=PS
                // PS4: 1=Square, 2=Cross, 3=Circle, 4=Triangle, 5=L1, 6=R1,
                //      7=L2, 8=R2, 9=Share, 10=Options, 11=L3, 12=R3,
                //      13=PS, 14=Touchpad  (note: wpilib button numbers not bits)
                let mut buttons: u16 = 0;
                match self.hidtype {
                    HidType::Ps4 => {
                        self.data.pov.store((buf[5] & 0x0f) as i16, Ordering::Relaxed);

                        for (i, mask) in &[(4, 0x01), (5, 0x02), (6, 0x04), (7, 0x08)] {
                            if buf[5] & (1 << i) != 0 {
                                buttons |= mask;
                            }
                        }

                        // TODO: adjust all the output bits up by one, if that turns
                        // out to be correct instead of this.

                        // Note: L3/R3 in top two bits will be discarded by ds crate
                        for (i, mask) in [0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800].iter().enumerate() {
                            if buf[6] & (1 << i) != 0 {
                                buttons |= mask;
                            }
                        }
                    }

                    // Xbox: 1=A, 2=B, 3=X, 4=Y, 5=LeftBumper, 6=RightBumper,
                    //      7=Back, 8=Start, 9=LeftStick, 10=RightStick
                    // Joystick: 0=Trigger, 1=Top
                }

                // debug!("buttons {:010b}", buttons);
                self.data.buttons.store(buttons, Ordering::Relaxed);

                // PS4: 0=LeftX, 1=LeftY, 2=RightX, 3=L2, 4=R2, 5=RightY
                //      Where L2/R2 are throttles so possibly 0 to 1.0 instead of
                //      -1.0 to 1.0.
                match self.hidtype {
                    HidType::Ps4 => {
                        self.data.axes[0].store((buf[1] - 128) as i8, Ordering::Relaxed);
                        self.data.axes[1].store((buf[2] - 128) as i8, Ordering::Relaxed);
                        self.data.axes[2].store((buf[3] - 128) as i8, Ordering::Relaxed);
                        self.data.axes[5].store((buf[4] - 128) as i8, Ordering::Relaxed);
                        // Note: these normally sit at -1.0 and go up to 1.0
                        self.data.axes[3].store((buf[8] - 128) as i8, Ordering::Relaxed);
                        self.data.axes[4].store((buf[9] - 128) as i8, Ordering::Relaxed);
                    }
                }

                // debug!("{:?}", self.get_report());

                thread::sleep(Duration::from_millis(POLL_PERIOD));
            }
        }
        else {
            loop {
                thread::sleep(Duration::from_millis(100000));
            }
        }
    }
}


    // use hidapi::HidApi;

    // println!("Printing all available hid devices:");

    // match HidApi::new() {
    //     Ok(api) => {
    //         for device in api.device_list() {
    //             println!(
    //                 "VID: {:04x}, PID: {:04x}, Serial: {}, Product name: {}, Interface: {}",
    //                 device.vendor_id(),
    //                 device.product_id(),
    //                 match device.serial_number() {
    //                     Some(s) => s,
    //                     _ => "<COULD NOT FETCH>",
    //                 },
    //                 match device.product_string() {
    //                     Some(s) => s,
    //                     _ => "<COULD NOT FETCH>",
    //                 },
    //                 device.interface_number()
    //             );
    //         }
    //     }
    //     Err(e) => {
    //         eprintln!("Error: {}", e);
    //     }
    // }

    // let api = HidApi::new().expect("Failed to create API instance");

    // let joystick = api.open(0x054c, 0x09cc).expect("Failed to open device");

    // loop {
    //     let mut buf = [0u8; 256];
    //     let res = joystick.read(&mut buf[..]).unwrap();

    //     let mut data_string = String::new();

    //     for (i, u) in buf[..res].iter().enumerate() {
    //         match i {
    //             1 | 2 | 3 | 4 | 8 | 9
    //                 => data_string.push_str(&format!("{u:4}")),

    //             5 | 6
    //                 => data_string.push_str(&format!(" {u:02x}")),

    //             7 => data_string.push_str(&format!(" {:02x}", u & 0b11)),

    //             20 | 22 | 24
    //                 => data_string.push_str(&format!("{:+6.2}", (*u as i8) as f32 / 0x20 as f32)),

    //             _ => {}
    //         }
    //     }

    //     println!("{}", data_string);
    //     std::thread::sleep(std::time::Duration::from_millis(100));
    // }

    // std::thread::sleep(std::time::Duration::from_secs(2));

