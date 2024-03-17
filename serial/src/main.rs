use serialport::SerialPort;
use std::os::unix::net::UnixListener; // Is this using uds_windows crate or not?

struct ArduinoSerial {
    port: Box<dyn SerialPort>,
}

impl ArduinoSerial {
    pub fn new(port: String) -> Self {
        Self {
            port: serialport::new(port, 9600)
                .open()
                .expect("Failed to open serial port"),
        }
    }
}

struct Ipc {
    listener: UnixListener,
}

impl Ipc {
    pub fn new(socket: String) -> Self {
        Self {
            listener: UnixListener::bind(socket).expect("Failed to bind socket"),
        }
    }
}

fn main() {
    todo!();
    let serial = ArduinoSerial::new("/dev/ttyUSB0".to_string());
    let socket = Ipc::new("./socket".to_string());
    loop {
        // Communicate with user program and pass information onto Arduino
        todo!();
    }
}
