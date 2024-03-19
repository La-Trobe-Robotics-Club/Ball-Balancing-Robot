use serialport::SerialPort;

#[cfg(target_family = "unix")]
use std::os::unix::net::UnixListener;

// note will only work on windows 10 build 17063+
// see https://devblogs.microsoft.com/commandline/af_unix-comes-to-windows
#[cfg(target_family = "windows")]
use uds_windows::UnixListener;

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
