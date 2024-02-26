import serial

# Serial config
ser = serial.Serial('/dev/ttyUSB0', timeout = 1)
center_motor = 0
left_motor = 0
right_motor = 0
serial_count = 0
serial_sent = None
serial_recieved = None

while True:
    # Output to Serial
    serial_recieved = ser.read()
    if serial_sent == None:
        serial_recieved = None
        match serial_count:
            case 0:
                ser.write(center_motor)
                serial_sent = center_motor
                serial_count = 1
            case 1:
                ser.write(left_motor)
                serial_sent = left_motor
                serial_count = 2
            case 2:
                ser.write(right_motor)
                serial_sent = right_motor
                serial_count = 0
            case _:
                print("ERROR: serial_count out of bounds")
                # TODO: Error here
    print(f" sent:{serial_sent} recieved:{serial_recieved}")