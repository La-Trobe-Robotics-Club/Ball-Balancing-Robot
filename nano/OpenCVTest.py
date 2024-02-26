import serial

# Serial config
ser = serial.Serial('/dev/ttyUSB0')
center_motor = 0
left_motor = 0
right_motor = 255
serial_count = 0
serial_sent = None
serial_recieved = None
# Wait for arduino to be ready
print("Waiting for Arduino")
ser.read_until("STARTED\r\n", 9)
print("Arduino ready")
ser.write(int.to_bytes(center_motor))
serial_sent = center_motor
serial_count = 1
print(f"count: {serial_count} sent:{str(serial_sent):<4} recieved:{str(serial_recieved):<4}")

while True:
    # Output to Serial
    if ser.in_waiting == 1:
        serial_recieved = int.from_bytes(ser.read(), byteorder="little")
        if serial_sent == None or serial_sent == serial_recieved:
            match serial_count:
                case 0:
                    ser.write(int.to_bytes(center_motor))
                    serial_sent = center_motor
                    serial_count = 1
                case 1:
                    ser.write(int.to_bytes(left_motor))
                    serial_sent = left_motor
                    serial_count = 2
                case 2:
                    ser.write(int.to_bytes(right_motor))
                    serial_sent = right_motor
                    serial_count = 0
                case _:
                    print("ERROR: serial_count out of bounds")
                    # TODO: Error here
            print(f"count: {serial_count} sent:{str(serial_sent):<4} recieved:{str(serial_recieved):<4}")
            serial_recieved = None
    elif ser.in_waiting > 1:
        print(f"{ser.read(ser.in_waiting)}")