import smbus
import time

bus = smbus.SMBus(1)
address = 0x6b

# Enable the accelerometer and set the output data rate to 119 Hz
bus.write_byte_data(address, 0x10, 0b10010111)

# Enable the gyroscope and set the output data rate to 119 Hz
bus.write_byte_data(address, 0x11, 0b10010111)

# Enable the magnetometer and set the output data rate to 20 Hz
bus.write_byte_data(address, 0x12, 0b00111000)

while True:
    # Read accelerometer data
    acc_x = bus.read_word_data(address, 0x28)
    acc_y = bus.read_word_data(address, 0x2a)
    acc_z = bus.read_word_data(address, 0x2c)

    # Read gyroscope data
    gyro_x = bus.read_word_data(address, 0x18)
    gyro_y = bus.read_word_data(address, 0x1a)
    gyro_z = bus.read_word_data(address, 0x1c)

    # Read magnetometer data
    mag_x = bus.read_word_data(address, 0x28)
    mag_y = bus.read_word_data(address, 0x2a)
    mag_z = bus.read_word_data(address, 0x2c)

    # Convert raw data to engineering units
    acc_x = acc_x / 16384.0
    acc_y = acc_y / 16384.0
    acc_z = acc_z / 16384.0

    gyro_x = gyro_x / 131.0
    gyro_y = gyro_y / 131.0
    gyro_z = gyro_z / 131.0

    mag_x = mag_x * 0.48828125
    mag_y = mag_y * 0.48828125
    mag_z = mag_z * 0.48828125

    # Print data to console
    print("Accelerometer (g): x=%.2f y=%.2f z=%.2f" % (acc_x, acc_y, acc_z))
    print("Gyroscope (dps): x=%.2f y=%.2f z=%.2f" % (gyro_x, gyro_y, gyro_z))
    print("Magnetometer (G): x=%.2f y=%.2f z=%.2f" % (mag_x, mag_y, mag_z))

    # Wait for next sample
    time.sleep(0.1)
