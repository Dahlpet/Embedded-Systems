import smbus

bus = smbus.SMBus(1) # 1 indicates /dev/i2c-1
address = 0x6b       # LSM9DS1 gyro and accelerometer address

# Read the WHO_AM_I register to verify the connection
whoami = bus.read_byte_data(address, 0x0F)

if whoami == 0x68:   # WHO_AM_I should return 0x68
    print("LSM9DS1 is connected")
else:
    print("LSM9DS1 is NOT connected")