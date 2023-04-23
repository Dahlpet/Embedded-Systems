import time, math, board, busio, adafruit_lsm9ds1
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import RPi.GPIO as GPIO

ROLL_ALARM = 45 # grader
PITCH_ALARM = 45 # grader
YAW_ALARM = 200 # grader
DISTANCE_ALARM = 15 # cm

GPIO.setup(21, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setmode(GPIO.BCM)

TRIG = 23
ECHO = 24

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Initialize I2C bus and sensor.
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

def pulse_time_to_distance():
    # Send trig signal.
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Wait for echo signal to start.
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    # Wait for echo signal to end.
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculate pulse duration and convert to distance.
    pulse_duration = pulse_end - pulse_start
    speed_of_sound = 34300  # cm/s
    distance = pulse_duration * speed_of_sound / 2

    return distance

fig = plt.figure()

ax = fig.add_subplot(111, projection='3d')
ax.set(xlabel='X', ylabel='Y', zlabel='Z')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)

# Create a plane
xx, yy = np.meshgrid([-0.5, 0.5], [-0.5, 0.5])
zz = np.zeros_like(xx)
plane = ax.plot_surface(xx, yy, zz, alpha=0.5, color='blue', edgecolor='none')

print("meassurment in progress...")

while True:
    # Read sensor data.
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro

    # Calculate roll, pitch, and yaw.
    roll= math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
    pitch= math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
    mag_x *= math.cos(pitch)
    mag_y *= math.cos(roll)
    heading = math.atan2(mag_y, mag_x)
    yaw = heading + 1.5

    # Rotate the plane
    rotation_matrix = np.array([[math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]])
    rotation_matrix = np.matmul(rotation_matrix, np.array([[math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]]))
    rotation_matrix = np.matmul(rotation_matrix, np.array([[1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]]))

    plane.remove()
    rotated_xx, rotated_yy, rotated_zz = np.dot(np.dstack((xx, yy, zz)), rotation_matrix).squeeze().T
    plane = ax.plot_surface(rotated_xx, rotated_yy, rotated_zz, alpha=0.5, color='blue', edgecolor='none')

    # Check for alarms
    distance = pulse_time_to_distance()
    if abs(math.degrees(roll)) > ROLL_ALARM or abs(math.degrees(pitch)) > PITCH_ALARM or abs(math.degrees(yaw)) > YAW_ALARM:
        plane.set_color('r')
        GPIO.output(21, GPIO.HIGH)
        print("Orientation alarm!")
    elif distance < DISTANCE_ALARM:
        plane.set_color('r')
        GPIO.output(20, GPIO.HIGH)
        print("Distance alarm!")
    else:
        plane.set_color('b')
        GPIO.output(21, GPIO.LOW)
        GPIO.output(20, GPIO.LOW)

    # Set title with orientation data and distance.
    distance = pulse_time_to_distance()
    ax.set_title(f"Roll: {math.degrees(roll):.1f}, Pitch: {math.degrees(pitch):.1f}, Yaw: {math.degrees(yaw):.1f}\nDistance: {distance:.2f} cm")

    fig.canvas.draw()
    plt.pause(0.01)