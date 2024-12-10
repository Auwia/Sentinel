import time

try:
    from adafruit_pca9685 import PCA9685
    from board import SCL, SDA
    import busio
except ImportError:
    class MockPCA9685:
        class Channel:
            def __init__(self):
                self.duty_cycle = 0

        def __init__(self, *args, **kwargs):
            self.channels = [self.Channel() for _ in range(16)]

        def deinit(self):
            print("Mock PCA9685 deinizializzato.")

    PCA9685 = MockPCA9685
    SCL = SDA = None

    class MockI2C:
        def __init__(self, *args, **kwargs):
            pass

    busio = MockI2C


class ServoController:
    def __init__(self, i2c_address=0x40, frequency=50):
        self.i2c = busio.I2C(SCL, SDA) if SCL and SDA else busio()
        self.pca = PCA9685(self.i2c, address=i2c_address)
        self.pca.frequency = frequency
        print(f"Controller PCA9685 inizializzato all'indirizzo I2C {i2c_address} con frequenza {frequency} Hz.")

    def set_servo_angle(self, channel, angle):
        min_duty = 0x0666
        max_duty = 0x2CCC
        duty_cycle = int(min_duty + (angle / 180.0) * (max_duty - min_duty))
        self.pca.channels[channel].duty_cycle = duty_cycle
        print(f"Canale {channel}: Angolo impostato a {angle}° (Duty cycle: {hex(duty_cycle)})")

    def stop_servo(self, channel):
        self.pca.channels[channel].duty_cycle = 0
        print(f"Canale {channel}: Servo spento.")

    def cleanup(self):
        for channel in range(16):
            self.pca.channels[channel].duty_cycle = 0
        self.pca.deinit()
        print("Controller PCA9685 pulito e risorse rilasciate.")
