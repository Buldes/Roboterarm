import time
from machine import Pin, I2C

class Servo_Driver:
        def __init__(self, addr: bytes = 0x40,emergency_switch_pin_num: int = 4):
            # i2c var
            self.i2c = None
            self.addr = addr

            # emergency off
            self.emergency_switch_pin_num = emergency_switch_pin_num
            self.emergency_switch_pin = None

            # servo config
            self.servo_config: list = [] # [[PIN_NUM, MAX_DEGREE, MIN_DUTY, MAX_DUTY], ...]


        def init_i2c(self):
            # i2c
            self.i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
            self.addr = 0x40
            # emergency switch
            self.emergency_switch_pin = Pin(self.emergency_switch_pin_num, Pin.OUT)

            # write wakeup
            self.i2c.writeto_mem(self.addr, 0x00, b"\x00")
            time.sleep_ms(20)

            # config
            self.i2c.writeto_mem(self.addr, 0x00, b'\x10')  # Sleep mode
            self.i2c.writeto_mem(self.addr, 0xFE, b'\x79')  # Prescale to 121 (50Hz)
            self.i2c.writeto_mem(self.addr, 0x00, b'\x00')  # Wake up
            time.sleep_ms(10)
            self.i2c.writeto_mem(self.addr, 0x00, b'\xa1')  # Auto-increment on

        def sec_to_ticks(self, time_sec):
            return int(round(time_sec / (20_000/4_096)))

        def turn_off(self):
            self.emergency_switch_pin.value(0)

            for a in range(16):
                self.set_channel_duty(a, 0)

        def turn_on(self):
            self.emergency_switch_pin.value(1)

            for servo in self.servo_config:
                self.set_channel_duty(servo[0], servo[2])

        def add_servo_config(self, PIN_NUM, MAX_DEGREE, MIN_DUTY, MAX_DUTY):
            self.servo_config.append([PIN_NUM, MAX_DEGREE, MIN_DUTY, MAX_DUTY])

        def delete_servo_config(self, num):
            self.servo_config.pop(num)

        def set_servo_to(self, num, degree):
            all_configs = self.servo_config[num]

            percentage = degree / all_configs[1]

            duty_num = int(round(all_configs[2] + ((all_configs[3] - all_configs[2]) * percentage)))

            self.set_channel_duty(all_configs[0], duty_num)


        def set_channel_duty(self, channel_num, duty_num):
            reg = 0x06 + (channel_num * 4)

            data = bytearray([0, 0, duty_num & 0xFF, (duty_num >> 8) & 0xFF])
            self.i2c.writeto_mem(self.addr, reg, data)