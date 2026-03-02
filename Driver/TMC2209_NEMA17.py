import time
from machine import  Pin

class TMC2209:
    def __init__(self, step_pin_num=33, dir_pin_num=25, en_pin_num=32):
        # pin num
        self.step_pin_num = step_pin_num
        self.dir_pin_num = dir_pin_num
        self.en_pin_num = en_pin_num

        # pins
        self.step_pin = None
        self.dir_pin = None
        self.en_pin = None

    def init_pins(self):
        self.step_pin = Pin(self.step_pin_num, Pin.OUT)
        self.dir_pin = Pin(self.dir_pin_num, Pin.OUT)
        self.en_pin = Pin(self.en_pin_num, Pin.OUT)

    def turn_on(self):
        self.en_pin.value(0)

    def turn_off(self):
        self.en_pin.value(1)

    def set_dir(self, dir: int):
        if dir > 0:
            self.dir_pin.value(0)
        else:
            self.dir_pin.value(1)

    def one_step(self):
        self.step_pin.value(1)
        time.sleep_us(10)
        self.step_pin.value(0)

    def run_steps_with_delay(self, total_steps, delay_ms):
        for _ in range(total_steps):
            self.one_step()
            time.sleep_us(delay_ms)
