from machine import Pin, PWM
import time

class LED_Stripe:
    def __init__(self):
        self.freq: int = 2000

        self.r_pin_num = 12
        self.g_pin_num = 14
        self.b_pin_num = 23

        self.r_pin = None
        self.g_pin = None
        self.b_pin = None

        self.r_pin_pwm = None
        self.g_pin_pwm = None
        self.b_pin_pwm = None

    def init_pins(self):
        self.r_pin = Pin(self.r_pin_num, Pin.OUT)
        self.g_pin = Pin(self.g_pin_num, Pin.OUT)
        self.b_pin = Pin(self.b_pin_num, Pin.OUT)

        self.r_pin_pwm = PWM(self.r_pin, freq=self.freq, duty=512)
        self.g_pin_pwm = PWM(self.g_pin, freq=self.freq, duty=512)
        self.b_pin_pwm = PWM(self.b_pin, freq=self.freq, duty=512)

    def set_freq(self, freq: int, pin_str: str="all"):
        if pin_str == "all":
            self.r_pin_pwm.freq(freq)
            self.g_pin_pwm.freq(freq)
            self.b_pin_pwm.freq(freq)
        elif pin_str == "r":
            self.r_pin_pwm.freq(freq)
        elif pin_str == "g":
            self.g_pin_pwm.freq(freq)
        elif pin_str == "b":
            self.b_pin_pwm.freq(freq)

    def value_to_duty(self, r):
        return int(1023 * (r / 255))

    def set_rgb(self, r, g, b):
        self.r_pin_pwm.duty(self.value_to_duty(r))
        self.g_pin_pwm.duty(self.value_to_duty(g))
        self.b_pin_pwm.duty(self.value_to_duty(b))

    def test_rgb(self):
        self.set_rgb(255, 0, 0)
        time.sleep(0.5)
        self.set_rgb(125, 0, 0)
        time.sleep(0.5)
        self.set_rgb(0, 255, 0)
        time.sleep(0.5)
        self.set_rgb(0, 125, 0)
        time.sleep(0.5)
        self.set_rgb(0, 0, 255)
        time.sleep(0.5)
        self.set_rgb(0, 0, 120)
        time.sleep(0.5)

    def test_linear_gradient(self, cycles: int = 2):
        r = 255
        g = 0
        b = 0

        for _ in range(255 * 3 * cycles):

            if r >= 1 and b == 0:
                if g < 255:
                    g += 1
                else:
                    r -= 1

            elif g >= 1:
                if b < 255:
                    b += 1
                else:
                    g -= 1

            elif b >= 1:
                if r < 255:
                    r += 1
                else:
                    b -= 1

            self.set_rgb(r, g, b)
            time.sleep_ms(10)
