from machine import Pin, PWM

class Fan_Controller:
    def __init__(self,
                 fan_pin_num: int = 16,
                 fan_freq: int = 10_000,
                 min_percentage: int = 0.2):

        self.fan_pin_num: int = fan_pin_num
        self.pin_freq = fan_freq
        self.min_percentage = min_percentage

        self.fan_pin = None
        self.fan_pwm = None

        self.min_temp: float = 20.0
        self.max_temp: float = 50.0

    def init(self):
        self.fan_pin = Pin(self.fan_pin_num, Pin.OUT)
        self.fan_pwm = PWM(self.fan_pin, freq=self.pin_freq, duty_u16=1023)

    def set_fan_percentage(self, percentage: float):
        if percentage < self.min_percentage:
            self.fan_pwm.duty(0)
        else:
            max_duty = 1023
            duty_int = int(percentage * max_duty)
            min_duty = int(self.min_percentage * max_duty)
            self.fan_pwm.duty(min(max(duty_int, min_duty), max_duty))

    def config_temperature(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def set_percentage_by_temp(self, temp):
        if temp < self.min_temp:
            self.fan_pwm.duty(0)
            return

        relative_temp = temp - self.min_temp
        percentage = relative_temp / (self.max_temp - self.min_temp)

        self.set_fan_percentage(max(percentage, self.min_percentage))