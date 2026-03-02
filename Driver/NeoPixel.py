import machine, neopixel

class NeoPixelController:
    def __init__(self, n: int = 8, data_pin: int = 19):
        self.led_num = n
        self.data_pin_num = data_pin

        self.data_pin = None
        self.np_c = None

        # test var
        self.cycle = 0

    def init(self):
        self.data_pin = machine.Pin(self.data_pin_num, machine.Pin.OUT)
        self.np_c = neopixel.NeoPixel(self.data_pin, self.led_num)

    def set_individual_color(self, index, color, write_ins: bool = True):
        self.np_c[index] = color
        if write_ins:
            self.np_c.write()

    def write_color(self):
        self.np_c.write()

    def set_all_color(self, color):
        for index in range(self.led_num):
            self.np_c[index] = color

        self.np_c.write()

    def test_led(self):
        self.set_all_color((0, 0, 0))
        self.set_individual_color(self.cycle, (20, 0, 0), False)

        if self.cycle > 0:
            self.set_individual_color(self.cycle - 1, (0, 20, 0), False)
        else:
            self.set_individual_color(7, (0, 20, 0), False)

        if self.cycle > 1:
            self.set_individual_color(self.cycle - 2, (0, 0, 20), False)
        elif self.cycle == 1:
            self.set_individual_color(7, (0, 0, 20), False)
        else:
            self.set_individual_color(6, (0, 0, 20), False)


        self.write_color()

        self.cycle += 1
        if self.cycle > 7:
            self.cycle = 0

