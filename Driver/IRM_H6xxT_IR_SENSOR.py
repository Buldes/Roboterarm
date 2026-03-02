import machine
import sys
sys.path.append('/Driver/library/')
from ir_rx.nec import NEC_8

class IR_Controller:
    def __init__(self, ir_pin_num: int = 18):
        self.ir_pin_num = ir_pin_num
        self.ir_pin = None
        self.ir_callback = None

        self.last_press = None

    def init(self):
        self.ir_pin = machine.Pin(self.ir_pin_num, machine.Pin.IN, machine.Pin.PULL_UP)

    def set_test_callback(self):
        self.ir_pin.irq(trigger=machine.Pin.IRQ_FALLING, handler=lambda _:print("SIGNAL"))

    def set_callback(self):
        self.ir_callback = NEC_8(self.ir_pin, self.callback)

    def callback(self, data, addr, ctrl):
        if data < 0:
            return
        self.last_press = hex(data)

    def close_callback(self):
        self.ir_callback.close()

    def get_last_press(self):
        last = self.last_press
        self.last_press = None
        return last

