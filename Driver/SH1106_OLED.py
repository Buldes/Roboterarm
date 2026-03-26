import time
from machine import Pin, I2C
import sys
sys.path.append('/Driver/library/')
from Driver.library import sh1106
import framebuf

class SH1106_OLED:
        def __init__(self):
            # i2c var
            self.i2c = None
            self.display = None

            # test var
            self.num = 0

        def init_i2c(self):
            self.i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
            self.display = sh1106.SH1106_I2C(128, 64, self.i2c)
            self.display.init_display()
            time.sleep_ms(20)
            self.clear()
            self.set_contrast(255)

        def clear(self, value: int=0):
            self.display.fill(value)

        def draw_text(self, text: str, x: int, y: int, color: int = 1):
            self.display.text(text, x, y, color)

        def show_content(self):
            self.display.show()

        def set_contrast(self, value: int):
            # value: 0 to 255
            self.display.contrast(value)

        def test_oled(self):

            self.clear(1)
            self.draw_text(f"{self.num}", 10, 10, 0)
            self.show_content()
            self.set_contrast(self.num)

            self.num += 1
            self.num = self.num % 255

        def draw_rect(self, x: int, y: int, w: int, h: int, color: int = 1, fill: int = 1):
            if fill:
                self.display.fill_rect(x, y, w, h, color)
            else:
                self.display.rect(x, y, w, h, color)

        def draw_big_text(self, text, x, y, scale, color=1):
            for char in text:

                char_buffer = bytearray(8)
                fb = framebuf.FrameBuffer(char_buffer, 8, 8, framebuf.MONO_VLSB)
                fb.text(char, 0, 0, 1)

                for cy in range(8):
                    for cx in range(8):
                        if fb.pixel(cx, cy):
                            self.display.fill_rect(int(x + cx * scale), int(y + cy * scale), int(scale), int(scale), color)

                x = int(x + 8 * scale)
