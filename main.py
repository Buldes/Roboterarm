import sys
import time
import machine

# Add Path to Driver
sys.path.append('/Driver/')
# import Driver
import Kinematic_Engine
import PCA9685_Servo
import TMC2209_NEMA17
import LED_Stripe
import DS18B20
import Fan_Driver
import SH1106_OLED
import NeoPixel
import IRM_H6xxT_IR_SENSOR

"""SERVO"""
# sd = PCA9685_Servo.Servo_Driver()
# sd.init_i2c()
#
# sd.add_servo_config(0, 180, sd.sec_to_ticks(650), sd.sec_to_ticks(2600))
# sd.add_servo_config(1, 180, sd.sec_to_ticks(560), sd.sec_to_ticks(2600))
# sd.add_servo_config(2, 180, sd.sec_to_ticks(650), sd.sec_to_ticks(2600))
# sd.add_servo_config(3, 270, sd.sec_to_ticks(430), sd.sec_to_ticks(2500))
# sd.add_servo_config(4, 270, sd.sec_to_ticks(430), sd.sec_to_ticks(2500))
#
# sd.turn_on()

"""NEMA 17"""

#tmc = TMC2209_NEMA17.TMC2209()
#tmc.init_pins()
#tmc.turn_on()
#time.sleep_ms(100)


"""LED STRIPE"""
# leds = LED_Stripe.LED_Stripe()
#
# leds.init_pins()

"""DS18B20 (Temperature)"""
temp_sensor = DS18B20.DS18B20()
temp_sensor.init()

"""Fan Controller"""
fan_controller = Fan_Driver.Fan_Controller()
fan_controller.init()

"""SH1106_OLED"""
oled_controller = SH1106_OLED.SH1106_OLED()
oled_controller.init_i2c()

"""NeoPixel"""
np_controller = NeoPixel.NeoPixelController()
np_controller.init()

"""IRM_H6xxT_IR_SENSOR"""
ir_controller = IRM_H6xxT_IR_SENSOR.IR_Controller()
ir_controller.init()
ir_controller.set_callback()

"""Emergency Button"""
emergency_button = machine.ADC(machine.Pin(13))
emergency_button.atten(machine.ADC.ATTN_11DB)
def read_value():
    global emergency_button
    v = emergency_button.read_u16()
    if v < 45_000:
        print("ON")
    else:
        print("OFF")
"""Beeper"""
beeper = machine.PWM(machine.Pin(17))

def play_tone(frequency: int = 4_000, duty: int = 10, duration: int = -1):
    if frequency == 0:
        beeper.duty(0)
    else:
        beeper.freq(frequency)
        beeper.duty(duty)

    if duration > -1:
        time.sleep_ms(duration)
        beeper.duty(0)

play_tone(frequency=0)
play_tone(frequency=0)
play_tone(frequency=0)
play_tone(frequency=0)
c_p = 0
while True:
    temp_sensor.measure_temp()

    temps = temp_sensor.get_temp()


    oled_controller.test_oled()
    np_controller.test_led()

    press = ir_controller.get_last_press()
    if press is not None:
        print(press)

    read_value()

    fan_controller.set_fan_percentage(c_p)
    c_p += 0.05

    if c_p > 1:
        c_p = 0



