import sys
import time
import machine
import math
import _thread
# Add Path to Driver
sys.path.append('/Driver/')
import Kinematic_Engine
import PCA9685_Servo
import TMC2209_NEMA17
import LED_Stripe
import DS18B20
import Fan_Driver
import SH1106_OLED
import NeoPixel
import IRM_H6xxT_IR_SENSOR
import network_and_socket

class SIRIUS:
    def __init__(self):
        """Fan Controller"""
        self.fan_controller = Fan_Driver.Fan_Controller()
        self.fan_controller.init()
        self.fan_controller.set_fan_percentage(1)

        """SH1106_OLED"""
        self.oled_controller = SH1106_OLED.SH1106_OLED()
        self.oled_controller.init_i2c()
        self.oled_controller.draw_big_text(text=f"SIRIUS", x=0, y=20, scale=2.7)
        self.oled_controller.show_content()

        """SERVO"""
        self.sd = PCA9685_Servo.Servo_Driver()
        self.sd.init_i2c()

        self.sd.add_servo_config(0, 180, self.sd.sec_to_ticks(650), self.sd.sec_to_ticks(2600))
        self.sd.add_servo_config(1, 180, self.sd.sec_to_ticks(560), self.sd.sec_to_ticks(2520), inverted=True)
        self.sd.add_servo_config(2, 180, self.sd.sec_to_ticks(650), self.sd.sec_to_ticks(2600))
        self.sd.add_servo_config(3, 270, self.sd.sec_to_ticks(430), self.sd.sec_to_ticks(2500))
        self.sd.add_servo_config(4, 270, self.sd.sec_to_ticks(430), self.sd.sec_to_ticks(2500))
        self.sd.turn_on()

        """NEMA 17"""
        self.tmc = TMC2209_NEMA17.TMC2209()
        self.tmc.init_pins()
        self.tmc.turn_on()


        """LED STRIPE"""
        self.leds = LED_Stripe.LED_Stripe()
        self.leds.init_pins()

        """DS18B20 (Temperature)"""
        self.temp_sensor = DS18B20.DS18B20()
        self.temp_sensor.init()
        time.sleep_ms(10)
        self.temp_sensor.measure_temp()

        """NeoPixel"""
        self.np_controller = NeoPixel.NeoPixelController()
        self.np_controller.init()

        """IRM_H6xxT_IR_SENSOR"""
        self.ir_controller = IRM_H6xxT_IR_SENSOR.IR_Controller()
        self.ir_controller.init()

        """Emergency Button"""
        self.emergency_button = machine.ADC(machine.Pin(13))
        self.emergency_button.init(atten=machine.ADC.ATTN_11DB)

        """Beeper"""
        self.beeper = machine.PWM(machine.Pin(17))
        self.play_tone(frequency=4_000, duty=4)
        time.sleep_ms(200)
        self.play_tone(frequency=0)

        """Timer Var"""
        self.temperatur_timer = None
        self.emergency_button_timer = None

        """Temp Variables"""
        self.temp_address_place: dict = {
            b'(\x99\xefP\x00\x00\x00T': [0, "J1"],
            b'(\xc9gQ\x00\x00\x00\x00': [1, "J2"],
            b'(\xbaHR\x00\x00\x00\xde': [2, "J3"],
            b'(\xa3\xaeP\x00\x00\x00R': [3, "J4"],
            b'(\x8c%T\x00\x00\x00u': [4, "J5"],
            b'(\xc9\xf1T\x00\x00\x00\xaa': [5, "TMC2209"],
            b'(\xbf.R\x00\x00\x00\xb8': [6, "Kabel"],
        }
        self.all_temps: dict = {
            b'(\x99\xefP\x00\x00\x00T': 0,
            b'(\xc9gQ\x00\x00\x00\x00': 0,
            b'(\xbaHR\x00\x00\x00\xde': 0,
            b'(\xa3\xaeP\x00\x00\x00R': 0,
            b'(\x8c%T\x00\x00\x00u': 0,
            b'(\xc9\xf1T\x00\x00\x00\xaa': 0,
            b'(\xbf.R\x00\x00\x00\xb8': 0,
            "max": 0
        }

        # interface temp
        self.min_temp = 20
        self.second_temp = 30
        self.third_temp = 40
        self.max_temp = 50
        self.temp_difference = 10
        self.max_led_brightness = 20
        self.led_switch = 0
        self.measure_temp: bool = True

        """Interface Var"""
        self.menu_order = [
            [
                [
                    [["func normalized_vector_input"], "Normalisiert\nWerte"],
                    [["func real_vector_input"], "Reale\nWerte"],
                ],
            "Zielvektor\neingeben"
            ],

            [
                [
                    [["func change_angle_speed"], "Max.\nGeschwindigkeit"],
                    [["func change_movement_type"], "Bewegungsart"],
                ],
                "Motoren\nEinstellung"
            ],

            [
                [
                    [["func set_zero_position"], "0er Position"],
                    [["func set_rest_position"], "Ruhe\nPosition"],
                    [["func set_beautiful_position"], "Schaubild\nPosition"],
                    [["func recalibrate"], "Neu\nkalibrieren"],
                ],
                "Positionierung"
            ],

            [
                [
                    [["func deactivate_local_control"], "Lokale-Kontrolle\nabschalten"],
                    [["func switch_remote_control"], "Fern-Kontroller\nEin/Aus"],
                ],
                "Kontrolle"
            ],
        ]
        self.current_menu = [0, -1, -1]
        self.current_menu_str: str = self.menu_order[0][-1]
        self.last_current_menu_str: str = None

        # ir button map
        self.ir_button_address: dict = {
            0x7: "down",
            0x9: "up",

            0x44: "back",
            0x40: "enter",

            0x16: "0",
            0xc: "1",
            0x18: "2",
            0x5e: "3",
            0x8: "4",
            0x1c: "5",
            0x5a: "6",
            0x42: "7",
            0x52: "8",
            0x4a: "9",

            21: "VOL-",
        }

        """Kinematic"""
        self.kinematics = Kinematic_Engine.Kinematik_Engine()
        self.current_angle_try:float = 0
        self.angle_steps: float = 0.1

        """movement variables"""
        self.current_angles: list = [0, [0, 0, 0]]
        self.to_angle: list = self.current_angles
        self.angle_speed: float = 10 # °/s
        self.movement_type = "sinus" # quadrat, sinus
        self.move_set = "move" # "set" = Fastest with no tick block

        """Control"""
        self.allow_remote_control: bool = True
        self.allow_local_control: bool = True
        
        """Network and socket"""
        self.socket_thread = None
        self.socket_class = None
        self.socket_data_queue: list = [["angles", "move", [0, [10, 60, 0]]], ["angles", "set", [0, [0, 0, 0]]]]

        """Update Tick"""
        self.last_update_tick_time: float = time.ticks_ms()

    # Debug
    def Log(self, message: str, type: int = 0):
        ## type: 0 = LOG, 1 = Warning, 2 = Error
        print(f""
              f"[{time.ticks_ms() / 1000:0.1f}] "
              f"[{'INFO' if type == 0 else 'WARNING' if type == 1 else 'ERROR'}] "
              f"{message}"
              )

    """Init"""

    def start_init(self):
        TOTAL = 4

        # start all timer
        self.show_loading(message="Starte Timer", total=TOTAL, current=0)
        self.start_all_timer()

        # setting callbacks
        self.show_loading(message="Starte Callbacks", total=TOTAL, current=1)
        self.ir_controller.set_callback()

        # 0 positon
        self.show_loading(message="0er Position", total=TOTAL, current=2)
        self.to_angle = [0, [0, 0, 0]]
        self.move_set = "move"
        self.move_angles(None)

        # thread
        self.show_loading(message="Starte Thread", total=TOTAL, current=3)
        self.start_network_and_socket_thread()

        # End
        self.show_loading(message="Fertig!", total=TOTAL, current=TOTAL)
        # time.sleep_ms(500)

    """Interface Control"""

    # general controll

    def play_tone(self, frequency: int = 4_000, duty: int = 10, duration: int = -1):
        if frequency == 0:
            self.beeper.duty(0)
        else:
            self.beeper.freq(frequency)
            self.beeper.duty(duty)

        if duration > -1:
            time.sleep_ms(duration)
            self.beeper.duty(0)

    def read_emergency_button_value(self):
        v = self.emergency_button.read_u16()
        if v < 45_000:
            return True # ON
        else:
            return False # OFF

    def temp_to_rgb(self, temp):
        # Definition deiner Stützpunkte (Temperatur, (R, G, B))
        points = [
            (15, (0, 0, self.max_led_brightness)),
            (20, (0, self.max_led_brightness, 0)),
            (30, (self.max_led_brightness, self.max_led_brightness, 0)),
            (40, (self.max_led_brightness, 0, 0)),
            (50, (self.max_led_brightness * 2, 0, self.max_led_brightness)),
            (51, (255, 0, 255))
        ]

        if temp <= points[0][0]: return points[0][1]
        if temp >= points[-1][0]: return points[-1][1]

        for i in range(len(points) - 1):
            low_t, low_rgb = points[i]
            high_t, high_rgb = points[i + 1]

            if low_t <= temp <= high_t:

                fraction = (temp - low_t) / (high_t - low_t)

                r = int(low_rgb[0] + (high_rgb[0] - low_rgb[0]) * fraction)
                g = int(low_rgb[1] + (high_rgb[1] - low_rgb[1]) * fraction)
                b = int(low_rgb[2] + (high_rgb[2] - low_rgb[2]) * fraction)

                return (r, g, b)

    # Screens

    def x_center_text(self, text: str, char_size: int = 8):
        text_width = len(text) * char_size
        return int(max(0, int((128 - text_width) / 2)))

    def show_loading(self, message: str, total: int, current: int, message_x: int = 0, show_logo: bool = True):
        self.Log(message)
        # clear screen
        self.oled_controller.clear()

        # draw logo
        add_y = 10
        if show_logo:
            add_y = 25
            self.oled_controller.draw_big_text(text=f"SIRIUS", x=15, y=5, scale=2)

        # draw text
        self.oled_controller.draw_text(f"Lade ({current}/{total})...", 0, 45)
        self.oled_controller.draw_text(message, message_x, 55)

        # draw progress bar
        self.oled_controller.draw_rect(x=5, y=5 + add_y, w=116, h=10, fill=0)
        bar_progress = int(current / total * 112)
        self.oled_controller.draw_rect(x=7, y=7 + add_y, w=bar_progress, h=6, fill=1)

        # show
        self.oled_controller.show_content()

    def draw_menu_str(self):
        # draw the text centered
        self.oled_controller.clear()

        lines = self.current_menu_str.split("\n")
        total_lines = len(lines)

        for index, line_text in enumerate(lines):
            y_cord = (index * 15) +  ((64 - total_lines * 15) / (total_lines + 1) * (index + 1))

            char_width = 8
            text_width = len(line_text) * char_width
            x_cord = max(0, int((128 - text_width) / 2))

            self.oled_controller.draw_text(x=int(x_cord), y=int(y_cord), text=str(line_text))

        self.oled_controller.show_content()

    # input + Screen

    def integer_Input(self, info: str, dot_after: int = 1):
        # set structure
        structure_text = "A.BCD" if dot_after == 1 else "AB.CD"
        selected_value = [None, None, None, None, False] # last one is for positive (False) or negativ (True)
        letters = ["A", "B", "C", "D"]
        final_input = None

        # clear screen
        self.oled_controller.clear()

        # draw info text
        text_x_pose = self.x_center_text(info)
        self.oled_controller.draw_text(x=text_x_pose, y=1, text=info)


        # draw start value
        text_show = structure_text.replace("A", "_").replace("B", "").replace("C", "").replace("D", "")
        text_x_pose = self.x_center_text(text_show, char_size=16)
        self.oled_controller.draw_big_text(x=text_x_pose, y=40, text=text_show, scale=2)

        self.oled_controller.show_content()

        while final_input is None:
            # get button
            pressed_hex, pressed_str = self.get_pressed_button()

            if pressed_str is not None:
                # accept
                if pressed_str == "enter":
                    final_input_str = structure_text
                    # extract number
                    for i in range(len(selected_value) - 1):
                        v = selected_value[i] if selected_value[i] is not None else 0
                        final_input_str = final_input_str.replace(letters[i], str(v))
                    # final output
                    final_input = float(final_input_str)
                    # negativ if needed
                    if selected_value[4]:
                        final_input *= -1
                    continue

                # check if pressed button is num
                elif pressed_str in ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]:
                    # change text
                    for index in range(len(selected_value) - 1):
                        # save new value
                        if selected_value[index] == None:
                            selected_value[index] = pressed_str
                            break
                        else:
                            pass

                # back action
                elif pressed_str == "back":
                    # check if cancel / back
                    if selected_value[0:4] == [None, None, None, None]:
                        final_input = False
                        continue

                    # reverse list
                    selected_value.reverse()

                    # iterate through all values until first "not none"
                    for index in range(len(selected_value) - 1):
                        if selected_value[index + 1] is not None:
                            selected_value[index + 1] = None
                            selected_value.reverse()
                            break

                # negativ and positiv switch
                elif pressed_str == "VOL-":
                    selected_value[4] = not selected_value[4]

                # update text show
                text_show = structure_text
                for index in range(len(selected_value) - 1):
                    v = selected_value[index]

                    # add when v is not None
                    if v is not None:
                        text_show = text_show.replace(letters[index], str(v))
                    # replace first with "_" and rest with " "
                    else:
                        try:
                            text_show = text_show.replace(letters[index], "_")
                            for a in range(len(selected_value)):
                                text_show = text_show.replace(letters[index + a], "")
                            break
                        except IndexError:
                            break
                # add "-" if needed
                if selected_value[4]:
                    text_show = "-" + text_show


                # update oled
                text_x_pose = self.x_center_text(text_show, char_size=16)
                self.oled_controller.draw_rect(x=0, y=40, w=128, h=22, fill=True, color=0)
                self.oled_controller.draw_big_text(x=text_x_pose, y=40, text=text_show, scale=2)
                self.oled_controller.show_content()

            # run timers
            if self.measure_temp:
                self.read_and_analyse_temp()

            # sleep shortly
            time.sleep_ms(10)

        # end of input
        return final_input

    def vector_input(self, normalized: bool = True):
        # default value
        x, y, z = None, None, None

        # settings
        if normalized:
            dot_after = 1
        else:
            dot_after = 2

        # x Input
        while x is None:
            x = self.integer_Input(info="X-Wert", dot_after=dot_after)
            if x is False:
                continue

            # y Input
            while y is None:
                y = self.integer_Input(info="Y-Wert", dot_after=dot_after)
                if y is False:
                    x = None
                    y = None
                    break

                # z Input
                while z is None:
                    z = self.integer_Input(info="Z-Wert", dot_after=dot_after)
                    if z is False:
                        y = None
                        z = None
                        break

        if x is False:
            return False

        else:
            return x, y, z

    def input_vector_and_calculate(self, normalized: bool = True):
        # default Value
        accepted = False
        pressed_ir_button = None
        vector_i = None

        while not accepted:
            # get vector
            vector_i = self.vector_input(normalized)

            # return
            if vector_i is False:
                return False
            else:
                # show values
                self.oled_controller.clear()
                self.oled_controller.draw_text(x=self.x_center_text("Korrekt?"), y=5, text="Korrekt?")
                self.oled_controller.draw_text(x=10, y=20, text=f"X: {vector_i[0]}")
                self.oled_controller.draw_text(x=10, y=30, text=f"Y: {vector_i[1]}")
                self.oled_controller.draw_text(x=10, y=40, text=f"Z: {vector_i[2]}")

                self.oled_controller.show_content()

                # accept request
                while True:
                    _, pressed_ir_button = self.get_pressed_button()

                    if pressed_ir_button == "back":
                        break
                    elif pressed_ir_button == "enter":
                        accepted = True
                        break

                    # run timers
                    if self.measure_temp:
                        self.read_and_analyse_temp()

        # show loading
        self.oled_controller.clear()

        self.oled_controller.draw_text(x=self.x_center_text("Berechne..."), y=50, text="Berechne...")
        self.oled_controller.draw_rect(x=5, y=15, w=116, h=15, fill=0)

        self.oled_controller.show_content()

        self.Log(f"Starting vector calculation with vector {vector_i} ...")

        # set Fan to max
        self.fan_controller.set_fan_percentage(1)

        # search for positon
        angles = False
        self.current_angle_try = 0

        while self.current_angle_try < 360:

            # calculate angle (returns False if not possible)
            if normalized:
                angles = self.kinematics.calc_angle_by_normalized(vector_i[0], vector_i[1], vector_i[2],
                                                                  self.current_angle_try)
            else:
                angles = self.kinematics.calc_angle_by_real(vector_i[0], vector_i[1], vector_i[2], self.current_angle_try)

            # FOUND
            if angles is not False:
                break

            # show progress
            if self.current_angle_try % 10 < 0.1:
                bar_progress = int(self.current_angle_try / 360 * 112)
                self.oled_controller.draw_rect(x=7, y=17, w=bar_progress, h=11, fill=1)
                self.oled_controller.show_content()

            self.current_angle_try += self.angle_steps

        # check if angles have been found
        if angles is False:
            text = "Es konnte\nkeine valide\nPositionierung\ngefunden werden!"

            self.oled_controller.clear()

            for index, t in enumerate(text.split("\n")):
                self.oled_controller.draw_text(x=self.x_center_text(t), y=10 + index * 10, text=t)

            self.oled_controller.show_content()
            time.sleep(5)
            return False
        else:
            return angles

    def choose_menu(self, options: list[str], current_selected: int):
        last_selected: int = None

        self.oled_controller.clear()

        while True:
            # draw new on update
            if last_selected != current_selected:

                for i, t in enumerate(options):
                    y = (64 - len(options) * 8) // (len(options) + 1) * ( i + 1 ) + ( i * 8)
                    x = self.x_center_text(t)

                    # unselected options
                    if i != current_selected:
                        self.oled_controller.draw_rect(
                            x=x - 2,
                            y=y - 2,
                            w=len(t) * 8 + 4,
                            h=10, fill=True, color=0
                        )
                        self.oled_controller.draw_text(x=x, y=y, text=t)
                    # selected options
                    else:
                        self.oled_controller.draw_rect(
                            x=x - 2,
                            y=y - 2,
                            w=len(t) * 8 + 4,
                            h=10, fill=True, color=1
                        )
                        self.oled_controller.draw_text(x=x, y=y, text=t, color=0)
                    self.oled_controller.show_content()

                    last_selected = current_selected

            # run timers
            if self.measure_temp:
                self.read_and_analyse_temp()

            # update when input
            _, pressed_str = self.get_pressed_button()
            if pressed_str is not None:
                # up
                if pressed_str == "up":
                    current_selected -= 1
                    if current_selected == -1:
                        current_selected = len(options) - 1
                # down
                elif pressed_str == "down":
                    current_selected += 1
                    if current_selected == len(options):
                        current_selected = 0

                # accept
                if pressed_str == "enter":
                    return current_selected

                # back
                elif pressed_str == "back":
                    return False

    def slider(self, max_value: int, min_value: int, current_value: int, info_text: str = ""):
        last_value: int = None

        while True:

            # update screen on change
            if last_value != current_value:
                self.oled_controller.clear()
                # draw value
                self.oled_controller.draw_big_text(x=self.x_center_text(str(current_value), char_size=16),
                                                   y=10, text=str(current_value), scale=2)
                # draw infotext
                self.oled_controller.draw_text(x=self.x_center_text(info_text), y=28, text=info_text)

                # draw progress bar
                self.oled_controller.draw_rect(x=0, y=38, w=127, h=10, fill=False)
                progress_w = 123 * ((current_value - min_value) / (max_value - min_value))
                self.oled_controller.draw_rect(x=2, y=40, w=int(progress_w), h=6, fill=True)

                # draw min/max
                self.oled_controller.draw_text(x=0, y=50, text=f"|{min_value}")
                self.oled_controller.draw_text(x=128 - (len(f"{max_value}|") * 8), y=50, text=f"{max_value}|")

                # show
                self.oled_controller.show_content()

                last_value = current_value

            # update when input
            _, pressed_str = self.get_pressed_button()
            if pressed_str is not None:
                # up
                if pressed_str == "down":
                    current_value -= 1
                    if current_value == min_value - 1:
                        current_value = max_value
                # down
                elif pressed_str == "up":
                    current_value += 1
                    if current_value == max_value + 1:
                        current_value = min_value

                # accept
                if pressed_str == "enter":
                    return current_value

                # back
                elif pressed_str == "back":
                    return False

                # shortcuts
                elif pressed_str in ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]:
                    percentage = int(pressed_str) / 10
                    current_value = int(percentage * (max_value - min_value)) + min_value

            # run timers
            if self.measure_temp:
                self.read_and_analyse_temp()

    def ask_yes_no(self, question:str):
        total_length = len(question.split("\n"))

        self.oled_controller.clear()

        for i, t in enumerate(question.split("\n")):
            x_pos = self.x_center_text(t)
            y_pos = (64 - total_length * 8) / (total_length) * (i + 1) + (i * 8)
            self.oled_controller.draw_text(x=x_pos, y=int(y_pos), text=t)

        self.oled_controller.show_content()

        while True:
            _, pressed = self.get_pressed_button()

            if pressed is not None:
                if pressed == "enter":
                    return  True
                elif pressed == "back":
                    return False

            # run timers
            if self.measure_temp:
                self.read_and_analyse_temp()

    # input

    def move_by_vector_input(self,normalized: bool = True):
        # get angles
        angles = self.input_vector_and_calculate(normalized)

        if angles is False: # no valid angle
            return False
        else:
            self.oled_controller.clear()

            self.oled_controller.draw_text(x=self.x_center_text("Bewegen?"), y=10, text="Bewegen?")
            self.oled_controller.draw_text(x=1, y=25, text=f"Basis:    {angles[0]:0.1f}")
            self.oled_controller.draw_text(x=1, y=35, text=f"Gelenk 1: {angles[1][0]:0.1f}")
            self.oled_controller.draw_text(x=1, y=45, text=f"Gelenk 2: {angles[1][1]:0.1f}")
            self.oled_controller.draw_text(x=1, y=55, text=f"Gelenk 3: {angles[1][2]:0.1f}")


            self.oled_controller.show_content()

            while True:
                _, pressed = self.get_pressed_button()
                if pressed is not None:
                    if pressed == "enter":
                        self.move_set = "move"
                        self.to_angle = angles
                        self.auto_run_movement()
                        time.sleep_ms(500)
                        return False
                    elif pressed == "back":
                        return False

                # run timers
                if self.measure_temp:
                    self.read_and_analyse_temp()

    def get_pressed_button(self):
        pressed = self.ir_controller.get_last_press()
        if pressed is not None:
            # play tone
            self.play_tone(duration=50)
            # get key pressed as str
            try:
                pressed_str = self.ir_button_address[pressed]
            except KeyError:
                return None, None

            self.Log(f"IR-Input detected. str: {pressed_str}, hex: {pressed}")
            return pressed, pressed_str
        else:
            return None, None

    def navigate_menu(self):
        pressed, pressed_str = self.get_pressed_button()

        if pressed is not None:

            """action based on input"""
            # menu navigation
            if pressed_str == "up" or pressed_str == "down":
                dir = -1 if pressed_str == "up" else + 1

                # get, which menu
                if self.current_menu[1] == -1:
                    menu_num = 0
                elif self.current_menu[2] == -1:
                    menu_num = 1
                else:
                    menu_num = 2

                # set new menu
                self.current_menu[menu_num] += 1 * dir

                # update if menu is not available
                if menu_num == 0:
                    under_menu = self.menu_order
                elif menu_num == 1:
                    under_menu = self.menu_order[self.current_menu[0]][0]
                else:
                    under_menu = self.menu_order[self.current_menu[0]][self.current_menu[1]][0]


                if self.current_menu[menu_num] > len(under_menu) - 1:
                    self.current_menu[menu_num] = 0
                elif self.current_menu[menu_num] < 0:
                        self.current_menu[menu_num] = len(under_menu) - 1


            # back
            elif pressed_str == "back":
                if self.current_menu[1] == -1:
                    self.current_menu[0] = 0
                elif self.current_menu[2] == -1:
                    self.current_menu[1] = -1
                else:
                    self.current_menu[2] = -1

            # accept
            elif pressed_str == "enter":
                # [specials here]

                if self.current_menu[1] == -1:
                    self.current_menu[1] = 0
                elif self.current_menu[2] == -1:
                    self.current_menu[2] = 0

            # get current menu
            if self.current_menu[1] == -1:
                self.current_menu_str = self.menu_order[self.current_menu[0]][-1]
            elif self.current_menu[2] == -1:
                self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]
            else:
                self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][0][self.current_menu[2]]

    def run_selected_function(self, func):
        if func == "normalized_vector_input":
            self.move_by_vector_input(normalized=True)
            self.current_menu = [0, 0 ,-1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]

        elif func == "real_vector_input":
            self.move_by_vector_input(normalized=False)
            self.current_menu = [0, 1 ,-1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]

        elif func == "change_angle_speed":
            self.change_angle_speed()
            self.current_menu = [1, 0 ,-1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]

        elif func == "change_movement_type":
            self.change_movement_type()
            self.current_menu = [1, 1 ,-1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]

        elif func == "set_zero_position":
            self.set_position("zero")
            self.current_menu = [2, 0 ,-1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]

        elif func == "set_rest_position":
            self.set_position("rest")
            self.current_menu = [2, 1, -1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]

        elif func == "set_beautiful_position":
            self.set_position("beautiful")
            self.current_menu = [2, 2, -1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]

        elif func == "recalibrate":
            self.recalibrate_motors()
            self.current_menu = [2, 3, -1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]

        elif func == "deactivate_local_control":
            self.change_control_permission("local")
            self.current_menu = [3, 0, -1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]

        elif func == "switch_remote_control":
            self.change_control_permission("remote")
            self.current_menu = [3, 1, -1]
            self.current_menu_str = self.menu_order[self.current_menu[0]][0][self.current_menu[1]][-1]


        else:
            self.Log(f"Couldn't find function {func}", type=1)

    # loop

    def check_input_and_draw(self):
        # check for any new input
        self.navigate_menu()

        # menu change
        if self.last_current_menu_str != self.current_menu_str:
            # check if function is selected
            if self.current_menu_str.split(" ")[0] == "func":
                self.run_selected_function(self.current_menu_str.split(" ")[1])

            # draw menu (if needed)
            if self.allow_local_control:
                self.draw_menu_str()
                self.last_current_menu_str = self.current_menu_str

    """Timer"""

    def start_all_timer(self):
        # temp_timer
        self.temperatur_timer = machine.Timer(1)
        self.temperatur_timer.init(period=1500, mode=machine.Timer.ONE_SHOT, callback=self.Timer1)

        # emergenc button
        self.emergency_button_timer = machine.Timer(3) # Timer 2 is used for movement
        self.emergency_button_timer.init(period=100, mode=machine.Timer.ONE_SHOT, callback=self.Timer_3)

    def Timer1(self, timer):
        self.measure_temp = True

    def Timer_3(self, timer):
        # check if button is tuned off (= Emergency stop)
        if not self.read_emergency_button_value():
            self.emergency_stop()
        # check if temperature is too high
        elif self.all_temps["max"] >= self.max_temp:
            self.emergency_stop(intense_beep=True)

        self.emergency_button_timer.init(period=100, mode=machine.Timer.ONE_SHOT, callback=self.Timer_3)

    """Temperature"""

    def read_and_analyse_temp(self, allow_emergency_stop: bool = True):
        # get all temps and addresses
        temps = self.temp_sensor.get_temp()

        # reset max value
        self.all_temps["max"] = 0

        # iterate through all temps
        for temp in temps:
            # get data
            degree = temp[0]
            address = bytes(temp[1])
            id = self.temp_address_place[address][0]

            # get rgb value
            rgb_color = tuple(self.temp_to_rgb(degree))

            # set new color
            self.np_controller.set_individual_color(
                index=id,
                color=rgb_color,
                write_ins=False
            )

            # save value
            self.all_temps[address] = degree

            # save highest
            self.all_temps["max"] = max(self.all_temps["max"], degree)

        # write on/off (blink)
        self.led_switch = not self.led_switch
        self.np_controller.set_individual_color(
            index=7,
            color=(2, 2, 2) if self.led_switch else (0, 0, 0),
            write_ins=False
        )

        # write colors
        self.np_controller.write_color()

        # set fans
        if allow_emergency_stop:
            self.fan_controller.set_percentage_by_temp(self.all_temps["max"])

        if not allow_emergency_stop:
            # allow_emergency_stop is only True when emergency stop is currently running.
            self.temp_sensor.measure_temp()
            return

        # reset temperature timer
        self.temperatur_timer.init(period=1000, mode=machine.Timer.ONE_SHOT, callback=self.Timer1)
        self.measure_temp = False

        # measure temp for next time
        self.temp_sensor.measure_temp()

    # can also be used manually
    def emergency_stop(self, intense_beep: bool = False):
        # save what is shown before
        saved_display_buffer = bytearray(self.oled_controller.display.renderbuf)

        # fan to max
        self.fan_controller.set_fan_percentage(1)

        # show on display
        self.oled_controller.clear(1)

        self.oled_controller.draw_big_text(
            x=self.x_center_text("NOTAUS", char_size=16),
            y=24, text="NOTAUS", scale=2, color=0)

        self.oled_controller.show_content()

        # beep
        if intense_beep:
            for _ in range(4):
                self.play_tone(frequency=4_000, duration=300, duty=200)
                self.play_tone(frequency=4_000, duration=300, duty=10)
        else:
            self.play_tone(frequency=4_000, duration=200, duty=200)

        while not self.read_emergency_button_value() or self.all_temps["max"] >= self.third_temp:
            time.sleep_ms(750)
            self.read_and_analyse_temp(False)
            self.fan_controller.set_fan_percentage(1)

        # show reactivation with 5sec countdown
        for sec in range(5):
            self.oled_controller.clear(1)
            self.oled_controller.draw_text(x=12, y=14, text="Reaktivierung", color=0)
            self.oled_controller.draw_text(x=44, y=24, text="in...", color=0)
            self.oled_controller.draw_big_text(x=54, y=44, text=f"{-sec + 5}", scale=2, color=0)
            self.oled_controller.show_content()

            self.play_tone(frequency=4_000, duration=100, duty=200)

            time.sleep_ms(900)

        # reactivate
        self.tmc.turn_on()
        self.set_position("zero")

        # show old content
        self.oled_controller.display.renderbuf[:] = saved_display_buffer
        self.oled_controller.display.show(True)

    """Movement"""

    def auto_run_movement(self, start_delay: int = 500):
        # start timer, so it interrupt everything in the main thread to have a smooth movement.
        timer_instance = machine.Timer(2)
        timer_instance.init(period=start_delay, mode=machine.Timer.ONE_SHOT, callback=self.move_angles)

        # only when start delay is not 0
        if start_delay > 0:
            self.oled_controller.clear(1)

            text = "!ACHTUNG!\n\nBEWEGUNG\nSTARTET"

            for i, t in enumerate(text.split("\n")):
                self.oled_controller.draw_text(x=self.x_center_text(t), y=12 + i * 10, text=t, color=0)

            self.oled_controller.show_content()

    def move_angles(self, _):
        self.Log("Starting movement...")

        total_start_time: float = time.ticks_ms()

        # <editor-fold desc="BASE MOVEMENT">
        self.Log("Moving Stepper...")
        base_dir = -1 if self.to_angle[0] > self.current_angles[0] else 1
        self.tmc.set_dir(base_dir)
        time.sleep_ms(50)
        self.tmc.run_steps_by_angle(self.to_angle[0], self.current_angles[0], 2_500)
        # </editor-fold>


        # <editor-fold desc="SERVO MOVEMENT">
        self.Log("Moving Servo...")
        # calculate all distances
        angle_1_distance = self.to_angle[1][0] - self.current_angles[1][0]
        angle_2_distance = self.to_angle[1][1] - self.current_angles[1][1]
        angle_3_distance = self.to_angle[1][2] - self.current_angles[1][2]

        # get the biggest distance
        max_distance = max(abs(angle_1_distance), abs(angle_2_distance), abs(angle_3_distance))

        # get time
        time_needed = max_distance / self.angle_speed
        time_needed_ms = time_needed * 1_000


        # failsafe
        if time_needed <= 0:
            self.current_angles = self.to_angle
            time_needed = 1 # important to do not throw any error

        # get degree per seconds
        angle_1_degree_per_seconds = 0 if angle_1_distance == 0 else angle_1_distance / time_needed / 1000 # °/ms
        angle_2_degree_per_seconds = 0 if angle_2_distance == 0 else angle_2_distance / time_needed / 1000 # °/ms
        angle_3_degree_per_seconds = 0 if angle_3_distance == 0 else angle_3_distance / time_needed / 1000 # °/ms

        # direction
        angle_1_dir = -1 if self.to_angle[1][0] > self.current_angles[1][0] else 1

        # stepper step-speed
        step_speed = 1_000 # us

        # start values
        percentage = 0
        angle_1_angle: float = self.current_angles[1][0]
        angle_2_angle: float = self.current_angles[1][0]
        angle_3_angle: float = self.current_angles[1][0]


        def linear_percentage(t):
            p = t / time_needed_ms
            if p >= 1:
                return 1
            return p

        def quadrat_percentage(t):
            p = t / time_needed_ms
            if p >= 1:
                return 1
            return -(p - 1)**2 + 1

        def sinus_percentage(t):
            p = t / time_needed_ms
            if p >= 1:
                return 1
            return 0.5 * math.sin((p - 0.5) * (2 * math.pi / 2)) + 0.5

        start_time: float = time.ticks_ms()
        while self.current_angles != self.to_angle:

            # get percentage
            if self.movement_type == "linear":
                percentage = linear_percentage((time.ticks_ms() - start_time))
            elif self.movement_type == "quadrat":
                percentage = quadrat_percentage((time.ticks_ms() - start_time))
            else:
                percentage = sinus_percentage((time.ticks_ms() - start_time))

            # calculate a new angle based on mode
            angle_1_angle = self.current_angles[1][0] + angle_1_distance * percentage
            angle_2_angle = self.current_angles[1][1] + angle_2_distance * percentage
            angle_3_angle = self.current_angles[1][2] + angle_3_distance * percentage

            # check if destination reached
            if percentage >= 1:
                self.current_angles = self.to_angle

            # update positon servo
            self.sd.set_servo_to(num=0, degree=angle_1_angle)
            self.sd.set_servo_to(num=1, degree=angle_2_angle)
            self.sd.set_servo_to(num=2, degree=angle_3_angle)

            # run timers
            if self.measure_temp:
                self.read_and_analyse_temp()


        self.Log(f"Movement finished in :{time.ticks_ms() - total_start_time} ms")
        # </editor-fold>

    def change_movement_type(self):
        new_type = self.choose_menu(
            options=["Linear", "Sinus", "Quadratisch"],
            current_selected=0 if self.movement_type == "linear" else 1 if self.movement_type == "sinus" else 2
        )
        if new_type is not False:
            self.movement_type = \
                "linear" if new_type == 0 \
                else "sinus" if new_type == 1 else \
                    "quadrat"

    def change_angle_speed(self):
        new_speed = self.slider(max_value=180, min_value=1, current_value=self.angle_speed, info_text="Grad/Sekunde")
        if new_speed is not False:
            self.angle_speed = new_speed

    def set_position(self, pos:str):
        self.Log(f"Set to pos: {pos}")
        self.move_set = "move"

        if pos == "zero":
            self.to_angle = [0, [0, 0, 0]]
        elif pos == "rest":
            self.to_angle = [0, [60, 60, 0]]
        elif pos == "beautiful":
            self.to_angle = [0, [30, 30, 30]]

        self.auto_run_movement(start_delay=250)
        time.sleep_ms(300)

    def recalibrate_motors(self):
        self.Log("Starting recalibration...")

        # stop Motor 2
        self.to_angle = [0, [60, 0, 60]]
        self.move_set = "move"

        self.move_angles(True)

        self.sd.set_channel_duty(channel_num=1, duty_num=0)
        self.tmc.turn_off()

        # show info
        self.oled_controller.clear(1)
        i_t = "Bewege den\nArm zur\nMarkierung.\nDruecke dann\neinen beliebigen\n-Knopf-"
        for i, t in enumerate(i_t.split("\n")):
            x_pos = self.x_center_text(t)
            y_pos = 10 * i + 2
            self.oled_controller.draw_text(x=x_pos, y=y_pos, text=t, color=0)

        self.oled_controller.show_content()

        # waiting for interaction
        while True:
            _, pressed = self.get_pressed_button()
            if pressed is not None:
                break

            # run timers
            if self.measure_temp:
                self.read_and_analyse_temp()

        # set to nice position
        self.tmc.turn_on()
        self.set_position("beautiful")

        self.oled_controller.clear()
        self.oled_controller.draw_text(x=self.x_center_text("Rekalibrierung"), y=20, text="Rekalibrierung")
        self.oled_controller.draw_text(x=self.x_center_text("erfolgreich"), y=30, text="erfolgreich")
        self.oled_controller.show_content()

        self.Log("Recalibration finished")

        time.sleep_ms(1_500)
        # deactivate Servos
        for n in range(4):
            self.sd.set_channel_duty(channel_num=n, duty_num=0)
        # deactivate Steppers
        self.tmc.turn_off()

    def set_servo_angles_instant(self):
        self.sd.set_servo_to(num=0, degree=self.to_angle[1][0])
        self.sd.set_servo_to(num=1, degree=self.to_angle[1][0])
        self.sd.set_servo_to(num=2, degree=self.to_angle[1][0])

        # update positon stepper
        base_dir = -1 if self.to_angle[0] > self.current_angles[0] else 1
        self.tmc.set_dir(base_dir)

        self.tmc.run_steps_by_angle(self.to_angle[0] , self.current_angles[0], 1_000)

    def move_one_tick(self, d_time):

        for a in range(3):
            # get angle
            c_angle = self.current_angles[1][a]
            t_angle = self.to_angle[1][a]

            # check if already done
            if c_angle == t_angle:
                continue

            # direction
            angle_dir = 1 if t_angle > c_angle else -1
            # new angle
            n_angle = c_angle + self.angle_speed * d_time * angle_dir

            # check if reached
            if n_angle >= t_angle and angle_dir == 1:
                self.current_angles[1][a] = self.to_angle[1][a]
            elif n_angle <= t_angle and angle_dir == -1:
                self.current_angles[1][a] = self.to_angle[1][a]
            else:
                self.current_angles[1][a] = n_angle
            # move
            self.sd.set_servo_to(num=a, degree=n_angle)



    """Control"""

    def change_control_permission(self, which:str):
        if which == "local":
            q = "Soll die\nLokale Kontrolle\nabgeschaltet\nwerden?"
        else:
            q = f"Willst du die\nFern-Kontrolle\n{'abschalten' if self.allow_remote_control else 'einschalten'}?"

        r = self.ask_yes_no(q)

        if which == "local" and r:
            self.Log(f"Change permission of 'local control' to False")
            self.allow_local_control = False

            # show on screen
            m1 = "Lokale Kontroller"
            m2 = "abgeschaltet"
            self.oled_controller.clear(1)
            self.oled_controller.draw_text(x=self.x_center_text(m1), y=22, text=m1, color=0)
            self.oled_controller.draw_text(x=self.x_center_text(m2), y=32, text=m2, color=0)
            self.oled_controller.show_content()

            self.play_tone(duration=200)

            self.oled_controller.set_contrast(1)


        elif r:
            self.Log(f"Change permission of 'remote control'. Old: {self.allow_remote_control}")

            self.allow_remote_control = not self.allow_remote_control

            self.Log(f"New: {self.allow_remote_control}")

    """Network"""
    
    def start_network_and_socket_thread(self):
        self.Log("Starting network and socket thread")
        self.socket_class = network_and_socket.network_and_socket(logging_func=self.Log,
                                                                  add_data_queue=self.add_to_data_queue)

        _thread.start_new_thread(self.socket_class.thread_loop, ())

    def add_to_data_queue(self, new_data):
        self.socket_data_queue.append(new_data)

    def run_data_queue(self):
        data_queue_copy = self.socket_data_queue.copy()

        # check if something is in data queue
        if len(data_queue_copy) > 0:
            data_to_process = data_queue_copy[0]

            # set new angles
            if data_to_process[0] == "angles":
                # instant
                if data_to_process[1] == "set":
                    self.to_angle = data_to_process[2]
                    self.move_set = "set"
                # move
                elif data_to_process[1] == "move":
                    self.to_angle = data_to_process[2]
                    self.move_set = "move"

            # delete first
            self.socket_data_queue.pop(0)

    """Auto mode"""

    def run_everything(self):
        self.last_update_tick_time = time.ticks_ms()
        time.sleep_ms(1)
        while True:
            # get tick time
            d_time = time.ticks_ms() - self.last_update_tick_time
            self.last_update_tick_time = time.ticks_ms()


            # run inputs
            if self.allow_local_control:
                self.check_input_and_draw()

            self.run_data_queue()

            # run movement
            if self.to_angle != self.current_angles:

                # smooth movement
                if self.move_set == "move":
                    self.auto_run_movement(start_delay=200)
                    time.sleep_ms(203)
                    self.last_current_menu_str = ""
                    self.last_update_tick_time = time.ticks_ms()

                # linear "set" movement
                elif self.move_set == "set":
                    self.move_one_tick(d_time / 1_000)

                # not found
                else:
                    self.Log(f"No move_set called {self.move_set}", 1)
                    self.move_set = "move"

            # run timers
            if self.measure_temp:
                self.read_and_analyse_temp()

            time.sleep_ms(max(0, 5 - d_time))


if __name__ == '__main__':
    sirius_controller = SIRIUS()
    sirius_controller.start_init()
    sirius_controller.run_everything()
