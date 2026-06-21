import time
import math

import json
from PySide6.QtWidgets import QMainWindow, QWidget, QPushButton, QApplication, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QButtonGroup, QLineEdit,QStackedWidget,QSizePolicy, QFileDialog, QMessageBox, QProgressDialog
import sys
from qt_material import apply_stylesheet
import os
from PySide6.QtCore import QPropertyAnimation, Property, QEasingCurve, Qt, QThread, QTimer, QRunnable, QObject, Signal, QThreadPool, Slot, QMutex, QSize, QRectF, QPointF, QLocale
from PySide6.QtGui import QFontDatabase, QPainter, QColor, QPen, QBrush,QDoubleValidator
from inputs import get_gamepad, UnpluggedError
import datetime
import Kinematic_Engine
import socket
from PySide6.QtSvgWidgets import QSvgWidget

def load_stylesheet(file_path):
    abs_path = os.path.abspath(__file__).replace("main.py", "")

    with open(f"{abs_path}/{file_path}", "r") as f:
        return f.read()

class Animated_Field(QWidget):
    def __init__(self, name):
        super().__init__()
        self.setObjectName(name)
        self.setAttribute(Qt.WA_StyledBackground, True)

        self.margin = 10
        self.setContentsMargins(self.margin, self.margin, self.margin, self.margin)

        self.ani = QPropertyAnimation(self, b"dynamic_margin")
        self.ani.setDuration(300)
        self.ani.setEasingCurve(QEasingCurve.InOutQuad)

    def get_margin(self):
        return self.margin

    def set_margin(self, var):
        self.margin = var
        self.setContentsMargins(self.margin, self.margin, self.margin, self.margin)

    dynamic_margin = Property(int, get_margin, set_margin)

    def enterEvent(self, event):
        self.animate_margin(5)
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.animate_margin(10)
        super().leaveEvent(event)

    def animate_margin(self, target_margin):
        self.ani.stop()
        self.ani.setEndValue(target_margin)
        self.ani.start()

class ControllerInput(QThread):
    controller_signal = Signal(str, str, int)
    controller_logging_output = Signal(int, str, int)

    def run(self):
        one_time = True
        DEADZONE = 6_000
        SCALE = 32768.0
        time.sleep(0.5)
        try:
            while True:
                if one_time:
                    self.controller_logging_output.emit(1, f"Verbindung zum Kontroller hergestellt", 0)
                    one_time = False

                events = get_gamepad()
                for e in events:
                    if e.ev_type != "Sync":
                        # dead-zone for joystick
                        if e.code in ["ABS_X", "ABS_Y", "ABS_RX", "ABS_RY"]:
                            val = e.state
                            if abs(val) < DEADZONE:
                                val = 0
                            else:
                                direction = 1 if val > 0 else -1
                                val = (abs(val) - DEADZONE) / (SCALE - DEADZONE) * direction * 100

                            self.controller_signal.emit(e.ev_type, e.code, int(val))
                        else:
                            self.controller_signal.emit(e.ev_type, e.code, e.state)

        except UnpluggedError as e:
            self.controller_logging_output.emit(0, f"Verbindung zum Kontroller getrennt", 1)
        except Exception as e:
            self.controller_logging_output.emit(0, f"Es trat ein Fehler auf: {e}", 1)

class WorkerSignal(QObject):
    finished = Signal(object, str, object)
    error = Signal(str, int)

class CalculateNewAngles(QRunnable):
    def __init__(self, data_to_process, kinematics, move_type):
        super().__init__()
        self.data = data_to_process
        self.signals = WorkerSignal()
        self.kinematics = kinematics
        self.move_type = move_type

    @Slot()
    def run(self):
        try:
            result = self.kinematics.calc_angle_by_normalized(self.data[0], self.data[1], self.data[2])

            self.signals.finished.emit(result, self.move_type, self.data)

        except Exception as e:
            self.signals.error.emit(str(e), 2)

class SocketThread(QThread):
    log_signal = Signal(str, int)
    socket_status = Signal(int, int) # 1. client status, 2. server status
    server_ping = Signal(float)

    def __init__(self):
        super().__init__()
        self.mutex = QMutex()
        self.current_data = None
        self.data_send = None
        self.is_running = True
        self.save_send_data_active: bool = False

        self.ping_test_delay = 2
        self.last_ping_test = time.time()

        self.last_data_feed = time.time()
        self.data_received = [False, 0] # [is received, start time]

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = ('10.42.0.62', 4242)
        self.socket.bind(("0.0.0.0", 4242))
        self.socket.settimeout(0.5)

        self.server_is_offline = True

    def feed_data(self, new_data):
        self.mutex.lock()
        self.current_data = new_data
        self.mutex.unlock()

    def stop(self):
        self.is_running = False

    def save_send_data(self, new_data):
        self.save_send_data_active = True
        self.mutex.lock()
        self.current_data = new_data
        self.mutex.unlock()

    def send_data(self, new_data, check_if_received: bool = True):
        self.socket.sendto(new_data.encode(), self.server_address)

        if not check_if_received:
            return

        self.data_received[1] = time.time()

        if self.save_send_data_active:
            while True:
                try:
                    data, addr = self.socket.recvfrom(1024)
                    self.data_received[0] = True
                    break
                except OSError:
                    self.socket.sendto(new_data.encode(), self.server_address)
                    continue  # message is not reached and has to be re-sent
            self.save_send_data_active = False
        else:
            while not self.data_received[0] and time.time() - self.data_received[1] <= 3.0:
                try:
                    data, addr = self.socket.recvfrom(1024)
                    self.data_received[0] = True
                    break
                except OSError:
                    self.socket.sendto(new_data.encode(), self.server_address)
                    continue # message is not reached and has to be re-sent

        self.last_ping_test = time.time() #  otherwise check_server_status won't work

        if self.data_received[0]:
            self.data_received[0] = False
            return
        else:
            self.socket_status.emit(1, 0)
            self.server_ping.emit(3_000)
            self.server_is_offline = True
            self.log_signal.emit(f"Verbindung zum Server fehlgeschlagen", 1)

    def check_server_status(self):
        start_time = time.perf_counter()

        self.send_data("ping", False)

        while time.perf_counter() - start_time <= 2:
            try:
                data, addr = self.socket.recvfrom(1024)
                break
            except TimeoutError:
                pass

        if time.perf_counter() - start_time >= 2:
            self.socket_status.emit(1, 0)
            self.server_ping.emit(2_000)
            self.server_is_offline = True
            self.log_signal.emit(f"Verbindung zum Server fehlgeschlagen", 1)
            return

        # get delay
        delay = time.perf_counter() - start_time
        self.socket_status.emit(1, 1)
        self.server_ping.emit(int(delay * 1_000 - 1))

        if self.server_is_offline: #  to prevent multiple
            self.log_signal.emit(f"Verbindung zum Server hergestellt", 0)

        self.server_is_offline = False

    def run(self):
        self.log_signal.emit("Starte Socket Thread", 0)

        self.socket_status.emit(1, -1)

        while self.is_running:
            # check if server is offline
            if self.server_is_offline:
                self.check_server_status()

            try:
                # get data
                self.mutex.lock()
                data_to_send = self.current_data
                self.mutex.unlock()

                # check if data is new
                if self.data_send != data_to_send:
                    self.send_data(data_to_send)

                    self.data_send = data_to_send

                    self.last_data_feed = time.time()

                elif time.time() - self.last_data_feed >= 0.2 and time.time() - self.last_ping_test >= self.ping_test_delay:
                    self.check_server_status()
                    self.last_ping_test = time.time()

                # sleep
                time.sleep(0.05)

            # ERROR
            except Exception as e:
                self.log_signal.emit(f"Es trat ein Fehler im Socket Thread auf: {e}", 2)
                self.socket_status.emit(0, 0)
                time.sleep(1)
        # stop
        self.socket.close()

class AspectRatioSvgWidget(QSvgWidget):
    def __init__(self, file_path, ratio=1.0):
        super().__init__(file_path)
        self.ratio = ratio # h / w

        size_policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        size_policy.setHeightForWidth(True)
        self.setSizePolicy(size_policy)

    def heightForWidth(self, width):
        return int(width * self.ratio)

    def sizeHint(self):
        w = self.width()

        return QSize(w, int(w * self.ratio))

class RobotArmCanvas(QWidget):
    def __init__(self, is_active: bool = True, is_static: bool = False):
        super().__init__()

        self.is_active = is_active
        self.is_static = is_static
        self.first_time: bool = True

        self.angles = [90, [0.1, 29.8, 60.5]]
        self.vector = [0, 0.62, 0.5]

        # color
        self.yellow = QColor(255, 255, 0, 255)
        self.transparent_yellow = QColor(255, 255, 0, 100)
        self.orange = QColor(255, 125, 0, 255)

        # dimensions
        self.c_width, self.c_height = self.width(), self.height()

        self.segment_length = 200
        self.ankle_radius = [30, 35, 30]

        # for draw
        self.start_point = [self.c_width // 2, 0]
        self.segment_cords: list = []

        self.base_start_point = [self.c_width // 4, self.c_height // 2]
        self.base_dimensions = [150, 15] # LENGTH, TRIANGLE

        self.vector_dimension = [5, 10]

        self.vector_pos = [[0, 0], [0, 0]]

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.game_tick)
        if self.is_static:
            self.timer.start(2_000)
        else:
            self.timer.start(16)

    def get_cord_by_angle(self, angle, length):

        angle = angle * (math.pi / 180)
        x = math.sin(angle) * length
        y = math.cos(angle) * length
        return [int(-x), int(y)]

    def draw_x_at_pos(self, painter, pos):
        x1 = pos[0] + self.vector_dimension[1] // 2
        x2 = pos[0] - self.vector_dimension[1] // 2
        y1 = pos[1] + self.vector_dimension[1] // 2
        y2 = pos[1] - self.vector_dimension[1] // 2
        painter.drawLine(x1, y1, x2, y2)
        painter.drawLine(x1, y2, x2, y1)

    def game_tick(self):

        if not self.is_active:
            return

        # <editor-fold desc="ARMS">
        # dimensions and start
        self.c_width, self.c_height = self.width(), self.height()
        self.start_point = [self.c_width // 2, self.ankle_radius[1]]

        # delete everything before
        self.segment_cords = []
        # add start pos
        self.segment_cords.append(self.start_point)

        # add all other pos
        for i in range(3):
            
            total_angle = sum(self.angles[1][0:i+1])

            n_cords = self.get_cord_by_angle(total_angle, self.segment_length)

            with_before = [n_cords[0] + self.segment_cords[i][0],
                           n_cords[1] + self.segment_cords[i][1]]
            self.segment_cords.append(with_before)

        # add vector
        normal_distance_to_base = min(math.sqrt(self.vector[0]**2 + self.vector[2] ** 2), 1)

        self.vector_pos[0] = [
            self.start_point[0] - normal_distance_to_base * self.segment_length * 3,
            self.start_point[1] + self.vector[1] * self.segment_length * 3
        ]

        # </editor-fold>

        # <editor-fold desc="BASE">
        self.base_start_point = [self.c_width - self.c_width // 4, self.c_height // 2]

        self.vector_pos[1] = [
            self.base_start_point[0] + self.vector[2] * self.base_dimensions[0],
            self.base_start_point[1] - self.vector[0] * self.base_dimensions[0]
        ]
        # </editor-fold>

        self.update()

    def paintEvent(self, event, /):
        painter = QPainter(self)

        # flip along X
        painter.save()
        painter.translate(self.width(), self.height())
        painter.scale(-1, -1)

        # AA
        painter.setRenderHint(QPainter.Antialiasing)

        """DRAW"""
        # BG
        painter.fillRect(self.rect(), QColor("#1a1a1a"))

        # set pen
        normal_pen = QPen(self.yellow)
        normal_pen.setWidth(3)

        line_pen = QPen(self.yellow)
        line_pen.setWidth(self.ankle_radius[2])
        line_pen.setCapStyle(Qt.RoundCap)

        thin_line_pen = QPen(self.yellow)
        thin_line_pen.setWidth(1)

        background_pen = QPen(QColor("#1a1a1a"))
        background_pen.setWidth(3)
        background_brush = QBrush(QColor("#1a1a1a"))

        vector_pen = QPen(self.orange)
        vector_pen.setWidth(self.vector_dimension[0])
        vector_pen.setCapStyle(Qt.RoundCap)

        painter.setPen(normal_pen)

        brush = QBrush(self.transparent_yellow)
        painter.setBrush(brush)

        # <editor-fold desc="DRAW ARMS">
        """draw arms"""
        painter.setPen(line_pen)
        for i in range(3):
            p1 = self.segment_cords[i]
            p2 = self.segment_cords[i + 1]
            painter.drawLine(p1[0], p1[1], p2[0], p2[1])

        # Draw ankles
        ## 1. remove circle
        painter.setPen(background_pen)
        painter.setBrush(background_brush)
        for i in range(3):
            x, y = self.segment_cords[i]
            painter.drawEllipse(
                QPointF(x, y),
                self.ankle_radius[1], self.ankle_radius[1])

        ## draw ankle as circle
        painter.setPen(normal_pen)
        painter.setBrush(brush)
        for i in range(3):
            x, y = self.segment_cords[i]
            r = self.ankle_radius[0]
            point = QPointF(x, y)
            painter.drawEllipse(point, r, r)

            ## draw description
            painter.save()
            painter.translate(x, y)
            painter.scale(-1, -1) # to negate the flip of the canvas

            text_rect = QRectF(-r, -r, 2 * r, 2 * r)

            if self.is_static:
                painter.drawText(text_rect, Qt.AlignCenter, f"a{i}")
            else:
                painter.drawText(text_rect, Qt.AlignCenter, f"{int(self.angles[1][i])}°")

            painter.restore()
        # </editor-fold>

        # <editor-fold desc="DRAW BASIS">
        """draw basis"""
        ## lines
        painter.setPen(thin_line_pen)
        p1 = [
            [self.base_start_point[0] - self.base_dimensions[0], self.base_start_point[1]],
            [self.base_start_point[0], self.base_start_point[1] - self.base_dimensions[0]]
        ]
        p2 = [
            [self.base_start_point[0] + self.base_dimensions[0], self.base_start_point[1]],
            [self.base_start_point[0], self.base_start_point[1] + self.base_dimensions[0]]
        ]
        for i in range(len(p1)):
            painter.drawLine(p1[i][0], p1[i][1], p2[i][0], p2[i][1])

        ## arc
        rect = QRectF(p1[0][0], p1[1][1], self.base_dimensions[0] * 2, self.base_dimensions[0] * 2)
        start_angle = -self.angles[0] + 360 + 90
        span_angle = self.angles[0]
        painter.drawPie(rect, int(start_angle * 16), int(span_angle * 16)) # Qt uses mor steps per degree

        ## arrow at bottom
        painter.drawPolygon([QPointF(self.base_start_point[0], p1[1][1]),
                             QPointF(self.base_start_point[0] + self.base_dimensions[1], p1[1][1] - self.base_dimensions[1] // 0.7),
                             QPointF(self.base_start_point[0] - self.base_dimensions[1], p1[1][1] - self.base_dimensions[1] // 0.7)])

        ## description
        painter.save()
        point_to_write = [self.base_start_point[0], p1[1][1] - self.base_dimensions[1] // 0.7 + 5]
        painter.translate(point_to_write[0], point_to_write[1])
        painter.scale(-1, -1) # to negate the flip of the canvas

        text_rect = QRectF(-self.base_dimensions[1] - 10, 0, 2 * self.base_dimensions[1] + 20, 2 * 20)
        if self.is_static:
            painter.drawText(text_rect, Qt.AlignCenter, f"Basis")
        else:
            painter.drawText(text_rect, Qt.AlignCenter, f"{int(self.angles[0])}°")

        painter.restore()

        # </editor-fold>

        # <editor-fold desc="VECTOR">
        painter.setPen(vector_pen)
        point_base = (self.vector_pos[1][0], self.vector_pos[1][1])
        point_arm = (self.vector_pos[0][0], self.vector_pos[0][1])
        self.draw_x_at_pos(painter, point_arm)
        self.draw_x_at_pos(painter, point_base)
        # </editor-fold>

        painter.end()

class MainWindow(QMainWindow):
    new_vector = Signal(float, float, float, str)
    new_angles = Signal(list, str)

    def __init__(self):
        super().__init__()
        self.setWindowFlags(Qt.FramelessWindowHint)

        """GUI VAR"""
        # MAIN
        self.middle_separation_layout = None
        self.middle_separation = None
        self.bottom_separation_layout = None
        self.bottom_separation = None
        self.main_layout = None
        self.central_widget = None
        # FIELDS
        self.field_1_layout = None
        self.field_1 = None
        self.field_2_layout = None
        self.field_2 = None
        self.field_3_layout = None
        self.field_3 = None
        self.field_4_layout = None
        self.field_4 = None
        self.field_5_layout = None
        self.field_5 = None

        # ALL FIELD 1
        self.title_label = None
        self.perspective_buttons = None
        self.perspective_button_group = None

        # ALL FIELD 2
        self.title_label_field2 = None
        self.controller_connection_btn = None
        self.controller_connection_label = None
        self.controller_control_type_button: list = []
        self.vector_label_title = None
        self.vector_label_normalized_title = None
        self.vector_label_normalized = None
        self.vector_label_real_title = None
        self.vector_label_real = None
        self.exit_button = None
        self.vector_input_label = None
        self.vector_input_fields = None
        self.vector_input_title = None
        self.vector_input_btn = None
        self.vector_input_type: list = []

        # ALL FIELD 3
        self.title_label_field3 = None
        self.angle_labels_title = None
        self.angle_labels_name = None
        self.angle_labels_values = None
        self.socket_status_title = None
        self.socket_status_names = None
        self.socket_status_status = None
        self.movement_sequence_title = None
        self.movement_sequence_start_button = None
        self.movement_sequence_file_name_button = None
        self.movement_sequence_save_pos_button = None
        self.movement_sequence_file_name_lable = None

        # ALL FIELD 4
        self.logging_label = None

        # ALL FIELD 5 — STACKED WIDGET
        self.perspective_tab = None
        self.perspective_layout = None
        self.orthogonal_tab = None
        self.orthogonal_layout = None
        self.animated_tab = None
        self.animated_layout = None
        self.static_tab = None
        self.static_layout = None
        self.field5_stacked_widget = None
        # PERSPECTIVE
        # ORTHOGONAL
        # 2D-ANIMATED
        self.robot_arm_canvas = None
        # 2D-STATIC
        self.static_svg_widget = None

        """GENERAL VAR"""
        self.version = "0.1.0 BETA"
        self.setWindowTitle(f"S.I.R.I.U.S. Control - {self.version}")
        self.base_path = os.path.dirname(os.path.abspath(__file__))
        self.robot_arm_svg_path = fr"{self.base_path}/assets/img/arm.svg"

        # For GUI
        self.perspective_selected = "2d-animated" # orthogonal, 2d-animated, 2d-static, perspektive
        self.all_perspectives: dict = ["perspektive", "orthogonal", "2d-animated", "2d-static"]

        # FOR VECTOR
        self.current_vector: list = [0.5, 0.6, 0.2]
        self.old_vector: list = self.current_vector.copy()
        self.current_angles: list = [0, [0, 0, 0]] # [base_rotation, joint_rotation]
        self.gripper_pos: list = [0, 0]

        # FOR SEQUENCE
        self.selected_sequence_file: str = ""
        self.current_sequence: list = []

        """CALCULATIONS"""
        self.kinematics = Kinematic_Engine.Kinematik_Engine(total_length=45)
        self.kinematics.max_length = 1
        self.kinematics.angle_change = 0.1

        # <editor-fold desc="SETUP">
        """SETUP"""
        self.load_all_fonts()
        self.setup_gui()
        self.connect_signals()

        """THREADS"""
        self.threadpool_kinematics = QThreadPool.globalInstance()
        self.log(f"Multithreading mit maximal {self.threadpool_kinematics.maxThreadCount()} Threads.")
        self.worker_is_busy: bool = False
        self.calculated_vector: list = [0, 0, 0]

        self.socket_thread = SocketThread()
        self.socket_thread.log_signal.connect(self.log)
        self.socket_thread.socket_status.connect(self.update_socket_status)
        self.socket_thread.server_ping.connect(self.update_ping_label)
        self.socket_thread.start()

        """--- CONTROLLER INTEGRATION ---"""
        self.controller_is_connected = 0 # -1 = not active, 0 = connected, 1 = active
        self.controller_speed: float = 0.1
        self.controller_delta_time: float = -1
        self.last_controller_joystick_input: list = [0, 0, 0, 0]
        self.current_button_pressed: set = set()
        self.controller_control_type: int = 0 # relative (1) or absolute (0)

        self.loop_joystick_input = QTimer(self)
        self.loop_joystick_input.timeout.connect(self.new_vector_by_joystick)
        self.loop_joystick_input.setInterval(10)
        self.loop_joystick_input.start()

        self.loop_controller_button_pressed = QTimer(self)
        self.loop_controller_button_pressed.timeout.connect(self.loop_button_pressed)
        self.loop_controller_button_pressed.setInterval(10)
        self.loop_controller_button_pressed.start()

        self.controller_thread = ControllerInput()
        self.controller_thread.controller_signal.connect(self.handle_controller_input)
        self.controller_thread.controller_logging_output.connect(self.change_controller_active)
        self.controller_thread.start()

        """DEFAULT POSITON"""
        self.set_to_pos(0)
        # </editor-fold>

    """LOGGING"""

    def log(self, text: str, type: int = 0):
        c_date = datetime.datetime.now().strftime("%d.%m.%Y %H:%M:%S")
        type_str= "LOG" if type == 0 else "WARNUNG" if type == 1 else "FEHLER"
        message = text

        full_str = f"[{c_date}][{type_str}] {message}"

        print(full_str)

        if self.logging_label is not None:
            self.logging_label.setText(f"> {full_str}")
            self.logging_label.setProperty("type", type_str[0])
            self.logging_label.style().unpolish(self.logging_label)
            self.logging_label.style().polish(self.logging_label)

    """SETUP"""

    def connect_signals(self):
        self.new_vector.connect(self.update_all_vectors)
        self.new_angles.connect(self.update_all_angles)

    def setup_gui(self):
        """INIT FUNC"""
        """MAIN"""
        # main_widget
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        """LAYOUTS"""
        # set  main layouts
        self.main_layout = QVBoxLayout(self.central_widget)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        # Field 1
        self.field_1 = Animated_Field("field")
        self.field_1_layout = QGridLayout(self.field_1)

        # separation bottom
        self.bottom_separation = QWidget()
        self.bottom_separation_layout = QHBoxLayout(self.bottom_separation)

        # Field 2
        self.field_2 = Animated_Field("field")
        self.field_2_layout = QGridLayout(self.field_2)

        # middle Separation
        self.middle_separation = QWidget()
        self.middle_separation_layout = QVBoxLayout(self.middle_separation)

        # Field 4
        self.field_4 = Animated_Field("field")
        self.field_4_layout = QGridLayout(self.field_4)

        # Field 5
        self.field_5 = Animated_Field("field")
        self.field_5_layout = QGridLayout(self.field_5)

        # Field 3
        self.field_3 = Animated_Field("field")
        self.field_3_layout = QGridLayout(self.field_3)

        # set all names

        self.bottom_separation.setObjectName("container")
        self.middle_separation.setObjectName("container")

        """Add all container"""
        self.main_layout.addWidget(self.field_1, 2)
        self.main_layout.addWidget(self.bottom_separation, 8)

        self.bottom_separation_layout.addWidget(self.field_2, 2)
        self.bottom_separation_layout.addWidget(self.middle_separation, 8)
        self.bottom_separation_layout.addWidget(self.field_3, 2)

        self.middle_separation_layout.addWidget(self.field_5, 9)
        self.middle_separation_layout.addWidget(self.field_4, 1)

        """FIELD 4 GUI"""
        self.setup_field_1()
        self.setup_field_2()
        self.setup_field_3()
        self.setup_field_4()
        self.setup_field_5()

    def setup_field_1(self):
        self.title_label = QLabel("S.I.R.I.U.S. - MAIN TERMINAL")
        self.title_label.setObjectName("title_name")

        # perspective buttons
        self.perspective_buttons = [
            QPushButton("Zentralperspektive"),
            QPushButton("Orthogonale Ansicht"),
            QPushButton("2D-Animiert"),
            QPushButton("2D-Statisch"),
        ]
        self.perspective_button_group = QButtonGroup(self)
        self.perspective_button_group.idClicked.connect(self.handle_perspective_change)

        # add everything
        self.field_1_layout.addWidget(self.title_label, 0, 0, 1, len(self.perspective_buttons))
        for i in range(len(self.perspective_buttons)):
            btn = self.perspective_buttons[i]
            btn.setCheckable(True)
            self.field_1_layout.addWidget(btn, 1, i)
            self.perspective_button_group.addButton(btn, i)

        self.perspective_buttons[2].setChecked(True)

    def setup_field_2(self):
        # title
        self.title_label_field2 = QLabel("Input".upper())
        self.title_label_field2.setObjectName("field_title_label")

        self.field_2_layout.addWidget(self.title_label_field2, 0, 0, 1, 2)

        # controller status and connection
        self.controller_connection_btn = QPushButton("Kontroller verbinden")
        self.controller_connection_btn.setObjectName("controller_btn")
        self.controller_connection_btn.clicked.connect(self.handle_btn_controller_input)

        self.controller_connection_label = [QLabel("Kontroller Status"), QLabel("---")]
        self.controller_connection_label[1].setObjectName("status_field")
        self.controller_connection_label[1].setProperty("status", "-1")

        self.field_2_layout.addWidget(self.controller_connection_label[0], 1, 0, 1, 1)
        self.field_2_layout.addWidget(self.controller_connection_label[1], 1, 1, 1, 1)
        self.field_2_layout.addWidget(self.controller_connection_btn, 2, 0, 1, 2)

        # controller control type
        self.controller_control_type_button = [
            QLabel("Kontroller Steuerung".upper()),
            QButtonGroup(self),
            QPushButton("absolute"),
            QPushButton("relative")
        ]
        self.controller_control_type_button[0].setObjectName("under_field_title")

        def set_controller_type(new_type):
            self.controller_control_type = new_type
        self.controller_control_type_button[1].idClicked.connect(lambda i: set_controller_type(i))


        for i in range(2):
            btn = self.controller_control_type_button[i + 2]
            btn.setCheckable(True)
            self.field_2_layout.addWidget(btn, 4, i, 1, 1)
            self.controller_control_type_button[1].addButton(btn, i)

        self.controller_control_type_button[2].setChecked(True)

        self.field_2_layout.addWidget(self.controller_control_type_button[0], 3, 0, 1, 2)

        # Vector input
        self.vector_input_title = QLabel("Vektor Eingeben".upper())
        self.vector_input_title.setObjectName("under_field_title")
        self.field_2_layout.addWidget(self.vector_input_title, 5, 0, 1, 2)

        self.vector_input_label = [QLabel("X:"), QLabel("Y:"), QLabel("Z:")]
        self.vector_input_fields = [QLineEdit(), QLineEdit(), QLineEdit()]

        float_validator = QDoubleValidator()
        float_validator.setLocale(QLocale(QLocale.English, QLocale.UnitedStates))
        float_validator.setNotation(QDoubleValidator.StandardNotation)

        for i, cord in enumerate(["X", "Y", "Z"]):
            self.vector_input_fields[i].setPlaceholderText(f"[{cord}-Wert eingeben]")
            self.vector_input_fields[i].setValidator(float_validator)

        for x, column in enumerate([self.vector_input_label, self.vector_input_fields]):
            for i in range(len(self.vector_input_label)):
                self.field_2_layout.addWidget(column[i], 7 + i, x, 1, 1)

        self.vector_input_type = [
            QButtonGroup(self),
            QPushButton("real"),
            QPushButton("normal"),
            "real"
        ]

        self.vector_input_type[0].idClicked.connect(lambda i: self.vector_input_click(i, 0))

        for i in range(2):
            btn = self.vector_input_type[i + 1]
            btn.setCheckable(True)
            self.field_2_layout.addWidget(btn, 10, i, 1, 1)
            self.vector_input_type[0].addButton(btn, i)

        self.vector_input_type[1].setChecked(True)

        self.vector_input_btn = QPushButton("Bestätigen")
        self.vector_input_btn.clicked.connect(lambda _: self.vector_input_click(None, 1))
        self.field_2_layout.addWidget(self.vector_input_btn, 11, 0, 1, 2)

        # vector label
        self.vector_label_title = QLabel("aktueller Vektor".upper())
        self.vector_label_title.setObjectName("under_field_title")
        self.vector_label_normalized_title = QLabel("Normalisiert")
        self.vector_label_normalized = QLabel("X:  -.-- \nY:  -.-- \nZ:  -.--")

        self.vector_label_real_title = QLabel("Real")
        self.vector_label_real = QLabel("X:  -.-- \nY:  -.-- \nZ:  -.--")

        self.field_2_layout.addWidget(self.vector_label_title, 12, 0, 1, 2)
        self.field_2_layout.addWidget(self.vector_label_normalized_title, 13, 0, 1, 1)
        self.field_2_layout.addWidget(self.vector_label_normalized, 13, 1, 1, 1)
        self.field_2_layout.addWidget(self.vector_label_real_title, 14, 0, 1, 1)
        self.field_2_layout.addWidget(self.vector_label_real, 14, 1, 1, 1)

        # set last row
        last_row = self.field_2_layout.rowCount()
        self.field_2_layout.setRowStretch(last_row, 1)

        # exit button
        self.exit_button = QPushButton("VERLASSEN")
        self.exit_button.clicked.connect(self.stop_everything)
        self.exit_button.setObjectName("exit_button")
        self.field_2_layout.addWidget(self.exit_button, last_row + 1, 0, 1, 2)

    def setup_field_3(self):
        self.title_label_field3 = QLabel("Output".upper())
        self.title_label_field3.setObjectName("field_title_label")
        self.field_3_layout.addWidget(self.title_label_field3, 0, 0, 1 ,2)

        # all angles
        self.angle_labels_title = QLabel("WINKEL")
        self.angle_labels_title.setObjectName("under_field_title")
        self.field_3_layout.addWidget(self.angle_labels_title, 1, 0, 1 ,2)

        self.angle_labels_name = [QLabel("Basis"), QLabel("a<sub>1</sub>"), QLabel("a<sub>2</sub>"),
                                   QLabel("a<sub>3</sub>")]

        self.angle_labels_values = [QLabel("--.-°"), QLabel("--.-°"), QLabel("--.-°"),
                                   QLabel("--.-°")]

        for i in range(len(self.angle_labels_name)):
            self.field_3_layout.addWidget(self.angle_labels_name[i], 2 + i, 0)
        for i in range(len(self.angle_labels_values)):
            self.field_3_layout.addWidget(self.angle_labels_values[i], 2 + i, 1)


        # status
        self.socket_status_title = QLabel("SOCKET STATUS")
        self.socket_status_title.setObjectName("under_field_title")
        self.field_3_layout.addWidget(self.socket_status_title, 6, 0, 1, 2)

        self.socket_status_names = [QLabel("Client"), QLabel("Server"), QLabel("Ping")]
        self.socket_status_status = [QLabel("---"), QLabel("---"), QLabel("---ms")]

        for i in range(3):
            self.socket_status_status[i].setObjectName("status_field")
            self.socket_status_status[i].setProperty("status", "-1")
            self.field_3_layout.addWidget(self.socket_status_status[i], 7 + i, 1, 1, 1)

        for i in range(3):
            self.field_3_layout.addWidget(self.socket_status_names[i], 7 + i, 0, 1, 1)

        # movement sequence
        self.movement_sequence_title = QLabel("BEWEGUNGSABLAUF")
        self.movement_sequence_title.setObjectName("under_field_title")
        self.field_3_layout.addWidget(self.movement_sequence_title, 14, 0, 1, 2)

        self.movement_sequence_file_name_button = QPushButton("Datei auswählen")
        self.movement_sequence_file_name_button.clicked.connect(self.select_sequence_file)
        self.field_3_layout.addWidget(self.movement_sequence_file_name_button, 15, 0, 1, 2)

        self.movement_sequence_file_name_lable = QLabel("Datei: -NICHT AUSGEWÄHLT-")
        self.field_3_layout.addWidget(self.movement_sequence_file_name_lable, 16, 0, 1, 2)

        self.movement_sequence_save_pos_button = QPushButton("Position schreiben")
        self.movement_sequence_save_pos_button.clicked.connect(self.add_pos_to_sequence)
        self.field_3_layout.addWidget(self.movement_sequence_save_pos_button, 17, 0, 1, 2)

        self.movement_sequence_start_button = QPushButton("Ablauf starten")
        self.movement_sequence_start_button.clicked.connect(self.send_sequence)
        self.field_3_layout.addWidget(self.movement_sequence_start_button, 18, 0, 1, 2)

        # set last row
        last_row = self.field_3_layout.rowCount()
        self.field_3_layout.setRowStretch(last_row, 1)

    def setup_field_4(self):
        self.logging_label = QLabel("Lade...")
        self.logging_label.setObjectName("logging_label")
        self.field_4_layout.addWidget(self.logging_label, 0, 0)

    def setup_field_5(self):
        # Tabs
        self.field5_stacked_widget = QStackedWidget()

        self.perspective_tab = QWidget()
        self.perspective_layout = QGridLayout(self.perspective_tab)
        self.orthogonal_tab = QWidget()
        self.orthogonal_layout = QGridLayout(self.orthogonal_tab)
        self.animated_tab = QWidget()
        self.animated_layout = QGridLayout(self.animated_tab)
        self.static_tab = QWidget()
        self.static_layout = QGridLayout(self.static_tab)

        # setup all tabs
        self.setup_perspective_perspective()
        self.setup_orthogonal_perspective()
        self.setup_animated_perspective()
        self.setup_static_perspective()

        # Add all widgets
        self.field5_stacked_widget.addWidget(self.perspective_tab)
        self.field5_stacked_widget.addWidget(self.orthogonal_tab)
        self.field5_stacked_widget.addWidget(self.animated_tab)
        self.field5_stacked_widget.addWidget(self.static_tab)

        self.field5_stacked_widget.setCurrentIndex(2)

        self.field_5_layout.addWidget(self.field5_stacked_widget, 0, 0)

    def load_all_fonts(self):
        for f in ["Sixtyfour-Regular", "Jersey10-Regular", "Tektur-Regular"]:
            font_path = os.path.join(self.base_path, "assets", "fonts", f"{f}.ttf")
            font_id = QFontDatabase.addApplicationFont(font_path)

            if font_id != -1:# 3. Den echten internen Namen der Schriftart auslesen
                font_families = QFontDatabase.applicationFontFamilies(font_id)
                my_custom_font_name = font_families[0]
                self.log(f"Erfolgreich geladen: {my_custom_font_name}")
            else:
                self.log(f"Fehler beim Laden der Schriftart von: {font_path}")

    # <editor-fold desc="VECTOR AND ANGLES">
    """VECTOR AND ANGLES"""

    def update_gui_vector(self):
        # normalized
        self.vector_label_normalized.setText(f"X:  {self.current_vector[0]:.2f}\n"
                                             f"Y:  {self.current_vector[1]:.2f}\n"
                                             f"Z:  {self.current_vector[2]:.2f}")

        self.robot_arm_canvas.vector = self.current_vector.copy()

        # real
        x_real = self.kinematics.normalized_to_real(self.current_vector[0])
        y_real = self.kinematics.normalized_to_real(self.current_vector[1])
        z_real = self.kinematics.normalized_to_real(self.current_vector[2])

        self.vector_label_real.setText(f"X:  {x_real:.2f}cm\n"
                                       f"Y:  {y_real:.2f}cm\n"
                                       f"Z:  {z_real:.2f}cm")

    def update_all_vectors(self, x, y, z, move_type):
        self.calculate_angle_by_new_vector(new_vec=[x, y, z], move_type=move_type)
        self.current_vector = [x, y, z]

    def update_all_angles(self, new_angles,move_type):
        self.current_angles: list = new_angles # [base_rotation, joint_rotation]

        self.socket_thread.feed_data(str(["angles", move_type, new_angles, "confirm"]))

        # update GUI
        for i in range(len(self.angle_labels_values) - 1):
            self.angle_labels_values[i + 1].setText(f"{new_angles[1][i]:.1f}°")

        self.angle_labels_values[0].setText(f"{new_angles[0]:.1f}°")

        # update canvas
        self.robot_arm_canvas.angles = self.current_angles

    def set_to_pos(self, pos: int):
        if pos == 0:
            self.new_vector.emit(0.5, 0.62, 0, "move")

    def calculate_angle_by_new_vector(self, new_vec: list,  move_type):
        if self.worker_is_busy:
            return

        self.worker_is_busy = True
        self.calculated_vector = new_vec

        worker = CalculateNewAngles(new_vec, self.kinematics, move_type)

        worker.signals.finished.connect(self.handle_angle_results)
        worker.signals.error.connect(lambda err: self.handle_angle_results(f"Fehler beim Berechnen der Winkel: {err}", "", None, True))

        self.threadpool_kinematics.start(worker)

    def handle_angle_results(self, res, move_type: str = "move", n_vec = None, is_error: bool = False):


        if res is False: # check if result is False
            self.log(f"Vektor ( {self.current_vector[0]:.1f} | {self.current_vector[1]:.1f} | {self.current_vector[2]:.1f} ) ist nicht erreichbar. "
                     f"Setze zu ( {self.old_vector[0]:.1f} | {self.old_vector[1]:.1f} | {self.old_vector[2]:.1f} )", 1)

            self.current_vector = self.old_vector
            self.calculated_vector = self.current_vector.copy()

        # check if error
        elif is_error:
            self.log(res, 2)
            self.current_vector = self.old_vector

        # vector is valid
        else:
            self.log("Vektor erreichbar")
            self.new_angles.emit(res, move_type)
            self.current_vector = n_vec
            self.old_vector = n_vec


        self.update_gui_vector()

        # open worker for next one
        self.worker_is_busy = False

        # check if calculated vektor is not current vector
        if self.calculated_vector != self.current_vector.copy():
            self.new_vector.emit(self.current_vector[0], self.current_vector[1], self.current_vector[2], move_type)
    # </editor-fold>

    """SEQUENCE"""

    def select_sequence_file(self):
        path, _ = QFileDialog.getSaveFileName(self, "Ablauf-Datei auswählen oder erstellen", "",
                                              "JSON Files (*.json)",options=QFileDialog.DontConfirmOverwrite)

        if not path: # not selected
            return

        if not os.path.exists(path):
            self.current_sequence = [ ["gripper", [0, 0]], ["angles", "move", [0, [0, 0, 0]]] ]
            with open(path, "w") as f:
                json.dump(self.current_sequence, f, indent=4)
        else:
            # check if data is valid
            data_read = []
            with open(path, "r") as f:
                data_read = json.load(f)

            try:
                for data in data_read:
                    if data[0] == "gripper":
                        for a in range(2):
                            if type(data[1][a]) != float and type(data[1][a]) != int:
                                raise TypeError

                    elif data[0] == "angles":
                        if not type(data[1]) == str:
                            raise TypeError
                        elif type(data[2][0]) != float and type(data[2][0]) != int :
                                raise TypeError
                        else:
                            for a in range(3):
                                if type(data[2][1][a]) != float and type(data[2][1][a]) != int:
                                    raise TypeError

            except Exception as e:
                self.log(f"Die gewählte Datei enthält einen Fehler!", 2)
                QMessageBox.critical(self, "Datei ungültig", f"Die ausgewählt Datei ist ungültig. Bitte überprüfe diese oder wähle eine andere.\nDatei: {path}")
                self.current_sequence = []
                self.selected_sequence_file = ""
                self.movement_sequence_file_name_lable.setText(f"Datei: -NICHT AUSGEWÄHLT-")
                return
            else:
                self.current_sequence = data_read


        self.log(f"Datei erfolgreich geladen")
        self.selected_sequence_file = path

        # update GUI
        self.movement_sequence_file_name_lable.setText(f"Datei: {path.split('/')[-1]}")

    def add_pos_to_sequence(self):
        try:
            if self.selected_sequence_file == "":
                self.log("Es ist keine Datei ausgewählt.", 1)
                return

            self.current_sequence.append(["angles", "move", self.current_angles])
            self.current_sequence.append(["gripper", self.gripper_pos])

            with open(self.selected_sequence_file, "w") as f:
                json.dump(self.current_sequence, f, indent=4)

            self.log("Position gespeichert")
        except:
            self.log("Es ist ein Fehler aufgetreten. Position konnte nicht gespeichert werden.", 2)

    def send_sequence(self):
        if self.selected_sequence_file == "":
            self.log("Es ist keine Datei ausgewählt.", 1)
            return

        self.log("Sende Bewegungsablauf...", 0)

        total_elements = len(self.current_sequence)

        progress_window = QProgressDialog("Bitte warten...", "Abbrechen", 0, total_elements, self)
        progress_window.setWindowTitle("S.I.R.I.U.S. - Movement-Sequence")
        progress_window.setWindowModality(Qt.WindowModal)
        progress_window.setAutoClose(True)
        progress_window.setMinimumDuration(0)
        progress_window.setValue(0)
        progress_window.show()

        time.sleep(.500)
        for index, data in enumerate(self.current_sequence):
            data.append("confirm")
            self.socket_thread.save_send_data(str(data))

            while self.socket_thread.current_data != self.socket_thread.data_send:
                time.sleep(0.01)

                if progress_window.wasCanceled():
                    self.log("Ladevorgang abgebrochen.", 1)
                    return

            progress_window.setValue(index)
            progress_window.setLabelText(f"Daten werden gesendet... ({index+1}/{total_elements})")
            self.log(f"Daten werden gesendet... ({index+1}/{total_elements})")

            if progress_window.wasCanceled():
                self.log("Ladevorgang abgebrochen.", 1)
                return

        progress_window.setValue(total_elements + 1)
        progress_window.destroy()
        self.log(f"Bewegungsablauf erfolgreich gesendet")

    """PERSPEKTIVE"""
    
    def handle_perspective_change(self, which: int):
        new_perspective = self.all_perspectives[which]
        self.perspective_selected = new_perspective
        self.log(f"Verändere perspektive zu '{self.perspective_selected}'")

        self.field5_stacked_widget.setCurrentIndex(which)

    def setup_perspective_perspective(self):
        pass

    def setup_orthogonal_perspective(self):
        pass

    def setup_animated_perspective(self):
        self.robot_arm_canvas = RobotArmCanvas()

        self.animated_layout.addWidget(self.robot_arm_canvas, 0, 0)

    def setup_static_perspective(self):
        self.static_svg_widget = RobotArmCanvas(is_static=True)

        self.static_layout.addWidget(self.static_svg_widget, 0, 0, 1, 1)

    # <editor-fold desc="X-BOX CONTROLLER">
    """X-BOX CONTROLLER"""

    def loop_button_pressed(self):
        for code in self.current_button_pressed:
            if code in ["ABS_Z", "ABS_RZ", "BTN_TL", "BTN_TR"]:
                self.controller_input_gripper(code)

    def handle_controller_input(self, ev_type, code, state):
        if self.controller_is_connected != 1:
            return
        # movement over joysticks
        if code in ["ABS_Y", "ABS_X", "ABS_RY", "ABS_RX"]:
            match code:
                case "ABS_Y":
                    self.last_controller_joystick_input[0] = state
                case "ABS_X":
                    self.last_controller_joystick_input[2] = state
                case "ABS_RY":
                    self.last_controller_joystick_input[1] = state
                case "ABS_RX":
                    self.last_controller_joystick_input[3] = state
        elif code in ["ABS_Z", "ABS_RZ", "BTN_TL", "BTN_TR"]:
            if state > 0 and code in ["BTN_TL", "BTN_TR"]:
                self.current_button_pressed.add(code)
            elif state > 100 and code in ["ABS_Z", "ABS_RZ"]:
                self.current_button_pressed.add(code)
            else:
                if code in self.current_button_pressed:
                    self.current_button_pressed.remove(code)
        else:
            self.log(f"{ev_type, code, state}")

    def controller_input_gripper(self, button_pressed):
        if button_pressed == "ABS_Z":
            self.gripper_pos[0] -= 0.01
        elif button_pressed == "ABS_RZ":
            self.gripper_pos[0] += 0.01
        elif button_pressed == "BTN_TL":
            self.gripper_pos[1] -= 0.3
        elif button_pressed == "BTN_TR":
            self.gripper_pos[1] += 0.3

        self.gripper_pos = [
            min(max(self.gripper_pos[0], 0), 1),
            min(max(self.gripper_pos[1], -135), 135),
        ]

        self.socket_thread.feed_data(str(["gripper", self.gripper_pos, "confirm"]))

        self.log(f"Greifer verändert: geschlossen: {self.gripper_pos[0]:.1f}, rotiert: {self.gripper_pos[1]:.1f}")

    def new_vector_by_joystick(self):
        if self.controller_control_type == 0:
            self.calc_new_vector_by_joystick_absolute()
        else:
            self.calc_new_vector_by_joystick_relative()

    def calc_new_vector_by_joystick_absolute(self):
        if self.controller_is_connected != 1:
            self.controller_delta_time = time.time()
            return

        # check for first run
        if self.controller_delta_time == -1:
            self.controller_delta_time = time.time()
            return

        # check if input is simply none
        if self.last_controller_joystick_input == [0, 0, 0, 0]:
            self.controller_delta_time = time.time()
            return

        # var
        n_vec = self.current_vector.copy()
        d_time = time.time() - self.controller_delta_time

        # iterate through every joystick input (Only 3 Axis)
        for i in range(3):
            # get current value
            c_value = self.last_controller_joystick_input[i]

            # return if c_value is 0
            if c_value == 0:
                continue

            # calculate add cord
            # **2 to make it increasing by higher value.
            if i != 1:
                direction = -1 if c_value > 0 else 1
            else:
                direction = -1 if c_value < 0 else 1

            add_cord = (c_value / 100)**2 * d_time * self.controller_speed * direction


            n_vec[i] += add_cord
            n_vec[i] = max(-1, min(1, n_vec[i]))


        self.new_vector.emit(n_vec[0], n_vec[1], n_vec[2], "set")

        self.controller_delta_time = time.time()

    def calc_new_vector_by_joystick_relative(self):
        if self.controller_is_connected != 1:
            self.controller_delta_time = time.time()
            return

        # check for first run
        if self.controller_delta_time == -1:
            self.controller_delta_time = time.time()
            return

        # check if input is simply none
        if self.last_controller_joystick_input == [0, 0, 0, 0]:
            self.controller_delta_time = time.time()
            return

        # var
        n_vec = self.current_vector.copy()
        d_time = time.time() - self.controller_delta_time

        # axis 2 (index = 1) is Y-Axis
        c_value = self.last_controller_joystick_input[1]

        # return if c_value is 0
        if c_value != 0:
            direction = -1 if c_value < 0 else 1

            add_cord = (c_value / 100) ** 2 * d_time * self.controller_speed * direction

            n_vec[1] += add_cord
            n_vec[1] = max(-1, min(1, n_vec[1]))

        # Calc new x and y so that basis angle stays the exact same and only the radius/distance changes.
        c_value = self.last_controller_joystick_input[0]
        if c_value != 0:
            direction = -1 if c_value < 0 else 1
            add_cord = (c_value / 100) ** 2 * d_time * self.controller_speed * direction

            d_old = math.sqrt(n_vec[0] ** 2 + n_vec[2] ** 2)
            d_new = d_old + add_cord

            scale_f = d_new / d_old

            n_vec[0] *= scale_f
            n_vec[2] *= scale_f

        # axis 3 (index = 2) rotates only along the base
        c_value = self.last_controller_joystick_input[2]
        if c_value != 0:
            direction = -1 if c_value < 0 else 1
            add_degree = (c_value / 50) ** 2 * d_time * self.controller_speed * direction

            current_radius = math.sqrt(n_vec[0] ** 2 + n_vec[2] ** 2)

            base_rotation_degree = math.atan2(n_vec[2], n_vec[0])

            new_degree = base_rotation_degree + add_degree
            print(base_rotation_degree, new_degree, add_degree)

            n_vec[0] = current_radius * math.cos(new_degree)
            n_vec[2] = current_radius * math.sin(new_degree)

        self.new_vector.emit(n_vec[0], n_vec[1], n_vec[2], "set")

        self.controller_delta_time = time.time()

    def change_controller_active(self, active: int, message:str, m_type: str):
        self.controller_is_connected = active
        if active == -1:
            self.controller_connection_label[1].setProperty("status", "0")
            self.controller_connection_label[1].setText("Deaktiviert")
            self.controller_connection_btn.setText("Aktivieren")

        elif active == 0:
            self.controller_connection_label[1].setProperty("status", "0")
            self.controller_connection_label[1].setText("Nicht Verbunden")
            self.controller_connection_btn.setText("Verbinden")

        elif active == 1:
            self.controller_connection_label[1].setProperty("status", "2")
            self.controller_connection_label[1].setText("Verbunden")
            self.controller_connection_btn.setText("Deaktivieren")

        self.controller_connection_label[1].style().unpolish(self.controller_connection_label[1])
        self.controller_connection_label[1].style().polish(self.controller_connection_label[1])

        if message != "":
            self.log(message, type=m_type)

    def handle_btn_controller_input(self):
        if self.controller_is_connected == -1:
            self.change_controller_active(1, "Kontroller aktiviert", 0)
        elif self.controller_is_connected == 0:
            self.log("Verbindung mit Kontroller wird hergestellt")
            self.controller_thread.terminate()
            self.controller_thread.wait()
            self.controller_thread.start()
        elif self.controller_is_connected == 1:
            self.change_controller_active(-1, "Kontroller deaktiviert", 0)

    def stop_everything(self):
        self.socket_thread.stop()
        self.controller_thread.terminate()
        self.close()
    # </editor-fold>

    """SOCKET STATUS"""

    def update_socket_status(self, client, server):
        s = [client, server]
        for i in range(2):
            if s[i] == -1:
                self.socket_status_status[i].setProperty("status", "-1")
                self.socket_status_status[i].setText("unbekannt")
            elif s[i]:
                self.socket_status_status[i].setProperty("status", "2")
                self.socket_status_status[i].setText("online")
            else:
                self.socket_status_status[i].setProperty("status", "0")
                self.socket_status_status[i].setText("offline")

            self.socket_status_status[i].style().unpolish(self.socket_status_status[i])
            self.socket_status_status[i].style().polish(self.socket_status_status[i])

    def update_ping_label(self, ping):
        self.socket_status_status[2].setText(f"{int(ping)} ms")

        if ping > 100:
            self.socket_status_status[2].setProperty("status", "0")
        elif ping > 50:
            self.socket_status_status[2].setProperty("status", "1")
        else:
            self.socket_status_status[2].setProperty("status", "2")

        self.socket_status_status[2].style().unpolish(self.socket_status_status[2])
        self.socket_status_status[2].style().polish(self.socket_status_status[2])

    """USER INPUT"""

    def vector_input_click(self, data, type: int):
        if type == 0: # change if real or normal
            self.vector_input_type[3] = "real" if data == 0 else "normal"
        else: # vector accepted
            data = [
                float(self.vector_input_fields[0].text()),
                float(self.vector_input_fields[1].text()),
                float(self.vector_input_fields[2].text()),
            ]
            if self.vector_input_type[3] == "real":
                data = [
                    data[0] /  45,
                    data[1] /  45,
                    data[2] /  45,
                ]

            self.new_vector.emit(data[0], data[1], data[2], "move")

if __name__ == "__main__":
    app = QApplication(sys.argv)

    apply_stylesheet(app, "dark_yellow.xml")

    base_style = app.styleSheet()
    custom_style = load_stylesheet("style.qss")

    app.setStyleSheet(base_style + "\n/* --- CUSTOM SIRIUS STYLE --- */\n" + custom_style)

    window = MainWindow()
    window.showFullScreen()

    sys.exit(app.exec())
