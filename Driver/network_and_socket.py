import time

import json
import usocket
import network
import ujson

class network_and_socket:
    def __init__(self, logging_func, add_data_queue):
        # socket var
        self.port = 4242
        self.socket = None
        self.all_ip_addr: list = []
        self.is_online = False

        self.add_data_queue = add_data_queue

        # network var
        self.ssid = "RPI5"
        self.password = "Raspberry"
        self.wlan = None
        self.is_connected = False
        self.ip_address = None

        # log
        self.log = logging_func


    def connect_wifi(self):
        self.log(f"Init WLAN: ssid: {self.ssid}, password: {self.password}")

        # init wlan
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)
        self.wlan.config(pm=0) # deactivates power safe mode

        if not self.wlan.isconnected():

            self.wlan.connect(self.ssid, self.password)

            while not self.wlan.isconnected():
                self.log("Connecting to WLAN...")
                time.sleep(1)

        self.ip_address = self.wlan.ifconfig()[0]
        self.log(f"Connected to WLAN. IP: {self.ip_address}")
        self.is_connected = True

    def activate_socket(self):
        self.log("Starting socket...")
        self.socket = usocket.socket(usocket.AF_INET, usocket.SOCK_DGRAM)

        self.socket.bind(("0.0.0.0", self.port))
        self.socket.settimeout(0.05)

        self.is_online = True

    def recv_data(self):
        try:
            data, addr = self.socket.recvfrom(1024)
        except OSError:
            return False
        # decode message
        message = data.decode('utf-8')

        # check if new ip
        if not addr in self.all_ip_addr:
            self.all_ip_addr.append(addr)

        return message

    def send_message(self, message):
        for addr in self.all_ip_addr:
            self.socket.sendto(bytes(message, "utf-8"), addr)

    def thread_loop(self):
        self.connect_wifi()
        self.activate_socket()

        self.log("UDP server online")

        while True:
            m = self.recv_data()

            if not m:
                continue

            if m == "ping":
                self.send_message("ping")
            else:
                m = m.replace("'", '"')
                data = json.loads(m)
                self.add_data_queue(data)