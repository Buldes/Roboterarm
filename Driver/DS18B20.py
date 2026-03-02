import machine
import onewire
import ds18x20

class DS18B20:
    def __init__(self, one_wire_pin_num: int = 26):

        self.one_wire_pin_num: int = one_wire_pin_num
        self.one_wire_pin = None
        self.ds_sensor = None

        self.all_sensors: list = []

    def init(self):
        self.one_wire_pin = machine.Pin(self.one_wire_pin_num)
        self.ds_sensor = ds18x20.DS18X20(onewire.OneWire(self.one_wire_pin))

        self.all_sensors = self.ds_sensor.scan()
        print(f"TOTAL SENSOR FOUND: {len(self.all_sensors)}")

    def measure_temp(self):
        self.ds_sensor.convert_temp()

    def get_temp(self, index: int = -1): # negativ index = all

        if index < 0:
            temps = []

            for sensor in self.all_sensors:
                sensor_id = "".join(['%02x' % b for b in sensor])

                temp = self.ds_sensor.read_temp(sensor)
                temps.append([temp, sensor])

            return temps

        sensor_id = "".join(['%02x' % b for b in self.all_sensors[0]])

        temp = self.ds_sensor.read_temp(sensor_id)

        return [temp, sensor_id]
