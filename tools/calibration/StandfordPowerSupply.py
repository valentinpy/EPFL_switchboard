import pyvisa

class StandfordPowerSupply():

    def __init__(self, resource=None):
        self.rm = pyvisa.ResourceManager()
        print(self.rm.list_resources())
        self.instrument = self.rm.open_resource(resource)
        print(self.instrument.query("*IDN?"))

    def set_voltage(self, V):
        self.instrument.write("VSET%f;" % V)

    def set_voltage_lim(self, V):
        self.instrument.write("VLIM%f;" % V)

    def get_voltage(self):
        return self.instrument.query("VOUT?")

    def output_on(self):
        self.instrument.write("HVON;")

    def output_off(self):
        self.instrument.write("HVOF;")
