import serial
import time

class Switchboard():

    def __init__(self, comPort):
        self.ser = serial.Serial(comPort, baudrate = 115200, timeout = 0.1, write_timeout = 0)
        self.send_order(b'SVset0\r')
        bytes = "SHB 1\r".encode()
        self.send_order(bytes)
        bytes = "SOC 1\r".encode()
        self.send_order(bytes)

    def reset_coeff(self):
        bytes = "SC0 0\r".encode()
        self.send_order(bytes)
        bytes = "SC1 1\r".encode()
        self.send_order(bytes)
        bytes = "SC2 0\r".encode()
        self.send_order(bytes)

    def set_coeff(self, C0, C1, C2):
        bytes = "SC0 {}\r".format(C0).encode()
        print(bytes)
        self.send_order(bytes)
        bytes = "SC1 {}\r".format(C1).encode()
        print(bytes)
        self.send_order(bytes)
        bytes = "SC2 {}\r".format(C2*1000000).encode()
        print(bytes)

        self.send_order(bytes)
        #
        bytes = "Save\r".encode()
        self.send_order(bytes)


    def set_voltage(self, V):
        bytes = "SVset {}\r".format(int(V)).encode()
        self.send_order(bytes)
    def get_voltage(self):
        bytes = "QVnow\r".encode()
        ret = int(self.query_value(bytes).decode('utf-8'))
        # print(ret)
        return ret

    def send_order(self, bytes):
        # print("Sent: {}".format(bytes))
        self.ser.write(bytes)
        # print("Read: {}\n".format(self.ser.readline()))

    def query_value(self, bytes):
        self.ser.reset_input_buffer()
        # print("Sent: {}".format(bytes))
        self.ser.write(bytes)
        ret = self.ser.readline()
        # print("Read: {}\n".format(ret))
        return(ret)


    def close(self):
        self.ser.close()