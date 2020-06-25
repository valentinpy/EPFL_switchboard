import serial
import numpy as np
import matplotlib.pyplot as plt
from switchboard import *
from daqmx import *
import time

if __name__ == '__main__':
       # load probe parameters
       probe_fit = np.load("probe_fit_inverse_2.npy")
       p_probe = np.poly1d(probe_fit)

       # user defined settings
       com_sw = "COM5" # to be specified
       minvoltage = 500
       maxvoltage = 4900
       stepvoltage = 50
       delay_sw_s = 5

       # init switchboard: connect, 0V, set coefficients for y=x (0,1,0)
       sw = Switchboard(com_sw)
       sw.set_voltage(0)
       sw.reset_coeff()
       time.sleep(1)

       # init DaqMx
       mesureAnalogique = DaqMx()
       sampling_frequency = 1000  # Fréquence d'echantillonnage (Hz)
       t = 1  # Durée d'acquisition (s)
       sample_number = t * sampling_frequency  # Nombre d'echantillons par voie

       # first pass: apply voltage setpoint and measure output
       target_voltage = []
       mean_voltage_probe = []
       std_voltage_probe = []
       for v in range(minvoltage, maxvoltage, stepvoltage):
              sw.set_voltage(v)
              time.sleep(delay_sw_s)
              # print(sw.get_voltage())

              tdata, data = mesureAnalogique.set_analog_measurement(['/Dev1/Ai0'], sampling_frequency, typeEch='fini',
                                                                    sample_number=sample_number)
              target_voltage.append(v)
              probeV = p_probe(np.mean(data))
              mean_voltage_probe.append(probeV)

              # std_voltage_probe.append(np.std(data))
              print("Target voltage: {}\t voltage measured by probe: {}".format(v, probeV))

       fit = np.polyfit(target_voltage, mean_voltage_probe, 2)
       p_sw = np.poly1d(fit)
       xp = np.linspace(minvoltage / 1000, maxvoltage / 1000, 20)

       # compute coefficients for best fit and apply to switchboards
       print("Probe curve equation:\n{}".format(p_sw))
       print("\n\ncoefficients: {}".format(fit))
       np.save("switchboard_fit_2", fit)

       sw.set_voltage(0)
       time.sleep(1)
       sw.set_coeff(fit[2], fit[1], fit[0])

        #second pass: check if the result is OK
       mean_voltage_probe_after = []
       for v in range(minvoltage, maxvoltage, stepvoltage):
              sw.set_voltage(v)
              time.sleep(delay_sw_s)
              # print(sw.get_voltage())

              tdata, data = mesureAnalogique.set_analog_measurement(['/Dev1/Ai0'], sampling_frequency, typeEch='fini',
                                                                    sample_number=sample_number)
              probeV = p_probe(np.mean(data))
              mean_voltage_probe_after.append(probeV)
              print("Target voltage: {}\t voltage measured by probe after calibration: {}".format(v, probeV))

       # compute error before and after
       error_before = [a - b for a, b in zip(mean_voltage_probe, target_voltage)]
       error_after = [a - b for a, b in zip(mean_voltage_probe_after, target_voltage)]


       plt.figure()
       plt.plot(target_voltage, error_before, 'b+-', target_voltage, error_after, 'r+-')
       plt.legend(["Error before", "Error after"])
       plt.xlabel("Target voltage [V]")
       plt.ylabel("Error voltage [V]")
       plt.show()

       sw.close()
