import pyvisa
import time

from StandfordPowerSupply import *
from daqmx import *
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    source = StandfordPowerSupply(resource="GPIB0::11::INSTR")
    minvoltage = 100
    maxvoltage = 5000
    step = 50
    waiting_time_source_s = 5

    mesureAnalogique = DaqMx()
    sampling_frequency = 1000  # Fréquence d'echantillonnage (Hz)
    t = 1  # Durée d'acquisition (s)
    sample_number = t * sampling_frequency  # Nombre d'echantillons par voie

    source.set_voltage_lim(5000)
    source.output_on()

    read_voltage_source = []
    mean_voltage_probe = []
    std_voltage_probe = []

    for tgt_voltage in range(minvoltage, maxvoltage, step):
        source.set_voltage(tgt_voltage)
        time.sleep(waiting_time_source_s)
        read_voltage_source.append(float(source.get_voltage()))

        tdata, data = mesureAnalogique.set_analog_measurement(['/Dev1/Ai0'], sampling_frequency, typeEch='fini', sample_number=sample_number)
        mean_voltage_probe.append(np.mean(data))
        std_voltage_probe.append(np.std(data))
        print("Source voltage: {}\t Probe: {}V, std:{}".format(read_voltage_source[-1], mean_voltage_probe[-1], std_voltage_probe[-1]))

    source.set_voltage(0)
    source.output_off()

    # fit = np.polyfit(read_voltage_source, mean_voltage_probe, 2)
    fit_inverse = np.polyfit(mean_voltage_probe, read_voltage_source, 2)
    # print(fit)
    # p = np.poly1d(fit)
    pinv = np.poly1d(fit_inverse)
    xp = np.linspace(minvoltage/1000, maxvoltage/1000, 20)


    plt.figure()
    plt.plot(read_voltage_source, mean_voltage_probe, 'b+-', pinv(xp), xp, 'r+-')
    # plt.xlim((0, 1.1*maxvoltage))
    # plt.ylim((0, 1.1*maxvoltage/1000))
    plt.xlabel("Target voltage [V]")
    plt.ylabel("Probe voltage [V]")
    plt.legend(["measured", "fitted (inverted): \n{}".format(pinv)])
    plt.show()

    print("Probe curve equation (inverted):\n{}".format(pinv))
    print("\n\ncoefficients (inverted): {}".format(fit_inverse))
    # np.save("probe_fit_2", fit)
    np.save("probe_fit_inverse_2", fit_inverse)