import serial
import numpy as np
import matplotlib.pyplot as plt
from Switchboard import Switchboard
from daqmx import *
import time


def record_voltage_sweep(target_v, settling_delay, measurement_duration, sampling_frequency, channels):
    sample_number = measurement_duration * sampling_frequency  # Nombre d'echantillons par voie
    # apply voltage setpoint and measure output
    n_channels = len(channels)
    mean_data = np.array([]).reshape((-1, n_channels))
    channels_str = ['/Dev1/Ai{}'.format(c) for c in channels]

    ts = []
    vs = []
    t_start = time.perf_counter()

    for v in target_v:
        sw.set_voltage(v)
        t_last_step = time.perf_counter()
        dt = 0

        # time.sleep(settling_delay)

        while dt < settling_delay:
            vs.append(sw.get_current_voltage())
            tnow = time.perf_counter()
            ts.append(tnow - t_start)
            dt = tnow - t_last_step

        tdata, data = mesureAnalogique.set_analog_measurement(channels_str,
                                                              sampling_frequency,
                                                              typeEch='fini',
                                                              sample_number=sample_number)
        data = data.reshape((n_channels, -1))  # make sure it's a 2-D array, even if using only one channel
        mean_sample = np.mean(data, axis=1, keepdims=True).transpose()
        mean_data = np.append(mean_data, mean_sample, axis=0)

        msg = "Target voltage: {:4d}\t measured by switchboard: {:4d}\t measured by probe: {}"
        print(msg.format(v, sw.get_current_voltage(), mean_sample))

    # plt.subplots()
    # plt.plot(ts, vs)
    # plt.xlabel("Time [s]")
    # plt.ylabel("Voltage [V]")
    # plt.draw()
    # plt.pause(0.1)

    return mean_data


if __name__ == '__main__':
    # load probe parameters
    probe_fit = np.load("probe_fit_inverse_2.npy")
    p_probe = np.poly1d(probe_fit)

    # user defined settings
    minvoltage = 300
    maxvoltage = 4800
    stepvoltage = 50
    delay_sw_s = 1.5
    t = 1  # Durée d'acquisition (s)

    # init switchboard: connect, 0V, set coefficients for y=x (0,1,0)
    sw = Switchboard()
    sw.open()
    # set min and max voltage so that we can perform the calibration and the calibrated range will be the allowed range
    sw.set_maximum_voltage(maxvoltage)
    sw.set_minimum_voltage(minvoltage)

    sw.set_voltage(0)
    sw.set_output_on()
    sw.reset_calibration_coefficients()
    time.sleep(1)

    # init DaqMx
    mesureAnalogique = DaqMx()
    sampling_freq = 1000  # Fréquence d'echantillonnage (Hz)

    target_voltages = np.array(range(minvoltage, maxvoltage+1, stepvoltage))

    measured_V = record_voltage_sweep(target_voltages, delay_sw_s, t, sampling_freq, channels=[0])

    sw.set_voltage(0)
    time.sleep(1)

    voltage_HV = p_probe(measured_V[:, 0])  # voltage measured by the HV probe
    # voltage_input = measured_V[:, 1]  # input voltage to the DCDC

    fit = np.polyfit(target_voltages, voltage_HV, 2)
    p_sw = np.poly1d(fit)
    # xp = np.linspace(minvoltage / 1000, maxvoltage / 1000, 20)

    # compute coefficients for best fit and apply to switchboards
    print("Probe curve equation:\n{}".format(p_sw))
    print("\n\ncoefficients: {}".format(fit))
    np.save("switchboard_fit_2", fit)

    sw.set_calibration_coefficients(fit[2], fit[1], fit[0])

    # second pass: check if the result is OK

    measured_V_after = record_voltage_sweep(target_voltages, delay_sw_s, t, sampling_freq, channels=[0])
    voltage_HV_after = p_probe(measured_V_after[:, 0])  # voltage measured by the HV probe
    # voltage_input_after = measured_V_after[:, 1]  # input voltage to the DCDC
    voltage_fit = p_sw(target_voltages)

    sw.set_voltage(0)
    sw.set_output_off()

    # compute error before and after
    error_before_HV = voltage_HV - target_voltages
    error_after_HV = voltage_HV_after - target_voltages
    error_fit = voltage_fit - target_voltages

    fig1, ax1 = plt.subplots()
    lines1 = ax1.plot(target_voltages, voltage_HV, 'b+-',
                      target_voltages, voltage_HV_after, 'r+-')
    ax1.set_xlabel("Target voltage [V]")
    ax1.set_ylabel("Output voltage [V]")
    ax1.legend(["Before", "After"])
    # ax2 = ax1.twinx()
    # lines2 = ax2.plot(target_voltages, voltage_input, 'g+-',
    #                   target_voltages, voltage_input_after, 'y+-')
    # ax2.set_ylabel("Input voltage [V]")
    # ax1.legend(lines1+lines2, ["HV before", "HV after", "Input before", "Input after"])

    fig2, ax3 = plt.subplots()
    ax3.plot(target_voltages, error_before_HV, 'b+-',
             target_voltages, error_after_HV, 'r+-',
             target_voltages, error_fit, 'k-')
    ax3.set_xlabel("Target voltage [V]")
    ax3.set_ylabel("Error voltage [V]")
    ax3.legend(["Error before", "Error after", "Fit"])
    ax3.set_title("C0: {:.3f}, C1: {:.3f}, C2: {:.3f}".format(fit[2], fit[1], fit[0]*1000000))

    # fig3, ax = plt.subplots()
    # ax.plot(target_voltages, voltage_HV / voltage_input, 'b+-',
    #         target_voltages, voltage_HV_after / voltage_input_after, 'r+-')
    # ax.set_xlabel("Target voltage [V]")
    # ax.set_ylabel("DCDC gain")
    # ax.legend(["Gain before", "Gain after"])

    sw.close()

    plt.show()
