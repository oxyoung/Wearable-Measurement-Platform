
lpf_numerator_coef = [[0.3913, 0.7827, 0.3913],
                     [0.1311, 0.2622, 0.1311]];
lpf_denominator_coef = [[1.0000, 0.3695, 0.1958],
                     [1.0000, -0.7478, 0.2722]];
hpf_numerator_coef = [[0.8371, -1.6742, 0.8371],
                     [0.9150, -1.8299, 0.9150]];
hpf_denominator_coef = [[1.0000, -1.6475, 0.7009],
                     [1.0000, -1.8227, 0.8372]];
ahf_numerator_coef_50Hz = [[0.9522, -1.5407, 0.9522, 0.8158, -0.8045, 0.0855],
                     [0.5869, -1.1146, 0.5869, 1.0499, -2.0000, 1.0499]];
ahf_denominator_coef_50Hz = [[1.0000, -1.5395, 0.9056, 1.0000, -1.1187, 0.3129],
                     [1.0000, -1.8844, 0.9893, 1.0000, -1.8991, 0.9892]];
ahf_output_gain_coef_50Hz = [1.3422, 1.4399]
# 20HZ - 150HZ
class Filter2nd(object):

    def __init__(self, filter_type, sampleFreq):
        self.states = [0, 0]
        self.num = [0, 0, 0]
        self.den = [0, 0, 0]
        if filter_type == 'lowpass':
            if sampleFreq == 500:
                for i in range (3):
                    self.num[i] = lpf_numerator_coef[0][i]
                    self.den[i] = lpf_denominator_coef[0][i]
            elif sampleFreq == 1000:
                for i in range (3):
                    self.num[i] = lpf_numerator_coef[1][i]
                    self.den[i] = lpf_denominator_coef[1][i]
        elif filter_type == 'highpass':
            if sampleFreq == 500:
                for i in range (3):
                    self.num[i] = hpf_numerator_coef[0][i]
                    self.den[i] = hpf_denominator_coef[0][i]
            elif sampleFreq == 1000:
                for i in range (3):
                    self.num[i] = hpf_numerator_coef[1][i]
                    self.den[i] = hpf_denominator_coef[1][i]

    def update(self, input):
        tmp = (input - self.den[1] * self.states[0] - self.den[2] * self.states[1]) / self.den[0];
        output = self.num[0] * tmp + self.num[1] * self.states[0] + self.num[2] * self.states[1];
        self.states[1] = self.states[0];
        self.states[0] = tmp;
        return output;


class Filter4nd(object):
    def __init__(self, sampleFreq):
        self.states = [0, 0, 0, 0]
        self.num = [0, 0, 0, 0, 0, 0]
        self.den = [0, 0, 0, 0, 0, 0]
        self.gain = 0
        for i in range (4):
            self.states[i] = 0
        if sampleFreq == 500:
            for i in range(6):
                self.num[i] = ahf_numerator_coef_50Hz[0][i];
                self.den[i] = ahf_denominator_coef_50Hz[0][i];
            self.gain = ahf_output_gain_coef_50Hz[0];
        elif sampleFreq == 1000:
            for i in range(6):
                self.num[i] = ahf_numerator_coef_50Hz[1][i];
                self.den[i] = ahf_denominator_coef_50Hz[1][i];
            self.gain = ahf_output_gain_coef_50Hz[1];

    def update(self, input):
        stageOut = self.num[0] * input + self.states[0]
        self.states[0] = (self.num[1] * input + self.states[1]) - self.den[1] * stageOut
        self.states[1] = self.num[2] * input - self.den[2] * stageOut;
        stageIn = stageOut;
        stageOut = self.num[3] * stageOut + self.states[2];
        self.states[2] = (self.num[4] * stageIn + self.states[3]) - self.den[4] * stageOut;
        self.states[3] = self.num[5] * stageIn - self.den[5] * stageOut;
        output = self.gain * stageOut;
        return output


class FiltersCombination(object):
    def __init__(self, sampleFreq):
        self.LPF = Filter2nd('lowpass', sampleFreq)
        self.HPF = Filter2nd('highpass', sampleFreq)
        self.AHF = Filter4nd(sampleFreq)

    def EMGfilter_update(self, input):
        output = self.AHF.update(input)
        output = self.LPF.update(output)
        output = self.HPF.update(output)
        return output

# import numpy as np
# import matplotlib.pyplot as plt
# import pandas as pd
# if __name__ == "__main__":
#     Filters = FiltersCombination(1000)
#
#     def sine_generator(fs, sinefreq, duration):
#         T = duration
#         nsamples = fs * T
#         w = 2. * np.pi * sinefreq
#         t_sine = np.linspace(0, T, nsamples, endpoint=False)
#         y_sine = np.sin(w * t_sine)
#         result = pd.DataFrame({
#             'data': y_sine}, index=t_sine)
#         return result
#
#
#     # Demonstrate the use of the filter.
#     # First make some data to be filtered.
#     T = 1.0  # seconds
#     n = int(T * 1000)  # total number of samples
#     t = np.linspace(0, T, n, endpoint=False)
#     # "Noisy" data.  We want to recover the 1.2 Hz signal from this.
#     data = 1 * np.sin(40 * 2 * np.pi * t) + 1 * np.cos(300 * 2 * np.pi * t)
#     y = []
#     for i in range(data.size):
#         # Filter the data, and plot both the original and filtered signals.
#         y.append(Filters.EMGfilter_update(data[i]))
#
#     # plt.subplot(2, 1, 2)
#     plt.plot(t, data, 'b-', label='data')
#     plt.plot(t, y, 'g-', linewidth=2, label='filtered data')
#     plt.xlabel('Time [sec]')
#     plt.grid()
#     plt.legend()
#
#     plt.subplots_adjust(hspace=0.35)
#     plt.show()
