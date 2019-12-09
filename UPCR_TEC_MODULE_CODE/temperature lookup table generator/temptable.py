from math import *


class Thermistor:
    "class to do thermistor maths"

    def __init__(self, adc_range, vref, t0, r0, r1, B, N):
        self.r1 = r1  # R1 in the resistor divider (thermistor being R2)
        self.r0 = r0  # stated resistance ex: 10K
        self.t0 = t0 + 273.15  # temperature at stated resistance ex: 25C
        self.B = B  # stated Beta coefficient ex 3380
        self.N = N  # number of values desired in the lookup table
        self.adc_range = adc_range  # resolution of the ADC ex: 12 bit adc = 4096
        self.vref = vref  # reference voltage of the ADC
        self.k = r0 * exp(-B / self.t0)  # constant part of calculation

    def drawtable(self):
        txt_file = open(
            "c:/Users/rarma/Dropbox/HAX/Crimson Dynamic/Crimson_dynamic/REV2/table.txt",
            "w",
        )
        for x in range(1, self.adc_range, int(self.adc_range / self.N)):
            vout = (x / 4096) * self.vref
            R2 = self.vref - vout
            R2 = 1 / R2
            R2 = R2 * vout * self.r1

            temp = (self.B / log(R2 / self.k)) - 273.15
            # print(temp)
            # we don't need negative temperatures in the table because the UPCR device doesn't support it
            if temp > 0:
                # spit out the table in the right format for C
                print("{ .x=", x, ", .y=", temp, "}", file=txt_file)
        txt_file.close()


A = Thermistor(4096, 3.3, 25, 10000, 10000, 3380, 200)
A.drawtable()

