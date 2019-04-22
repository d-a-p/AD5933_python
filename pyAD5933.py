import smbus
from math import sqrt
import matplotlib.pyplot as plt


AD5933_FUNCTION_NOP = 0x0

AD5933_CONTROL_INT_SYSCLK = (0x0 << 3)
AD5933_INTERNAL_SYS_CLK = 16000000

# List of all the registers
AD5933_REG_CONTROL_HB = CONTROL_REG0 = 			0x80
AD5933_REG_CONTROL_LB = CONTROL_REG1 = 			0x81

FREQ_MIN_REG0 = 		0x82
FREQ_MIN_REG1 = 		0x83
FREQ_MIN_REG2 = 		0x84

FREQ_INC_REG0 = 		0x85
FREQ_INC_REG1 = 		0x86
FREQ_INC_REG2 = 		0x87

INC_NUM_REG0 = 			0x88
INC_NUM_REG1 = 			0x89

STTL_TIME_CY_NUM_REG0 = 0x8A
STTL_TIME_CY_NUM_REG1 = 0x8B

AD5933_REG_STATUS = STATUS_REG = 			0x8F

TEMP_DATA_REG0 = 		0x92
TEMP_DATA_REG1 = 		0x93

AD5933_REG_REAL_DATA = REAL_DATA_REG0 = 		0x94
REAL_DATA_REG1 = 		0x95

AD5933_REG_IMAG_DATA = IMG_DATA_REG0 = 		0x96
IMG_DATA_REG1 = 		0x97

MAX_FREQ = 				100000
MIN_FREQ = 				1000

# Device address
ADDR_DEV = 0x0D  ######################## find the right address

# AD5933 commands
AD5933_STAT_DATA_VALID  = (0x1 << 1)
AD5933_CONTROL_RESET = INIT_WITH_START_FREQ = 	(0x01 << 4)

AD5933_FUNCTION_STANDBY = 0xB
AD5933_FUNCTION_INIT_START_FREQ = 0x1
AD5933_FUNCTION_START_SWEEP = 0x2
AD5933_FUNCTION_REPEAT_FREQ = 0x4

# Voltage indexes
AD5933_RANGE_1000mVpp = V_OUT_1 = 				3
AD5933_RANGE_2000mVpp = V_OUT_2 = 				0
AD5933_RANGE_400mVpp = V_OUT_04 = 				2
AD5933_RANGE_200mVpp = V_OUT_02 = 				1

def control_func(val):
	return val << 4

def control_range(val):
	return val << 1

def control_pga_gain(val):
	return val << 0

def int_to_signed_short(value):
    return -(value & 0x8000) | (value & 0x7fff)


class AD5933:

	def __init__(self, clk=AD5933_INTERNAL_SYS_CLK):
		print("Device clock: ", clk)

		self.bus = smbus.SMBus(1)  # take from 0: /dev/i2c_0 or 1; /dev/i2c_1
		self.inc = 0
		self. freq_inc = 0
		self.start_freq = 0
		self.cRange = V_OUT_2
		self.cGain = 1
		self.gainFactor = 0.0
		self.clk = clk

    #config sweep
	def set_freq_range(self, freq=1000, inc=500, freq_inc=198):
		"""
		Set the frequency sweep parameters

		min -- start frequency
		inc -- number of incrments(points)
		freq_inc -- frequency incrment
		"""

		if freq <= MIN_FREQ:
			freq = MIN_FREQ

		elif freq > MAX_FREQ:
			freq = MAX_FREQ

		if (inc * freq_inc) > MAX_FREQ:
				inc = min(int(MAX_FREQ // freq_inc), 511)

		self.inc = inc
		self.start_freq = freq
		self.freq_inc = freq_inc
		freq = int((float(freq *4) / self.clk) * pow(2, 27))
		freq_inc = int((float(freq_inc*4)/ self.clk) * pow(2, 27))

		self.set_reg_value(FREQ_MIN_REG0, freq, 3)
		self.set_reg_value(FREQ_INC_REG0, freq_inc, 3)
		self.set_reg_value(INC_NUM_REG0, inc, 2)


    # Low level methods
	def read_reg(self, reg):
		return self.bus.read_byte_data(ADDR_DEV, reg)

	def write_reg(self, reg, value):
		return self.bus.write_byte_data(ADDR_DEV, reg, value)


	def set_range_and_gain(self, frange, gain):
		self.set_reg_value(AD5933_REG_CONTROL_HB,
				 control_func(AD5933_FUNCTION_NOP) |
				 control_range(frange) |
				 control_pga_gain(gain),
				 1);
		self.cRange = frange
		self.cGain = gain
		print("done set range and gain")


	def set_reg_value(self, reg, val, nbytes):
		writeData= [0,0]
		for byte in range(nbytes):
			writeData[0] = reg + nbytes - byte - 1
			writeData[1] = (val >> (byte * 8)) & 0xFF
			self.write_reg(writeData[0],writeData[1])

	def get_reg_value(self, reg, nbytes):
		rval = 0
		tmp = 0
		for byte in range(nbytes):
			tmp = self.read_reg(reg)
			rval = rval << 8
			rval += tmp
			reg = reg + 1
		return rval

	def reset(self):
		self.set_reg_value(AD5933_REG_CONTROL_LB,
								AD5933_CONTROL_RESET | AD5933_CONTROL_INT_SYSCLK,
								1);

	def start_sweep(self):
		self.set_reg_value(AD5933_REG_CONTROL_HB,
                            control_func(AD5933_FUNCTION_STANDBY) |
                            control_range(self.cRange) |
                            control_pga_gain(self.cGain),
                            1);
		self.reset();
		print("reset done")
		self.set_reg_value(AD5933_REG_CONTROL_HB,
							   control_func(AD5933_FUNCTION_INIT_START_FREQ)|
							   control_range(self.cRange) |
							   control_pga_gain(self.cGain),
							   1);
		# Start the Sweep
		self.set_reg_value(AD5933_REG_CONTROL_HB,
						   control_func(AD5933_FUNCTION_START_SWEEP) |
						   control_range(self.cRange) |
						   control_pga_gain(self.cGain),
						   1);

		status = 0
		while((status & AD5933_STAT_DATA_VALID) == 0):
			status = self.get_reg_value(AD5933_REG_STATUS,1)
			print("status:", status, self.cRange, self.cGain)


	def cal_gain_factor(self, calibImp, freqFun = AD5933_FUNCTION_REPEAT_FREQ):
		gainFactor = 0
		magnitude  = 0
		status     = 0


		self.set_reg_value(AD5933_REG_CONTROL_HB,
							control_func(freqFun)|
                            control_range(self.cRange) |
                            control_pga_gain(self.cGain),
							1);

		#Wait for data received to be valid
		while((status & AD5933_STAT_DATA_VALID) == 0):
			status = self.get_reg_value(AD5933_REG_STATUS, 1)

		RealPart = 0
		ImagPart = 0
		tmp = 0
		registerAddress = AD5933_REG_REAL_DATA
		regData = [0,0]
		for byte in range(2):
			tmp = self.read_reg(registerAddress)
			RealPart = RealPart << 8
			RealPart += tmp;
			registerAddress = registerAddress + 1
			regData[byte] = RealPart
		print(regData)
		tmp = 0
		registerAddress = AD5933_REG_IMAG_DATA
		for byte in range(2):
			tmp = self.read_reg(registerAddress)
			ImagPart = ImagPart << 8
			ImagPart += tmp
			registerAddress = registerAddress + 1
			regData[byte] = ImagPart
		print(regData)
		RealPart = int_to_signed_short(RealPart)
		ImagPart = int_to_signed_short(ImagPart)

		magnitude = sqrt((RealPart * RealPart) + (ImagPart * ImagPart))
		gainFactor = 1 / (magnitude * calibImp)
		print("Calibration Step:\n\tR=%hi\n\tI=%hi\n\t|Z|=%f\n",RealPart,ImagPart,magnitude)
		self.gainFactor = gainFactor
		return gainFactor

	def cal_impedance(self):
		magnitude  = 0;
		impedance  = 0;
		status     = 0;
		self.set_reg_value(AD5933_REG_CONTROL_HB,
								control_func(AD5933_FUNCTION_REPEAT_FREQ)|
								control_range(self.cRange) |
								control_pga_gain(self.cGain),
								1);

		while((status & AD5933_STAT_DATA_VALID) == 0):
			status = self.get_reg_value(AD5933_REG_STATUS,1);


		RealPart = 0;
		ImagPart = 0;
		byte     = 0;
		tmp = 0;

		registerAddress = AD5933_REG_REAL_DATA;
		for byte in range(2):
			tmp = self.read_reg(registerAddress);
			RealPart = RealPart << 8;
			RealPart += tmp;
			registerAddress = registerAddress + 1;

		registerAddress = AD5933_REG_IMAG_DATA;
		for byte in range(2):
			tmp = self.read_reg(registerAddress);
			ImagPart = ImagPart << 8;
			ImagPart += tmp;
			registerAddress = registerAddress + 1;
		RealPart = int_to_signed_short(RealPart)
		ImagPart = int_to_signed_short(ImagPart)

		magnitude = sqrt((RealPart * RealPart) + (ImagPart * ImagPart));
		impedance = 1.0 / (magnitude * self.gainFactor)

		return impedance;

	def cal_unknow_impedance(self):
		Z_MOD = [0]*self.inc
		for i in range(self.inc):
			impedance = self.cal_impedance();
			Z_MOD[i] = impedance;
			CurrentFrequency = self.start_freq + self.freq_inc*i;

			print("Impedance read: %f ohms (@ %lu Hz)\n\r", impedance, CurrentFrequency);
		return Z_MOD



#below variables will be from user
impedance = 156.0
gain = 1
range = 1
npoints = 10
freq_inc = 1000
start_freq = 10000


def recalibrate_gain(impedance, AD5933_CALIBRATION_IMPEDANCE = user_cal_impedance):
	return 100*abs((AD5933_CALIBRATION_IMPEDANCE-impedance))/float(AD5933_CALIBRATION_IMPEDANCE)


if __name__ == '__main__':
    sensor = AD5933()
    sensor.set_range_and_gain(gain, range)
    sensor.set_freq_range(start_freq, inc = npoints,  freq_inc = freq_inc)
    print("config sweep done")
    sensor.start_sweep()
    print("sweep done")
    gainFactor = sensor.cal_gain_factor(impedance)
    print("gainFactor:", gainFactor)
    impedance = sensor.cal_impedance()
    print("imp |z|:", impedance)
    print("recalibrated imp:", recalibrate_gain(impedance))
    zmod = sensor.cal_unknow_impedance()
    plt.plot(zmod)
    plt.show()
