import struct

class GPIO:
	def __init__(self, bus):
		self._address = 0x3b
		self._bus = bus

	@property
	def id(self):
		return self._bus.read_i2c_block_data(self._address, 0x00, 1)[0]

	@property
	def fw(self):
		return self._bus.read_i2c_block_data(self._address, 0x01, 1)[0]

	@property
	def serial(self):
		data = self._bus.read_i2c_block_data(self._address, 0x02, 4)
		return struct.unpack('<I', bytes(data))[0]

	@property
	def direction(self):
		return self._bus.read_i2c_block_data(self._address, 0x07, 1)[0]

	@direction.setter
	def direction(self, dir):
		self._bus.write_i2c_block_data(self._address, 0x07, [dir])

	@property
	def input(self):
		return self._bus.read_i2c_block_data(self._address, 0x08, 1)[0]

	@property
	def output(self):
		return self._bus.read_i2c_block_data(self._address, 0x09, 1)[0]

	@output.setter
	def output(self, out):
		self._bus.write_i2c_block_data(self._address, 0x09, [out])

	@property
	def interrupt_mask(self):
		return self._bus.read_i2c_block_data(self._address, 0x0e, 1)[0]

	@interrupt_mask.setter
	def interrupt_mask(self, mask):
		self._bus.write_i2c_block_data(self._address, 0x0e, [mask])

	@property
	def interrupt_sense(self):
		data = self._bus.read_i2c_block_data(self._address, 0x0f, 2)
		return struct.unpack('<H', bytes(data))[0]

	@interrupt_sense.setter
	def interrupt_sense(self, sense):
		print([(sense >> 8) & 0xFF, sense & 0xFF])
		self._bus.write_i2c_block_data(self._address, 0x0f, [sense & 0xFF, (sense >> 8) & 0xFF])

	@property
	def interrupt_status(self):
		return self._bus.read_i2c_block_data(self._address, 0x11, 1)[0]
