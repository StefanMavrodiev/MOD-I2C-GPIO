#!/usr/bin/env python3

import smbus
import gpio


import unittest

class TestGPIO(unittest.TestCase):

	def setUp(self):
		self._gpio = gpio.GPIO(smbus.SMBus(1))

	def test_id(self):
		self.assertEqual(self._gpio.id, 0x43)

	def test_fw(self):
		self.assertEqual(self._gpio.fw, 0x01)

	def test_serial(self):
		self.assertEqual(self._gpio.serial, 0x12345678)




def main():
	bus = smbus.SMBus(1)

	g = gpio.GPIO(bus)
	print("ID: 0x{:02X}".format(g.id))
	print("FW: 0x{:02X}".format(g.fw))
	print("SERIAL: 0x{:08X}".format(g.serial))

	g.output = 0x00
	# GPIO0 output
	g.direction = 0xFE
	print(g.interrupt_status)

	g.output = 0x00

	# Enable interrupt
	g.interrupt_mask = 0x02
	g.interrupt_sense = 0x000c

	for i in range(10):
		g.output = 0x01
		print(g.interrupt_status)
		g.output = 0x00
		print(g.interrupt_status)

	# for i in range(0, 256):
	# 	g.output = i

if __name__ == "__main__":
	main()
