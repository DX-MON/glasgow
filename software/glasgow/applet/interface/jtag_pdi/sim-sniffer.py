#!/usr/bin/env python
from nmigen import *
from nmigen.lib.fifo import AsyncFIFO
from nmigen.compat import TSTriple
from nmigen.sim import Simulator

from sys import argv, path
from pathlib import Path

gatewarePath = Path(argv[0]).resolve().parent.parent.parent.parent.parent
path.insert(0, str(gatewarePath))

from glasgow.applet.interface.jtag_pdi.sniffer import JTAGPDISnifferSubtarget

class JTAGPads:
	def __init__(self, *, name = None):
		layout = [
			("tck", 1),
			("tms", 1),
			("tdi", 1),
			("tdo", 1),
			("srst", 1),
		]

		for pin, width in layout:
			triple = TSTriple(width, name = pin)
			setattr(self, f'{pin}_t', triple)

pads = JTAGPads()
dataFIFO = AsyncFIFO(width = 8, depth = 1024)

subtarget = JTAGPDISnifferSubtarget(pads = pads, in_fifo = dataFIFO)

tck = pads.tck_t.i
tms = pads.tms_t.i
tdi = pads.tdi_t.i
tdo = pads.tdo_t.i

def benchSync():
	yield

def resetJTAG():
	yield tms.eq(1)
	yield
	yield
	yield
	yield
	yield tms.eq(0)
	yield

def jtagInsn(insn):
	yield tms.eq(1)
	yield
	yield
	yield tms.eq(0)
	yield
	yield
	yield tdi.eq(insn & 1)
	insn >>= 1
	yield
	yield tdi.eq(insn & 1)
	insn >>= 1
	yield tdo.eq(0)
	yield
	yield tdi.eq(insn & 1)
	insn >>= 1
	yield
	yield tdi.eq(insn & 1)
	insn >>= 1
	yield tms.eq(1)
	yield
	yield tdi.eq(1)
	yield
	yield tms.eq(0)
	yield tdo.eq(1)
	yield

def readIDCode():
	yield tms.eq(1)
	yield
	yield tms.eq(0)
	yield
	yield
	yield tdi.eq(0)
	# F
	yield tdo.eq(1)
	yield
	yield tdo.eq(1)
	yield
	yield tdo.eq(1)
	yield
	yield tdo.eq(1)
	yield
	# 3
	yield tdo.eq(1)
	yield
	yield tdo.eq(1)
	yield
	yield tdo.eq(0)
	yield
	yield tdo.eq(0)
	yield
	# 0
	yield tdo.eq(0)
	yield
	yield tdo.eq(0)
	yield
	yield tdo.eq(0)
	yield
	yield tdo.eq(0)
	yield
	# 2
	yield tdo.eq(0)
	yield
	yield tdo.eq(1)
	yield
	yield tdo.eq(0)
	yield
	yield tdo.eq(0)
	yield
	# 4
	yield tdo.eq(0)
	yield
	yield tdo.eq(0)
	yield
	yield tdo.eq(1)
	yield
	yield tdo.eq(0)
	yield
	# 8
	yield tdo.eq(0)
	yield
	yield tdo.eq(0)
	yield
	yield tdo.eq(0)
	yield
	yield tdo.eq(1)
	yield
	# 9
	yield tdo.eq(1)
	yield
	yield tdo.eq(0)
	yield
	yield tdo.eq(0)
	yield
	yield tdo.eq(1)
	yield
	# 6
	yield tdo.eq(0)
	yield
	yield tdo.eq(1)
	yield
	yield tdo.eq(1)
	yield
	yield tdo.eq(0)
	yield tms.eq(1)
	yield
	yield tdi.eq(1)
	yield
	yield tms.eq(0)
	yield tdo.eq(1)
	yield

def jtagPDI(dataIn, dataOut):
	byteIn = dataIn[0]
	byteOut = dataOut[0]
	yield tms.eq(1)
	yield
	yield tms.eq(0)
	yield
	yield
	for bit in range(8):
		yield tdi.eq(byteIn & 1)
		byteIn >>= 1
		yield tdo.eq(byteOut & 1)
		byteOut >>= 1
		yield
	yield tdi.eq(dataIn[1])
	yield tdo.eq(dataOut[1])
	yield tms.eq(1)
	yield
	yield tdi.eq(1)
	yield
	yield tms.eq(0)
	yield tdo.eq(1)
	yield

def benchJTAG():
	yield tms.eq(0)
	yield tdi.eq(1)
	yield tdo.eq(1)
	yield
	yield from resetJTAG()
	yield
	yield from jtagInsn(3)
	yield
	yield from readIDCode()
	yield
	yield from jtagInsn(7)
	yield
	yield from jtagPDI((0xC0, 0), (0xEB, 1))
	yield
	yield from jtagPDI((0xFD, 1), (0xEB, 1))
	yield
	yield from jtagPDI((0x80, 1), (0xEB, 1))
	yield
	yield from jtagPDI((0x00, 0), (0x00, 0))
	yield
	yield
	yield from jtagPDI((0xA1, 1), (0xEB, 1))
	yield
	yield from jtagPDI((0x02, 1), (0xEB, 1))
	yield
	yield from jtagPDI((0x00, 0), (0xEB, 1))
	yield
	yield
	yield from jtagPDI((0x24, 0), (0xEB, 1))
	yield
	yield from jtagPDI((0x00, 0), (0x1E, 0))
	yield
	yield from jtagPDI((0x00, 0), (0x98, 1))
	yield
	yield from jtagPDI((0x00, 0), (0x42, 0))
	yield
	yield
	yield from jtagPDI((0x4C, 1), (0xEB, 1))
	yield
	yield from jtagPDI((0xCA, 0), (0xEB, 1))
	yield
	yield from jtagPDI((0x01, 1), (0xEB, 1))
	yield
	yield from jtagPDI((0x00, 0), (0xEB, 1))
	yield
	yield from jtagPDI((0x01, 1), (0xEB, 1))
	yield
	yield from jtagPDI((0x00, 0), (0xEB, 1))
	yield
	yield
	yield
	yield

sim = Simulator(subtarget)
# Define the JTAG clock to have a period of 1/4MHz
#sim.add_clock(250e-9, domain = 'jtag')
sim.add_clock(2e-6, domain = 'jtag')
# Define the system clock to have a period of 1/48MHz
sim.add_clock(20.8e-9)

sim.add_sync_process(benchSync, domain = 'sync')
sim.add_sync_process(benchJTAG, domain = 'jtag')

with sim.write_vcd('jtag_pdi-sniffer.vcd'):
	sim.reset()
	sim.run()
