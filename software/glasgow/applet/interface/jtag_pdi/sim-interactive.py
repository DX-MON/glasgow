#!/usr/bin/env python
from nmigen import *
from nmigen.lib.fifo import AsyncFIFO
from nmigen.compat import TSTriple
from nmigen.sim import Simulator

from sys import argv, path
from pathlib import Path

gatewarePath = Path(argv[0]).resolve().parent.parent.parent.parent.parent
path.insert(0, str(gatewarePath))

from glasgow.applet.interface.jtag_pdi.interactive import JTAGPDIInteractiveSubtarget
from glasgow.applet.interface.jtag_pdi import TAPInstruction, Header

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

class Harness(Elaboratable):
	def __init__(self):
		self.pads = JTAGPads()
		self.dataInFIFO = DomainRenamer({
			"read": "sync",
			"write": "sync",
		})(AsyncFIFO(width = 8, depth = 1024))
		self.dataOutFIFO = DomainRenamer({
			"read": "sync",
			"write": "sync",
		})(AsyncFIFO(width = 8, depth = 1024))

		self.subtarget = JTAGPDIInteractiveSubtarget(pads = self.pads, in_fifo = self.dataInFIFO,
			out_fifo = self.dataOutFIFO, period_cyc = 4000) # 4MHz

	def elaborate(self, platform) -> Module:
		m = Module()
		m.submodules.dataInFIFO = self.dataInFIFO
		m.submodules.dataOutFIFO = self.dataOutFIFO
		m.submodules.subtarget = self.subtarget
		return m

subtarget = Harness()
pads = subtarget.pads
dataInFIFO = subtarget.dataInFIFO
dataOutFIFO = subtarget.dataOutFIFO

tck = pads.tck_t.o
tms = pads.tms_t.o
tdi = pads.tdi_t.o
tdo = pads.tdo_t.i

jtagResetPerform = False
jtagResetComplete = False

idCodePerform = False
idCodeComplete = False

pdiPerform = False
pdiComplete = False

def readData():
	while (yield dataInFIFO.r_rdy) == 0:
		yield
	data = yield dataInFIFO.r_data
	yield dataInFIFO.r_en.eq(1)
	yield
	yield dataInFIFO.r_en.eq(0)
	yield
	return data

def writeData(data):
	yield dataOutFIFO.w_data.eq(data)
	yield dataOutFIFO.w_en.eq(1)
	yield

def benchSync():
	global jtagResetPerform, idCodePerform, pdiPerform

	# Request the hardware to perform a JTAG reset
	while not jtagResetPerform:
		yield
	yield dataOutFIFO.w_data.eq(Header.Reset)
	yield dataOutFIFO.w_en.eq(1)
	yield
	jtagResetPerform = False
	yield dataOutFIFO.w_en.eq(0)
	yield
	while not jtagResetComplete:
		yield
	assert (yield from readData()) == 1

	# Request the hardware to read out the target IDCode
	while not idCodePerform:
		yield
	yield from writeData(Header.IDCode)
	yield dataOutFIFO.w_en.eq(0)
	idCodePerform = False
	while not idCodeComplete:
		yield
	assert (yield from readData()) == 0x69
	assert (yield from readData()) == 0x84
	assert (yield from readData()) == 0x20
	assert (yield from readData()) == 0x3F

	while not pdiPerform:
		yield
	yield from writeData(Header.PDI)
	yield from writeData(0xC0)
	yield from writeData(0xFD)
	yield dataOutFIFO.w_en.eq(0)
	pdiPerform = False
	while not pdiComplete:
		yield

	while not pdiPerform:
		yield
	yield from writeData(Header.PDI)
	yield from writeData(0x80)
	yield dataOutFIFO.w_en.eq(0)
	pdiPerform = False
	while not pdiComplete:
		yield
	assert (yield from readData()) == 0x00

	while not pdiPerform:
		yield
	yield from writeData(Header.PDI)
	yield from writeData(0xA1)
	yield from writeData(0x02)
	yield from writeData(0x00)
	yield dataOutFIFO.w_en.eq(0)
	pdiPerform = False
	while not pdiComplete:
		yield

	while not pdiPerform:
		yield
	yield from writeData(Header.PDI)
	yield from writeData(0x24)
	yield dataOutFIFO.w_en.eq(0)
	pdiPerform = False
	while not pdiComplete:
		yield
	assert (yield from readData()) == 0x1E
	assert (yield from readData()) == 0x98
	assert (yield from readData()) == 0x42

def jtagClock(process):
	def clockTicker():
		coroutine = process()
		response = None
		while True:
			try:
				request = coroutine.send(response)
				if request is None:
					while (yield tck) == 1:
						yield
					while (yield tck) == 0:
						yield
					response = None
				else:
					response = yield request
			except StopIteration:
				return
	return clockTicker

def resetJTAG():
	global jtagResetComplete

	while (yield tms) == 0:
		yield
	resetCycles = 0
	while (yield tms) == 1:
		yield
		resetCycles += 1
	assert resetCycles == 5
	assert (yield tms) == 0
	jtagResetComplete = True
	yield

def jtagInsn(insn):
	# RESET => IDLE
	while (yield tms) == 0:
		yield
	# IDLE => SELECT-DR
	assert (yield tms) == 1
	yield
	# SELECT-DR => SELECT-IR
	assert (yield tms) == 1
	yield
	# SELECT-IR => CAPTURE-IR
	assert (yield tms) == 0
	yield
	# CAPTURE-IR => SHIFT-IR
	assert (yield tms) == 0
	yield
	# SHIFT-IR the bits
	assert (yield tms) == 0
	assert (yield tdi) == (insn & 1)
	insn >>= 1
	yield
	assert (yield tms) == 0
	assert (yield tdi) == (insn & 1)
	insn >>= 1
	yield tdo.eq(0)
	yield
	assert (yield tms) == 0
	assert (yield tdi) == (insn & 1)
	insn >>= 1
	yield
	# SHIFT-IR => EXIT-IR
	assert (yield tms) == 1
	assert (yield tdi) == (insn & 1)
	insn >>= 1
	yield
	# EXIT-IR => UPDATE-IR
	assert (yield tms) == 1
	assert (yield tdi) == 1
	yield tdo.eq(1)
	yield

def writeIDCode():
	global idCodeComplete

	# UPDATE-IR => SELECT-DR
	assert (yield tms) == 1
	yield
	# SELECT-DR => CAPTURE-DR
	assert (yield tms) == 0
	# F
	yield tdo.eq(1)
	yield
	# CAPTURE-DR => SHIFT-DR
	assert (yield tms) == 0
	yield tdo.eq(1)
	yield
	assert (yield tms) == 0
	assert (yield tdi) == 0
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
	yield
	assert (yield tms) == 0
	assert (yield tdi) == 0
	yield
	# SHIFT-DR => EXIT1-DR
	assert (yield tms) == 1
	assert (yield tdi) == 0
	yield
	# EXIT1-DR => UPDATE-DR
	assert (yield tms) == 1
	assert (yield tdi) == 1
	yield
	# UPDATE-DR => IDLE
	assert (yield tms) == 0
	yield tdo.eq(1)
	idCodeComplete = True
	yield

def jtagPDI(dataIn, dataOut):
	byteIn = dataIn[0]
	byteOut = dataOut[0]
	# IDLE
	while (yield tms) == 0:
		yield
	# IDLE | UPDATE-[DR|IR] => SELECT-DR
	assert (yield tms) == 1
	yield
	# SELECT-DR => CAPTURE-DR
	assert (yield tms) == 0
	yield
	for bit in range(8):
		assert (yield tms) == 0
		yield tdo.eq(byteOut & 1)
		byteOut >>= 1
		yield
		assert (yield tdi) == (byteIn & 1)
		byteIn >>= 1
	assert (yield tms) == 0
	yield tdo.eq(dataOut[1])
	yield
	assert (yield tms) == 0
	assert (yield tdi) == dataIn[1]
	yield
	# SHIFT-DR => EXIT1-DR
	assert (yield tms) == 1
	yield
	# EXIT1-DR => UPDATE-DR
	assert (yield tms) == 1
	assert (yield tdi) == 1
	yield
	# UPDATE-DR => IDLE
	assert (yield tms) == 0
	yield tdo.eq(1)
	yield

@jtagClock
def benchJTAG():
	global jtagResetPerform, idCodePerform, pdiPerform, pdiComplete

	assert (yield tms) == 0
	assert (yield tdi) == 1
	yield tdo.eq(1)
	yield
	jtagResetPerform = True
	yield from resetJTAG()
	yield
	idCodePerform = True
	yield from jtagInsn(TAPInstruction.idCode)
	yield from writeIDCode()
	yield
	pdiPerform = True
	yield from jtagInsn(TAPInstruction.pdiCom)
	yield from jtagPDI((0xC0, 0), (0xEB, 1))
	yield from jtagPDI((0xFD, 1), (0xEB, 1))
	pdiComplete = True
	yield
	pdiPerform = True
	yield from jtagPDI((0x80, 1), (0xEB, 1))
	yield from jtagPDI((0x00, 0), (0x00, 0))
	pdiComplete = True
	yield
	pdiPerform = True
	yield from jtagPDI((0xA1, 1), (0xEB, 1))
	yield from jtagPDI((0x02, 1), (0xEB, 1))
	yield from jtagPDI((0x00, 0), (0xEB, 1))
	pdiComplete = True
	yield
	pdiPerform = True
	yield from jtagPDI((0x24, 0), (0xEB, 1))
	yield from jtagPDI((0x00, 9), (0x1E, 0))
	yield from jtagPDI((0x00, 0), (0x98, 1))
	yield from jtagPDI((0x00, 0), (0x42, 0))
	pdiComplete = True
	yield
	# yield
	# yield from jtagPDI((0x4C, 1), (0xEB, 1))
	# yield
	# yield from jtagPDI((0xCA, 0), (0xEB, 1))
	# yield
	# yield from jtagPDI((0x01, 1), (0xEB, 1))
	# yield
	# yield from jtagPDI((0x00, 0), (0xEB, 1))
	# yield
	# yield from jtagPDI((0x01, 1), (0xEB, 1))
	# yield
	# yield from jtagPDI((0x00, 0), (0xEB, 1))
	# yield
	# yield
	# yield
	# yield

sim = Simulator(subtarget)
# Define the system clock to have a period of 1/48MHz
sim.add_clock(20.8e-9)

sim.add_sync_process(benchSync, domain = 'sync')
sim.add_sync_process(benchJTAG, domain = 'sync')

with sim.write_vcd('jtag_pdi-interactive.vcd'):
	sim.reset()
	sim.run()
