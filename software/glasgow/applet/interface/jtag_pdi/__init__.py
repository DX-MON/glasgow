import logging
import argparse
from enum import Enum
from nmigen import *
from nmigen.lib.cdc import FFSynchronizer

from ... import *

class TAPInstruction(Enum):
	idCode = 0x3
	pdiCom = 0x7

class JTAGTAP(Elaboratable):
	def __init__(self, pads):
		self._pads = pads
		self.pdiDataIn = Signal(9)
		self.pdiDataOut = Signal(9)
		self.pdiReady = Signal()

	def elaborate(self, platform):
		m = Module()
		tck = self._pads.tck_t.i
		tms = self._pads.tms_t.i
		tdi = self._pads.tdi_t.i
		tdo = self._pads.tdo_t.i
		srst = self._pads.srst_t.i

		m.domains.jtag = ClockDomain()
		m.d.comb += ClockSignal(domain = 'jtag').eq(tck)

		shiftDR = Signal()
		shiftIR = Signal()
		updateDR = Signal()
		updateIR = Signal()
		dataIn = Signal(32)
		dataOut = Signal(32)
		idCode = Signal(32)
		pdiDataIn = self.pdiDataIn
		pdiDataOut = self.pdiDataOut
		pdiReady = self.pdiReady
		insn = Signal(4, decoder = TAPInstruction)
		insnNext = Signal.like(insn)

		m.d.comb += [
			updateDR.eq(0),
			updateIR.eq(0),
		]

		with m.FSM(domain = 'jtag'):
			with m.State("RESET"):
				with m.If(~tms):
					m.d.jtag += [
						shiftDR.eq(0),
						shiftIR.eq(0),
						insn.eq(TAPInstruction.idCode),
					]
					m.next = "IDLE"
			with m.State("IDLE"):
				with m.If(tms):
					m.d.jtag += pdiReady.eq(0)
					m.next = "SELECT-DR"

			with m.State("SELECT-DR"):
				with m.If(tms):
					m.next = "SELECT-IR"
				with m.Else():
					m.next = "CAPTURE-DR"
			with m.State("CAPTURE-DR"):
				with m.If(tms):
					m.d.jtag += shiftDR.eq(1)
					m.next = "SHIFT-DR"
				with m.Else():
					m.next = "EXIT1-DR"
			with m.State("SHIFT-DR"):
				with m.If(tms):
					m.d.jtag += shiftDR.eq(0)
					m.next = "EXIT1-DR"
			with m.State("EXIT1-DR"):
				with m.If(tms):
					m.next = "UPDATE-DR"
				with m.Else():
					m.next = "PAUSE-DR"
			with m.State("PAUSE-DR"):
				with m.If(tms):
					m.next = "EXIT2-DR"
			with m.State("EXIT2-DR"):
				with m.If(tms):
					m.next = "UPDATE-DR"
				with m.Else():
					m.next = "SHIFT-DR"
			with m.State("UPDATE-DR"):
				m.d.comb += updateDR.eq(1)
				with m.If(tms):
					m.next = "SELECT-DR"
				with m.Else():
					m.next = "IDLE"

			with m.State("SELECT-IR"):
				with m.If(tms):
					m.next = "RESET"
				with m.Else():
					m.next = "CAPTURE-DR"
			with m.State("CAPTURE-IR"):
				with m.If(tms):
					m.d.jtag += shiftIR.eq(1)
					m.next = "SHIFT-IR"
				with m.Else():
					m.next = "EXIT1-IR"
			with m.State("SHIFT-IR"):
				with m.If(tms):
					m.d.jtag += shiftIR.eq(0)
					m.next = "EXIT1-IR"
			with m.State("EXIT1-IR"):
				with m.If(tms):
					m.next = "UPDATE-IR"
				with m.Else():
					m.next = "PAUSE-IR"
			with m.State("PAUSE-IR"):
				with m.If(tms):
					m.next = "EXIT2-IR"
			with m.State("EXIT2-IR"):
				with m.If(tms):
					m.next = "UPDATE-IR"
				with m.Else():
					m.next = "SHIFT-IR"
			with m.State("UPDATE-IR"):
				m.d.comb += updateIR.eq(1)
				with m.If(tms):
					m.next = "SELECT-DR"
				with m.Else():
					m.next = "IDLE"

		with m.If(shiftDR):
			m.d.jtag += [
				dataIn.eq(Cat(dataIn[1:32], tdi)),
				dataOut.eq(Cat(dataOut[1:32], tdo)),
			]
		with m.Elif(updateDR):
			with m.If(insn == TAPInstruction.idCode):
				m.d.jtag += idCode.eq(dataIn)
			with m.Elif(insn == TAPInstruction.pdiCom):
				m.d.jtag += [
					pdiDataIn.eq(dataIn[22:32]),
					pdiDataOut.eq(dataOut[22:32]),
					pdiReady.eq(1),
				]

		with m.If(shiftIR):
			m.d.jtag += insnNext.eq(Cat(insnNext[1:4], tdi))
		with m.Elif(updateIR):
			m.d.jtag += insn.eq(insnNext)

		m.d.comb += [
			self._pads.tck_t.oe.eq(0),
			self._pads.tms_t.oe.eq(0),
			self._pads.tdi_t.oe.eq(0),
			self._pads.tdo_t.oe.eq(0),
			self._pads.srst_t.oe.eq(0),
		]
		return m

class PDIOpcodes(Enum):
	(
		LDS, LD, STS, ST,
		LDCS, REPEAT, STCS, KEY,
	) = range(8)

	IDLE = 0xf

class PDIDissector(Elaboratable):
	def __init__(self, tap):
		self._tap = tap
		self.data = Signal(8)
		self.ready = Signal()
		self.error = Signal()

	def elaborate(self, platform):
		m = Module()
		pdiDataIn = Signal(9)
		pdiDataOut = Signal(9)
		pdiReadyNext = Signal()
		pdiReady = Signal()
		pdiStrobe = Signal()

		parityInOK = Signal()
		parityOutOK = Signal()
		process = Signal()

		data = self.data
		opcode = Signal(PDIOpcodes)
		readCount = Signal(32)
		writeCount = Signal(32)
		updateCounts = Signal()

		m.submodules += [
			FFSynchronizer(self._tap.pdiDataIn, pdiDataIn),
			FFSynchronizer(self._tap.pdiDataOut, pdiDataOut),
			FFSynchronizer(self._tap.pdiReady, pdiReadyNext),
		]

		m.d.comb += pdiStrobe.eq(pdiReadyNext & ~pdiReady)
		m.d.sync += [
			pdiReady.eq(pdiReadyNext),
			process.eq(pdiStrobe),
		]

		with m.If(pdiStrobe):
			m.d.sync += [
				parityInOK.eq(pdiDataIn.xor() == 0),
				parityOutOK.eq(pdiDataOut.xor() == 0),
			]

		with m.FSM():
			with m.State("IDLE"):
				with m.If(process):
					m.next = "CHECK-PARITY"
			with m.State("CHECK-PARITY"):
				with m.If((opcode == PDIOpcodes.IDLE) | (writeCount != 0)):
					with m.If(parityInOK):
						m.next = "HANDLE-WRITE"
					with m.Else():
						m.next = "PARITY-ERROR"
				with m.Else():
					with m.If(parityOutOK):
						m.next = "HANDLE-READ"
					with m.Else():
						m.next = "PARITY-ERROR"
			with m.State("HANDLE-WRITE"):
				with m.If(opcode == PDIOpcodes.IDLE):
					m.d.sync += [
						data.eq(pdiDataIn[0:8]),
						opcode.eq(pdiDataIn[4:8]),
						updateCounts.eq(1),
					]
				with m.Else():
					m.d.sync += [
						data.eq(pdiDataIn[0:8]),
						writeCount.eq(writeCount - 1),
					]
				m.next = "SEND-DATA"
			with m.State("HANDLE-READ"):
				m.d.sync += [
					data.eq(pdiDataOut[0:8]),
					readCount.eq(readCount - 1),
				]
				m.next = "SEND-DATA"
			with m.State("SEND-DATA"):
				with m.If(updateCounts):
					m.d.sync += updateCounts.eq(0)
				with m.Elif((writeCount == 0) & (readCount == 0)):
					m.d.sync += opcode.eq(PDIOpcodes.IDLE)
				m.d.comb += self.ready.eq(1)
				m.next = "IDLE"
			with m.State("PARITY-ERROR"):
				m.d.comb += self.error.eq(1)
				m.d.sync += opcode.eq(PDIOpcodes.IDLE)
				m.next = "IDLE"

		#TODO: Build instruction to count handling state machine

		return m

class JTAGPDISubtarget(Elaboratable):
	def __init__(self, pads, in_fifo):
		self._pads = pads
		self._in_fifo = in_fifo

	def elaborate(self, platform):
		m = Module()
		tap = m.submodules.tap = JTAGTAP(self._pads)
		pdi = m.submodules.pdi = PDIDissector(tap)
		in_fifo = self._in_fifo

		with m.If(pdi.ready & in_fifo.writable):
			m.d.comb += [
				in_fifo.din.eq(pdi.data),
				in_fifo.we.eq(1),
			]

		return m

class JTAGPDIInterface:
	def __init__(self, interface):
		self.lower = interface

	async def read(self):
		pass

class JTAGPDIApplet(GlasgowApplet, name="jtag-pdi"):
	logger = logging.getLogger(__name__)
	help = "capture JTAG-PDI traffic"
	description = """
	Capture Atmel JTAG-PDI traffic
	"""

	__pins = ("tck", "tms", "tdi", "tdo", "srst")

	@classmethod
	def add_build_arguments(cls, parser, access):
		super().add_build_arguments(parser, access)

		for pin in ("tdi", "tms", "tdo", "tck"):
			access.add_pin_argument(parser, pin, default = True)
		access.add_pin_argument(parser, "srst", default = True)

	def build(self, target, args):
		self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
		subtarget = iface.add_subtarget(JTAGPDISubtarget(
			pads = iface.get_pads(args, pins = self.__pins),
			in_fifo = iface.get_in_fifo(depth = 8192),
		))

	async def run(self, device, args):
		iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
		return JTAGPDIInterface(iface)

# -------------------------------------------------------------------------------------------------

class PDIAppletTestCase(GlasgowAppletTestCase, applet=JTAGPDIApplet):
	@synthesis_test
	def test_build(self):
		self.assertBuilds()
