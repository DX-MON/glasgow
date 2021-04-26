import logging
import argparse
from enum import Enum
from nmigen import *
from nmigen.lib.cdc import FFSynchronizer

from ... import *

class TAPInstructions(Enum):
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
		insn = Signal(4, decoder = TAPInstructions)
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
						insn.eq(TAPInstructions.idCode)
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
				dataOut.eq(Cat(dataOut[1:32], tdo))
			]
		with m.Elif(updateDR):
			with m.If(insn == TAPInstructions.idCode):
				m.d.jtag += idCode.eq(dataIn)
			with m.Elif(insn == TAPInstructions.pdiCom):
				m.d.jtag += [
					pdiDataIn.eq(dataIn[22:32]),
					pdiDataOut.eq(dataOut[22:32]),
					pdiReady.eq(1)
				]

		with m.If(shiftIR):
			m.d.jtag += insnNext.eq(Cat(insnNext[1:4], tdi))
		with m.Elif(updateIR):
			m.d.jtag += insn.eq(insnNext)

		return m

class JTAGPDISubtarget(Elaboratable):
	def __init__(self, pads):
		self._pads = pads

	def elaborate(self, platform):
		m = Module()
		m.submodules.tap = JTAGTAP(self._pads)
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
		))

	async def run(self, device, args):
		iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
		return JTAGPDIInterface(iface)

# -------------------------------------------------------------------------------------------------

class PDIAppletTestCase(GlasgowAppletTestCase, applet=JTAGPDIApplet):
	@synthesis_test
	def test_build(self):
		self.assertBuilds()
