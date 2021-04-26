import logging
import argparse
from vcd import VCDWriter
from nmigen import *
from nmigen.lib.cdc import FFSynchronizer
from nmigen.lib.fifo import AsyncFIFO

from ... import *

class JTAGSignals(Record):
	def __init__(self, name = None):
		layout = [
			("tck", 1),
			("tms", 1),
			("tdi", 1),
			("tdo", 1),
			("srst", 1),
		]
		super().__init__(layout, name = name)

class JTAGAnalyzerBus(Elaboratable):
	def __init__(self, pads):
		self._pads = pads
		self.tck = Signal()
		self.tms = Signal()
		self.tdi = Signal()
		self.tdo = Signal()
		self.srst = Signal()
		self.states = JTAGSignals()

	def elaborate(self, platform):
		m = Module()
		pads = self._pads
		m.submodules += [
			FFSynchronizer(pads.tck_t.i, self.tck),
			FFSynchronizer(pads.tms_t.i, self.tms),
			FFSynchronizer(pads.tdi_t.i, self.tdi),
			FFSynchronizer(pads.tdo_t.i, self.tdo),
			FFSynchronizer(pads.srst_t.i, self.srst),
		]
		m.d.comb += [
			pads.tck_t.oe.eq(0),
			pads.tms_t.oe.eq(0),
			pads.tdi_t.oe.eq(0),
			pads.tdo_t.oe.eq(0),
			pads.srst_t.oe.eq(0),
			self.states.tck.eq(self.tck),
			self.states.tms.eq(self.tms),
			self.states.tdi.eq(self.tdi),
			self.states.tdo.eq(self.tdo),
			self.states.srst.eq(self.srst),
		]
		return m

class JTAGAnalyzerChanges(Elaboratable):
	def __init__(self, bus):
		self.states = bus.states
		self.have_changes = JTAGSignals()
		self.idleStop = Signal()

	def elaborate(self, platform):
		m = Module()

		tck_i = self.states.tck
		tck_r = Signal()
		states_next = self.states
		states = JTAGSignals()
		have_changes = self.have_changes
		idleCycles = Signal(range(16), reset = 0)
		idleCyclesNext = Signal(idleCycles.shape())

		m.d.sync += [
			tck_r.eq(tck_i),
			idleCycles.eq(idleCyclesNext),
		]

		m.d.comb += [
			have_changes.eq(states ^ states_next),
			idleCyclesNext.eq(idleCycles),
			self.idleStop.eq(idleCyclesNext == 15),
		]

		with m.If((tck_i != tck_r) & (states_next.tck == 1)):
			m.d.sync += states.eq(states_next)
			with m.If(have_changes[1:].any()):
				m.d.comb += idleCyclesNext.eq(0)
			with m.Elif(idleCycles != 15):
				m.d.comb += idleCyclesNext.eq(idleCycles + 1)
		return m

class JTAGAnalyzerEventAdapter(Elaboratable):
	def __init__(self, bus, changes, in_fifo):
		self.tck = bus.tck
		self.tms = bus.tms
		self.tdi = bus.tdi
		self.tdo = bus.tdo
		self.srst = bus.srst
		self.changes = changes
		self.in_fifo = in_fifo

	def elaborate(self, platform):
		m = Module()

		tck_i = self.tck
		tck_r = Signal()
		tms_i = self.tms
		tms_r = Signal()
		tdi_i = self.tdi
		tdi_r = Signal()
		tdo_i = self.tdo
		tdo_r = Signal()
		srst_i = self.srst
		srst_r = Signal()

		data = Signal(8)
		triggers = Signal(5)
		trigger = Signal()
		idleStop = self.changes.idleStop

		m.d.sync += [
			tck_r.eq(tck_i),
			tms_r.eq(tms_i),
			tdi_r.eq(tdi_i),
			tdo_r.eq(tdo_i),
			srst_r.eq(srst_i),
		]

		m.d.comb += [
			data.eq(Cat(tck_i, tms_i, tdi_i, tdo_i, srst_i)),
			triggers.eq(Cat(
				tck_i != tck_r,
				tms_i != tms_r,
				tdi_i != tdi_r,
				tdo_i != tdo_r,
				srst_i != srst_r
			)),
			trigger.eq(triggers.any() & ~idleStop),

			self.in_fifo.din.eq(data),
			self.in_fifo.we.eq(self.in_fifo.writable & trigger),
		]
		return m

class JTAGAnalyzerSubtarget(Elaboratable):
	def __init__(self, pads, in_fifo):
		self._pads = pads
		self._in_fifo = in_fifo

	def elaborate(self, platform):
		m = Module()
		m.submodules.bus = JTAGAnalyzerBus(self._pads)
		m.submodules.changes = JTAGAnalyzerChanges(m.submodules.bus)
		m.submodules.event_adapter = JTAGAnalyzerEventAdapter(m.submodules.bus,
			m.submodules.changes, self._in_fifo)
		return m

class JTAGAnalyzerInterface:
	def __init__(self, interface):
		self.lower = interface

	async def read(self):
		data = await self.lower.read()
		def process():
			for octet in data:
				bits = []
				for bit in range(5):
					valueMask = 1 << bit
					value = (octet & valueMask) >> bit
					bits.append(value)
				yield bits
		return process()

class JTAGAnalyzerApplet(GlasgowApplet, name="jtag-analyzer"):
	logger = logging.getLogger(__name__)
	help = "capture JTAG traffic"
	description = """
	Capture JTAG traffic, a specialisation of the analyzer applet.
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
		subtarget = iface.add_subtarget(JTAGAnalyzerSubtarget(
			pads = iface.get_pads(args, pins = self.__pins),
			in_fifo = iface.get_in_fifo(depth = 16384),
		))

	@classmethod
	def add_run_arguments(cls, parser, access):
		return super().add_run_arguments(parser, access)

		g_pulls = parser.add_mutually_exclusive_group()
		g_pulls.add_argument(
			"--pull-ups", default = False, action = "store_true",
			help = "enable pull-ups on all pins")
		g_pulls.add_argument(
			"--pull-downs", default = False, action = "store_true",
			help = "enable pull-downs on all pins")

	async def run(self, device, args):
		pull_low = set()
		pull_high = set()
		# if args.pull_ups:
		# 	pull_high = set(args.pin_set_i)
		# if args.pull_downs:
		# 	pull_low = set(args.pin_set_i)
		iface = await device.demultiplexer.claim_interface(self, self.mux_interface,
			args, pull_low = pull_low, pull_high = pull_high)
		return JTAGAnalyzerInterface(iface)

	@classmethod
	def add_interact_arguments(cls, parser):
		parser.add_argument(
			"file", metavar = "VCD-FILE", type = argparse.FileType("w"),
			help = "write VCD waveforms to VCD-FILE")

	async def interact(self, device, args, iface):
		vcd_writer = VCDWriter(args.file, timescale = "1 ns", check_values = False)
		tckSignal = vcd_writer.register_var(scope = "", name = "tck", var_type = "wire", size = 1, init = 1)
		tmsSignal = vcd_writer.register_var(scope = "", name = "tms", var_type = "wire", size = 1, init = 0)
		tdiSignal = vcd_writer.register_var(scope = "", name = "tdi", var_type = "wire", size = 1, init = 0)
		tdoSignal = vcd_writer.register_var(scope = "", name = "tdo", var_type = "wire", size = 1, init = 0)
		srstSignal = vcd_writer.register_var(scope = "", name = "srst", var_type = "wire", size = 1, init = 0)

		cycle = 0
		try:
			tck = tms = tdi = tdo = srst = 0
			while True:
				for byte in await iface.read():
					tckNext, tmsNext, tdiNext, tdoNext, srstNext = byte
					if tckNext != tck:
						tck = tckNext
						vcd_writer.change(tckSignal, cycle, tck)
					if tmsNext != tms:
						tms = tmsNext
						vcd_writer.change(tmsSignal, cycle, tms)
					if tdiNext != tdi:
						tdi = tdiNext
						vcd_writer.change(tdiSignal, cycle, tdi)
					if tdoNext != tdo:
						tdo = tdoNext
						vcd_writer.change(tdoSignal, cycle, tdo)
					if srstNext:
						srst = srstNext
						vcd_writer.change(srstSignal, cycle, srst)
					cycle += 1
		finally:
			vcd_writer.close(cycle)

# -------------------------------------------------------------------------------------------------

class AnalyzerAppletTestCase(GlasgowAppletTestCase, applet=JTAGAnalyzerApplet):
	@synthesis_test
	def test_build(self):
		self.assertBuilds()
