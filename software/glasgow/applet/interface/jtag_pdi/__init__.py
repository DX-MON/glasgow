import logging
import argparse
from vcd import VCDWriter
from enum import IntEnum
from nmigen import *
from nmigen.lib.cdc import FFSynchronizer

from ... import *

class TAPInstruction(IntEnum):
	idCode = 0x3
	pdiCom = 0x7
	bypass = 0xf

class PDIOpcodes(IntEnum):
	(
		LDS, LD, STS, ST,
		LDCS, REPEAT, STCS, KEY,
	) = range(8)

	IDLE = 0xf

class Header(IntEnum):
	IDCode = 0x10
	PDI = 0x11
	Reset = 0x1E
	Error = 0x1F

class JTAGPDIInterface:
	def __init__(self, interface):
		self.lower = interface

	async def read(self):
		data = await self.lower.read()
		def process():
			for octet in data:
				yield octet
		return process()

class JTAGPDIApplet(GlasgowApplet, name="jtag-pdi"):
	logger = logging.getLogger(__name__)
	help = "capture JTAG-PDI traffic"
	description = """
	Capture Atmel JTAG-PDI traffic
	"""

	__pins = ("tck", "tms", "tdi", "tdo", "srst")

	def __init__(self, *args, **kwargs):
		super().__init__(*args, **kwargs)
		self._pdi_handlers = {
			PDIOpcodes.LDS: self._handle_lds,
			PDIOpcodes.LD: self._handle_ld,
			PDIOpcodes.STS: self._handle_sts,
			PDIOpcodes.ST: self._handle_st,
			PDIOpcodes.LDCS: self._handle_ldcs,
			PDIOpcodes.STCS: self._handle_stcs,
			PDIOpcodes.REPEAT: self._handle_repeat,
			PDIOpcodes.KEY: self._handle_key,
		}

	@classmethod
	def add_build_arguments(cls, parser : argparse.ArgumentParser, access):
		super().add_build_arguments(parser, access)

		for pin in ("tdi", "tms", "tdo", "tck"):
			access.add_pin_argument(parser, pin, default = True)
		access.add_pin_argument(parser, "srst", default = True)

		parser.add_argument(
			"-f", "--frequency", metavar = "FREQ", type = int, default = 4000,
			help = "set TCK frequency to FREQ kHz (default: %(default)s)"
		)

	def build(self, target, args):
		self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
		if args.raw_file or args.pdi_file:
			from .sniffer import JTAGPDISnifferSubtarget
			subtarget = iface.add_subtarget(JTAGPDISnifferSubtarget(
				pads = iface.get_pads(args, pins = self.__pins),
				in_fifo = iface.get_in_fifo(depth = 8192),
			))
		else:
			from .interactive import JTAGPDIInteractiveSubtarget
			subtarget = iface.add_subtarget(JTAGPDIInteractiveSubtarget(
				pads = iface.get_pads(args, pins = self.__pins),
				in_fifo = iface.get_in_fifo(depth = 1024),
				out_fifo = iface.get_out_fifo(depth = 1024),
				period_cyc = target.sys_clk_freq // (args.frequency * 1000),
			))

	async def run(self, device, args):
		iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
		return JTAGPDIInterface(iface)

	@classmethod
	def add_interact_arguments(cls, parser : argparse.ArgumentParser):
		g_output = parser.add_mutually_exclusive_group()
		g_output.add_argument(
			"--raw-vcd", metavar = "VCD-FILE", type = argparse.FileType("w"), dest = "raw_file",
			help = "write VCD waveforms to VCD-FILE")
		g_output.add_argument(
			"--pdi-vcd", metavar = "VCD-FILE", type = argparse.FileType("w"), dest = "pdi_file",
			help = "write VCD waveforms to VCD-FILE")
		g_output.add_argument(
			"--interactive", default = False, action = "store_true",
			help = "run an interactive PDI prompt")

	async def _write_raw_vcd(self, file, iface : JTAGPDIInterface):
		vcd_writer = VCDWriter(file, timescale = "1 ns", check_values = False)
		pdiClk = vcd_writer.register_var(scope = "", name = "pdiClk", var_type = "wire", size = 1, init = 1)
		pdiData = vcd_writer.register_var(scope = "", name = "pdiData", var_type = "wire", size = 8)

		cycle = 1
		try:
			while True:
				for byte in await iface.read():
					vcd_writer.change(pdiClk, cycle, 0)
					vcd_writer.change(pdiData, cycle, byte)
					cycle += 1
					vcd_writer.change(pdiClk, cycle, 1)
					cycle += 1
		finally:
			vcd_writer.close(cycle)

	def _handle_lds(self, sizeA, sizeB, repCount):
		return sizeB, sizeA

	def _handle_ld(self, sizeA, sizeB, repCount):
		return sizeB * (repCount + 1), 0

	def _handle_sts(self, sizeA, sizeB, repCount):
		return 0, sizeA + sizeB

	def _handle_st(self, sizeA, sizeB, repCount):
		return 0, sizeB * (repCount + 1)

	def _handle_ldcs(self, sizeA, sizeB, repCount):
		return 1, 0

	def _handle_stcs(self, sizeA, sizeB, repCount):
		return 0, 1

	def _handle_repeat(self, sizeA, sizeB, repCount):
		return 0, sizeB

	def _handle_key(self, sizeA, sizeB, repCount):
		return 0, 8

	def _decode_counts(self, insn, args, repCount):
		sizeA = ((args & 0x0C) >> 2) + 1
		sizeB = (args & 0x03) + 1
		return self._pdi_handlers[insn](sizeA, sizeB, repCount)

	def _reverse_count(self, count, byteCount):
		result = 0
		for byte in range(byteCount):
			value = (count >> (8 * byte)) & 0xFF
			result |= value << (8 * (byteCount - byte - 1))
		return result

	async def _write_pdi_vcd(self, file, iface : JTAGPDIInterface):
		vcd_writer = VCDWriter(file, timescale = "1 ns", check_values = False)
		pdiClk = vcd_writer.register_var(scope = "", name = "pdiClk", var_type = "wire", size = 1, init = 1)
		pdiData = vcd_writer.register_var(scope = "", name = "pdiData", var_type = "wire", size = 1, init = 1)

		cycle = 1
		operation = 0
		count = 4
		pdiInsn = PDIOpcodes.IDLE
		readCount = 0
		writeCount = 0
		repCount = 0
		repBytes = 0

		try:
			while True:
				for byte in await iface.read():
					if operation == 0:
						operation = byte
						continue
					elif operation == Header.IDCode:
						count -= 1
						if count == 0:
							operation = 0
							count = 4
						continue
					elif operation == Header.PDI and (readCount == 0 and writeCount == 0):
						pdiInsn = (byte & 0xE0) >> 5
						readCount, writeCount = self._decode_counts(pdiInsn, byte & 0x0F, repCount)
						if pdiInsn == PDIOpcodes.LD or pdiInsn == PDIOpcodes.ST:
							repCount = 0
					else:
						if writeCount != 0:
							writeCount -= 1
						else:
							readCount -= 1

						if pdiInsn == PDIOpcodes.REPEAT:
							repCount <<= 8
							repCount |= byte
							repBytes += 1
							if writeCount == 0:
								repCount = self._reverse_count(repCount, repBytes)
								repBytes = 0

						if readCount == 0 and writeCount == 0:
							operation = 0

					vcd_writer.change(pdiClk, cycle, 0)
					vcd_writer.change(pdiData, cycle, 0)
					cycle += 1
					vcd_writer.change(pdiClk, cycle, 1)
					cycle += 1
					parity = 0
					for i in range(8):
						bit = (byte >> i) & 1
						parity ^= bit
						vcd_writer.change(pdiClk, cycle, 0)
						vcd_writer.change(pdiData, cycle, bit)
						cycle += 1
						vcd_writer.change(pdiClk, cycle, 1)
						cycle += 1
					vcd_writer.change(pdiClk, cycle, 0)
					vcd_writer.change(pdiData, cycle, parity)
					cycle += 1
					vcd_writer.change(pdiClk, cycle, 1)
					cycle += 1
					vcd_writer.change(pdiClk, cycle, 0)
					vcd_writer.change(pdiData, cycle, 1)
					cycle += 1
					vcd_writer.change(pdiClk, cycle, 1)
					cycle += 1
					vcd_writer.change(pdiClk, cycle, 0)
					cycle += 1
					vcd_writer.change(pdiClk, cycle, 1)
					cycle += 1
		finally:
			vcd_writer.close(cycle)

	async def interact(self, device, args, iface):
		if args.raw_file:
			await self._write_raw_vcd(args.raw_file, iface)
		elif args.pdi_file:
			await self._write_pdi_vcd(args.pdi_file, iface)

# -------------------------------------------------------------------------------------------------

class PDIAppletTestCase(GlasgowAppletTestCase, applet=JTAGPDIApplet):
	@synthesis_test
	def test_build(self):
		self.assertBuilds()
