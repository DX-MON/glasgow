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
	__nvm_key_bytes = [0xff, 0x88, 0xd8, 0xcd, 0x45, 0xab, 0x89, 0x12]
	__debug_key_bytes = [0x21, 0x81, 0x7c, 0x9f, 0xd4, 0x2d, 0x21, 0x3a]

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

	def build(self, target, args):
		self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
		if args.raw_file or args.pdi_file:
			from .sniffer import JTAGPDISnifferSubtarget
			iface.add_subtarget(JTAGPDISnifferSubtarget(
				pads = iface.get_pads(args, pins = self.__pins),
				in_fifo = iface.get_in_fifo(depth = 8192),
			))
		else:
			from .interactive import JTAGPDIInteractiveSubtarget
			iface.add_subtarget(JTAGPDIInteractiveSubtarget(
				pads = iface.get_pads(args, pins = self.__pins),
				in_fifo = iface.get_in_fifo(depth = 1024),
				out_fifo = iface.get_out_fifo(depth = 1024),
				period_cyc = target.sys_clk_freq // (args.frequency * 1000),
			))

	async def run(self, device, args):
		iface = await device.demultiplexer.claim_interface(self, self.mux_interface, args)
		return JTAGPDIInterface(iface)

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

	@staticmethod
	def _encode_csreg(register : str):
		if register == 'status':
			return 0
		elif register == 'reset':
			return 1
		elif register == 'ctrl':
			return 2
		elif register.isdecimal():
			return int(register)
		return 'Invalid Control/Status register value given'

	@staticmethod
	def _encode_opcode(opcode, sizeA = None, sizeB = None, address = None):
		args = 0
		if address:
			if address > 15:
				return 'Invalid Control/Status register address'
			args = address
		elif sizeA is not None and sizeB is not None:
			if sizeA == 0 or sizeB == 0:
				return 'Size value cannot be 0'
			elif sizeA > 4 or sizeB > 4:
				return 'Size value too large, cannot be more than 4'
			args = ((sizeA - 1) << 2) | (sizeB - 1)
		return (opcode << 5) | args

	@staticmethod
	def _parse_number(value : str):
		if value.startswith('0x'):
			if any(not ((c >= '0' and c <= '9') or (c >= 'a' and c <= 'f'))
					for c in value[2:].lower()):
				return 'Invalid hexadecimal value'
			return int(value, 16)
		elif value.isnumeric():
			return int(value)
		return 'Invalid number'

	@staticmethod
	def _parse_ptr(value : str):
		access = value.lower()
		if access == '*ptr' or access == '*(ptr)':
			return 1
		elif access == '*ptr++' or access == '*(ptr++)':
			return 2
		elif access == 'ptr':
			return 3
		elif access == 'ptr++':
			return 4 # Allow access to the spicy 'reserved' ptr access encoding
		return 'Invalid pointer access'

	@staticmethod
	def _least_bytes_for(value : int):
		if value == (value & 0xFF):
			return 1
		elif value == (value & 0xFFFF):
			return 2
		elif value == (value & 0xFFFFFF):
			return 3
		elif value == (value & 0xFFFFFFFF):
			return 4
		return 'Invalid value, or value too long'

	@staticmethod
	def _suffix_to_bytes(suffix : str):
		if suffix == 'u8':
			return 1
		elif suffix == 'u16':
			return 2
		elif suffix == 'u24':
			return 3
		elif suffix == 'u32':
			return 4
		return 'Invalid instruction suffix'

	@staticmethod
	def _to_bytes(length : int, value : int):
		result = []
		for i in range(length):
			result.append(value & 0xFF)
			value >>= 8
		return result

	@staticmethod
	def _check_data(data, parts : list):
		for i, value in enumerate(data):
			if not isinstance(value, int):
				return f'{value} for data byte {i + 1}'
			elif JTAGPDIApplet._least_bytes_for(value) != 1:
				return f'Value {parts[2 + i]} ({value:#x}) is not a single byte'
		return None

	def _parse_command(self, line : str):
		parts = line.split()
		command = parts[0].lower()
		if command.startswith('lds.'):
			if len(parts) != 2:
				return 'Incorrect number of arguments to LDS instruction'
			sizeB = self._suffix_to_bytes(command.split('.', 1)[1])
			if not isinstance(sizeB, int):
				return sizeB
			address = self._parse_number(parts[1])
			if not isinstance(address, int):
				return address
			sizeA = self._least_bytes_for(address)
			if not isinstance(sizeA, int):
				return sizeA
			return ([self._encode_opcode(PDIOpcodes.LDS, sizeA = sizeA, sizeB = sizeB)] + \
				self._to_bytes(sizeA, address), sizeB)
		elif command.startswith('sts.'):
			if len(parts) < 2:
				return 'Incorrect number of arguments to STS instruction'
			sizeB = self._suffix_to_bytes(command.split('.', 1)[1])
			if not isinstance(sizeB, int):
				return sizeB
			if len(parts) > 2 + sizeB:
				return 'Incorrect number of arguments to STS instruction'
			address = self._parse_number(parts[1])
			if not isinstance(address, int):
				return address
			sizeA = self._least_bytes_for(address)
			if not isinstance(sizeA, int):
				return sizeA
			data = [self._parse_number(part) for part in parts[2:]]
			check = self._check_data(data, parts)
			if check is not None:
				return check
			return ([self._encode_opcode(PDIOpcodes.STS, sizeA = sizeA, sizeB = sizeB)] +
				self._to_bytes(sizeA, address) + data, 0)
		elif command == 'ld':
			if len(parts) != 2:
				return 'Incorrect number of arguments to LD instruction'
			repeats = self.__repeatCount + 1
			self.__repeatCount = 0
			access = self._parse_ptr(parts[1])
			if not isinstance(access, int):
				return access
			return ([self._encode_opcode(PDIOpcodes.LD, sizeA = access, sizeB = 1)], repeats)
		elif command.startswith('ld.'):
			if len(parts) != 2:
				return 'Incorrect number of arguments to LD instruction'
			elif self.__repeatCount != 0: # TODO: allow this at some point.. maybe
				return 'Attempting to repeat complex LD instruction'
			sizeB = self._suffix_to_bytes(command.split('.', 1)[1])
			if not isinstance(sizeB, int):
				return sizeB
			access = self._parse_ptr(parts[1])
			if not isinstance(access, int):
				return access
			return ([self._encode_opcode(PDIOpcodes.LD, sizeA = access, sizeB = sizeB)], sizeB)
		elif command == 'st':
			if len(parts) < 2:
				return 'Incorrect number of arguments to ST instruction'
			repeats = self.__repeatCount + 1
			self.__repeatCount = 0
			if len(parts) > 2 + repeats:
				return 'Incorrect number of arguments to ST instruction'
			access = self._parse_ptr(parts[1])
			if not isinstance(access, int):
				return access
			data = [self._parse_number(part) for part in parts[2:]]
			check = self._check_data(data, parts)
			if check is not None:
				return check
			return ([self._encode_opcode(PDIOpcodes.ST, sizeA = access, sizeB = 1)] + data, 0)
		elif command.startswith('st.'):
			if len(parts) != 3:
				return 'Incorrect number of arguments to ST instruction'
			elif self.__repeatCount != 0: # TODO: allow this at some point.. maybe
				return 'Attempting to repeat complex ST instruction'
			sizeB = self._suffix_to_bytes(command.split('.', 1)[1])
			if not isinstance(sizeB, int):
				return sizeB
			if len(parts) > 2 + sizeB:
				return 'Incorrect number of arguments to ST instruction'
			access = self._parse_ptr(parts[1])
			if not isinstance(access, int):
				return access
			data = [self._parse_number(part) for part in parts[2:]]
			check = self._check_data(data, parts)
			if check is not None:
				return check
			return ([self._encode_opcode(PDIOpcodes.ST, sizeA = access, sizeB = sizeB)] + data, 0)
		elif command == 'ldcs':
			if len(parts) != 2:
				return 'Incorrect number of arguments to LDCS instruction'
			reg = self._encode_csreg(parts[1])
			if not isinstance(reg, int):
				return reg
			return ([self._encode_opcode(PDIOpcodes.LDCS, address = reg)], 1)
		elif command == 'stcs':
			if len(parts) != 3:
				return 'Incorrect number of arguments to STCS instruction'
			reg = self._encode_csreg(parts[1])
			if not isinstance(reg, int):
				return reg
			value = self._parse_number(parts[2])
			check = self._check_data((value,), parts)
			if check is not None:
				return check
			return ([self._encode_opcode(PDIOpcodes.STCS, address = reg), value], 0)
		elif command == 'repeat':
			if len(parts) != 2:
				return 'Incorrect number of arguments to REPEAT instruction'
			repeats = self._parse_number(parts[1])
			if not isinstance(repeats, int):
				return repeats
			sizeB = self._least_bytes_for(repeats)
			if not isinstance(sizeB, int):
				return sizeB
			self.__repeatCount = repeats
			return ([self._encode_opcode(PDIOpcodes.REPEAT, sizeA = 1, sizeB = sizeB)] +
				self._to_bytes(sizeB, repeats), 0)
		elif command == 'key':
			if len(parts) != 2:
				return 'Incorrect number of arguments to KEY instruction'
			which = parts[1].lower()
			if which == 'nvm':
				key = self.__nvm_key_bytes
			elif which == 'debug':
				key = self.__debug_key_bytes
			else:
				return 'Invalid key specification for instruction'
			return ([self._encode_opcode(PDIOpcodes.KEY)] + key, 0)
		return 'Invalid opcode'

	async def _interactive_prompt(self, iface):
		from sys import stdin
		from . import meta

		await iface.write([Header.Reset])
		if meta.to_int8(await iface.read(length = 1)) != 1:
			self.logger.error("Failed to reset target, aborting")
			return
		self.logger.info("Device reset complete")

		await iface.write([Header.IDCode])
		idCode = meta.decode_device_id_code(await iface.read(length = 4))
		self.logger.info(f"Device is a {idCode[1]} {idCode[3]} revision {idCode[2]} ({idCode[0]})")
		self.logger.info("Begining PDI session with device, type 'exit' to leave")

		response = None
		while response != 'exit':
			print("> ", flush=True, end='')
			response = stdin.readline().strip()
			if response == '':
				continue
			if response == 'exit':
				break
			command = self._parse_command(response)
			if isinstance(command, str):
				self.logger.error(command)
				continue
			operation, readCount = command
			await iface.write([Header.PDI] + operation)
			if readCount != 0:
				result = bytes(await iface.read())
			result = bytes(await iface.read(length = readCount))
			self.logger.info(f'Recieved {result}')

	async def interact(self, device, args, iface):
		if args.raw_file:
			await self._write_raw_vcd(args.raw_file, iface)
		elif args.pdi_file:
			await self._write_pdi_vcd(args.pdi_file, iface)
		else:
			await self._interactive_prompt(iface.lower)

# -------------------------------------------------------------------------------------------------

class PDIAppletTestCase(GlasgowAppletTestCase, applet=JTAGPDIApplet):
	@synthesis_test
	def test_build(self):
		self.assertBuilds()
