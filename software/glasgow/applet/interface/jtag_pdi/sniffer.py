from nmigen import *
from nmigen.lib.cdc import FFSynchronizer
from . import TAPInstruction, PDIOpcodes, Header

class JTAGTAP(Elaboratable):
	def __init__(self, *, pads):
		self._pads = pads
		self.idCode = Signal(32)
		self.idCodeReady = Signal()
		self.pdiDataIn = Signal(9)
		self.pdiDataOut = Signal(9)
		self.pdiReady = Signal()

	def elaborate(self, platform) -> Module:
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
		idCodeReady = Signal()
		pdiDataIn = self.pdiDataIn
		pdiDataOut = self.pdiDataOut
		pdiReady = self.pdiReady
		insn = Signal(4, decoder = TAPInstruction)
		insnNext = Signal.like(insn)

		m.submodules += [
			FFSynchronizer(idCode, self.idCode),
			FFSynchronizer(idCodeReady, self.idCodeReady),
		]

		m.d.comb += [
			updateDR.eq(0),
			updateIR.eq(0),
			idCodeReady.eq(0),
			pdiReady.eq(0),
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
					m.next = "SELECT-DR"

			with m.State("SELECT-DR"):
				with m.If(tms):
					m.next = "SELECT-IR"
				with m.Else():
					m.next = "CAPTURE-DR"
			with m.State("CAPTURE-DR"):
				with m.If(tms):
					m.next = "EXIT1-DR"
				with m.Else():
					m.d.jtag += shiftDR.eq(1)
					m.next = "SHIFT-DR"
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
					m.next = "CAPTURE-IR"
			with m.State("CAPTURE-IR"):
				with m.If(tms):
					m.next = "EXIT1-IR"
				with m.Else():
					m.d.jtag += shiftIR.eq(1)
					m.next = "SHIFT-IR"
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
				m.d.jtag += idCode.eq(dataOut)
				m.d.comb += idCodeReady.eq(1),
			with m.Elif(insn == TAPInstruction.pdiCom):
				m.d.jtag += [
					pdiDataIn.eq(dataIn[23:32]),
					pdiDataOut.eq(dataOut[23:32]),
				]
				m.d.comb += pdiReady.eq(1)

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

class PDIDissector(Elaboratable):
	def __init__(self, *, tap : JTAGTAP):
		self._tap = tap
		self.data = Signal(8)
		self.sendHeader = Signal()
		self.ready = Signal()
		self.error = Signal()

	def elaborate(self, platform) -> Module:
		m = Module()
		pdiDataIn = Signal(9)
		pdiDataOut = Signal(9)
		pdiReadyNext = Signal()
		pdiReady = Signal()
		pdiStrobe = Signal()

		parityInOK = Signal()
		parityOutOK = Signal()

		data = self.data
		opcode = Signal(PDIOpcodes)
		args = Signal(4)
		readCount = Signal(32)
		writeCount = Signal(32)
		repCount = Signal(32)
		updateCounts = Signal()
		updateCountsNext = Signal()
		updateRepeat = Signal()
		newCommand = Signal()

		m.submodules += [
			FFSynchronizer(self._tap.pdiDataIn, pdiDataIn),
			FFSynchronizer(self._tap.pdiDataOut, pdiDataOut),
			FFSynchronizer(self._tap.pdiReady, pdiReadyNext),
		]

		m.d.comb += pdiStrobe.eq(~pdiReadyNext & pdiReady)
		m.d.sync += pdiReady.eq(pdiReadyNext)

		m.d.comb += [
			parityInOK.eq(pdiDataIn.xor() == 0),
			parityOutOK.eq(pdiDataOut.xor() == 0),
			self.sendHeader.eq(0),
			self.ready.eq(0),
			self.error.eq(0),
			updateCounts.eq(0),
			updateRepeat.eq(0),
		]
		m.d.sync += updateCountsNext.eq(updateCounts)

		with m.FSM(name = "pdiFSM"):
			with m.State("RESET"):
				m.d.sync += opcode.eq(PDIOpcodes.IDLE)
				m.next = "IDLE"
			with m.State("IDLE"):
				with m.If(pdiStrobe):
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
						opcode.eq(pdiDataIn[5:8]),
						args.eq(pdiDataIn[0:4]),
						newCommand.eq(1),
					]
					m.d.comb += [
						self.sendHeader.eq(1),
						updateCounts.eq(1),
					]
				with m.Else():
					m.d.sync += [
						data.eq(pdiDataIn[0:8]),
						writeCount.eq(writeCount - 1),
					]

				with m.If(opcode == PDIOpcodes.REPEAT):
					m.d.comb += updateRepeat.eq(1)
				m.next = "SEND-DATA"
			with m.State("HANDLE-READ"):
				m.d.sync += [
					data.eq(pdiDataOut[0:8]),
					readCount.eq(readCount - 1),
				]
				m.next = "SEND-DATA"
			with m.State("SEND-DATA"):
				with m.If(~updateCountsNext & (writeCount == 0) & (readCount == 0)):
					with m.If(repCount != 0):
						m.d.sync += newCommand.eq(0)
						m.d.comb += updateCounts.eq(1)
					with m.Else():
						m.d.sync += opcode.eq(PDIOpcodes.IDLE)
				m.d.comb += self.ready.eq(1)
				m.next = "IDLE"
			with m.State("PARITY-ERROR"):
				m.d.comb += self.error.eq(1)
				m.d.sync += opcode.eq(PDIOpcodes.IDLE)
				m.next = "IDLE"

		sizeA = Signal(5)
		sizeB = Signal(5)
		repeatData = Signal(32)

		m.d.comb += [
			sizeA.eq(args[2:4] + 1),
			sizeB.eq(args[0:2] + 1),
		]

		with m.FSM(name = "insnFSM"):
			with m.State("IDLE"):
				with m.If(updateCounts):
					m.next = "DECODE"
			with m.State("DECODE"):
				with m.Switch(opcode):
					with m.Case(PDIOpcodes.LDS):
						m.next = "LDS"
					with m.Case(PDIOpcodes.LD):
						m.next = "LD"
					with m.Case(PDIOpcodes.STS):
						m.next = "STS"
					with m.Case(PDIOpcodes.ST):
						m.next = "ST"
					with m.Case(PDIOpcodes.LDCS):
						m.next = "LDCS"
					with m.Case(PDIOpcodes.STCS):
						m.next = "STCS"
					with m.Case(PDIOpcodes.REPEAT):
						m.next = "REPEAT"
					with m.Case(PDIOpcodes.KEY):
						m.next = "KEY"
			with m.State("LDS"):
				m.d.sync += [
					writeCount.eq(sizeA),
					readCount.eq(sizeB),
				]
				m.next = "IDLE"
			with m.State("LD"):
				m.d.sync += [
					writeCount.eq(0),
					readCount.eq(sizeB),
				]
				with m.If((repCount != 0) & ~newCommand):
					m.d.sync += repCount.eq(repCount - 1)
				m.next = "IDLE"
			with m.State("STS"):
				m.d.sync += [
					writeCount.eq(sizeA + sizeB),
					readCount.eq(0),
				]
				m.next = "IDLE"
			with m.State("ST"):
				m.d.sync += [
					writeCount.eq(sizeB),
					readCount.eq(0),
				]
				with m.If((repCount != 0) & ~newCommand):
					m.d.sync += repCount.eq(repCount - 1)
				m.next = "IDLE"
			with m.State("LDCS"):
				m.d.sync += [
					writeCount.eq(0),
					readCount.eq(1),
				]
				m.next = "IDLE"
			with m.State("STCS"):
				m.d.sync += [
					writeCount.eq(1),
					readCount.eq(0),
				]
				m.next = "IDLE"
			with m.State("REPEAT"):
				m.d.sync += [
					writeCount.eq(sizeB),
					readCount.eq(0),
				]
				m.next = "CAPTURE-REPEAT"
			with m.State("KEY"):
				m.d.sync += [
					writeCount.eq(8),
					readCount.eq(0),
				]
				m.next = "IDLE"

			with m.State("CAPTURE-REPEAT"):
				with m.If(updateRepeat):
					m.d.sync += repeatData.eq(Cat(pdiDataIn[0:8], repeatData[0:24]))
					with m.If(writeCount == 1):
						m.next = "UPDATE-REPEAT"
			with m.State("UPDATE-REPEAT"):
				with m.Switch(args[0:2]):
					with m.Case(0):
						m.d.sync += repCount.eq(Cat(repeatData[0:8]))
					with m.Case(1):
						m.d.sync += repCount.eq(Cat(repeatData[8:16], repeatData[0:8]))
					with m.Case(2):
						m.d.sync += repCount.eq(Cat(repeatData[16:24], repeatData[8:16], repeatData[0:8]))
					with m.Case(3):
						m.d.sync += repCount.eq(Cat(repeatData[16:32],
							repeatData[16:24], repeatData[8:16], repeatData[0:8]))
				m.next = "IDLE"
		return m

class JTAGPDISnifferSubtarget(Elaboratable):
	def __init__(self, *, pads, in_fifo):
		self._pads = pads
		self._in_fifo = in_fifo

	def elaborate(self, platform) -> Module:
		m = Module()
		tap = m.submodules.tap = JTAGTAP(pads = self._pads)
		pdi = m.submodules.pdi = PDIDissector(tap = tap)
		in_fifo = self._in_fifo

		idCodeReadyNext = tap.idCodeReady
		idCodeReady = Signal()
		idCodeStrobe = Signal()

		m.d.comb += idCodeStrobe.eq(~idCodeReadyNext & idCodeReady)
		m.d.sync += idCodeReady.eq(idCodeReadyNext)

		with m.FSM():
			with m.State("IDLE"):
				with m.If(idCodeStrobe):
					m.next = "IDCODE-HEADER"
				with m.Elif(pdi.sendHeader):
					m.next = "PDI-HEADER"
				with m.Elif(pdi.ready):
					m.next = "PDI-DATA"
			with m.State("IDCODE-HEADER"):
				m.d.comb += [
					in_fifo.w_data.eq(Header.IDCode),
					in_fifo.w_en.eq(1),
				]
				m.next = "IDCODE-DATA-3"
			with m.State("IDCODE-DATA-3"):
				m.d.comb += [
					in_fifo.w_data.eq(tap.idCode[24:32]),
					in_fifo.w_en.eq(1),
				]
				m.next = "IDCODE-DATA-2"
			with m.State("IDCODE-DATA-2"):
				m.d.comb += [
					in_fifo.w_data.eq(tap.idCode[16:24]),
					in_fifo.w_en.eq(1),
				]
				m.next = "IDCODE-DATA-1"
			with m.State("IDCODE-DATA-1"):
				m.d.comb += [
					in_fifo.w_data.eq(tap.idCode[8:16]),
					in_fifo.w_en.eq(1),
				]
				m.next = "IDCODE-DATA-0"
			with m.State("IDCODE-DATA-0"):
				m.d.comb += [
					in_fifo.w_data.eq(tap.idCode[0:8]),
					in_fifo.w_en.eq(1),
				]
				m.next = "IDLE"
			with m.State("PDI-HEADER"):
				m.d.comb += [
					in_fifo.w_data.eq(Header.PDI),
					in_fifo.w_en.eq(1),
				]
				with m.If(pdi.ready):
					m.next = "PDI-DATA"
				with m.Else():
					m.next = "IDLE"
			with m.State("PDI-DATA"):
				m.d.comb += [
					in_fifo.w_data.eq(pdi.data),
					in_fifo.w_en.eq(1),
				]
				m.next = "IDLE"
		return m
