from nmigen import *
from . import TAPInstruction, PDIOpcodes, Header

class JTAGBus(Elaboratable):
	def __init__(self, *, pads):
		self._pads = pads
		self.tck = Signal(reset = 1)
		self.tms = Signal(reset = 0)
		self.tdi = Signal()
		self.tdo = Signal(reset = 1)
		self.srst = Signal(reset = 1)
		self.srst_oe = Signal(reset = 0)

	def elaborate(self, platform) -> Module:
		m = Module()
		pads = self._pads
		m.d.comb += [
			pads.tck_t.o.eq(self.tck),
			pads.tms_t.o.eq(self.tms),
			pads.tdi_t.o.eq(self.tdo),
			self.tdi.eq(pads.tdo_t.i),
			pads.tck_t.oe.eq(1),
			pads.tms_t.oe.eq(1),
			pads.tdi_t.oe.eq(1),
			pads.tdo_t.oe.eq(0),
		]
		if (hasattr, "srst_t"):
			m.d.comb += [
				pads.srst_t.o.eq(self.srst),
				pads.srst_t.oe.eq(self.srst_oe),
			]
		return m

class JTAGTAP(Elaboratable):
	def __init__(self, *, bus : JTAGBus, period_cyc):
		self._bus = bus
		self._period_cyc = period_cyc

		self.resetIssue = Signal()
		self.resetComplete = Signal()

		self.idCode = Signal(32)
		self.idCodeIssue = Signal()
		self.idCodeReady = Signal()
		self.pdiDataIn = Signal(9)
		self.pdiDataOut = Signal(9)
		self.pdiIssue = Signal()
		self.pdiReady = Signal()

	def elaborate(self, platform) -> Module:
		m = Module()
		tck = self._bus.tck
		tms = Signal()
		tdi = Signal()
		tdo = Signal()

		half_cyc = int(self._period_cyc // 2)
		tckTimer = Signal(range(half_cyc), reset = half_cyc - 1)
		resetTimer = Signal(range(3), reset = 0)

		resetIssue = self.resetIssue
		resetComplete = self.resetComplete
		idCode = self.idCode
		idCodeIssue = self.idCodeIssue
		idCodeReady = self.idCodeReady
		pdiDataIn = self.pdiDataIn
		pdiDataOut = self.pdiDataOut
		pdiIssue = self.pdiIssue
		pdiReady = self.pdiReady

		mayUpdate = Signal()
		cycleReady = Signal()

		insn = Signal(4, decoder = TAPInstruction)
		insnNext = Signal.like(insn)
		insnCounter = Signal(range(3))
		dataIn = Signal(32)
		dataOut = Signal(32)
		dataCounter = Signal(range(31))

		with m.FSM(name = "tck"):
			with m.State("TCK-H"):
				m.d.comb += tck.eq(1)
				with m.If(tckTimer != 0):
					m.d.sync += tckTimer.eq(tckTimer - 1)
				with m.Else():
					m.d.sync += [
						tckTimer.eq(half_cyc - 1),
						mayUpdate.eq(1),
					]
					m.next = "TCK-L"
			with m.State("TCK-L"):
				m.d.comb += tck.eq(0)
				with m.If(tckTimer != 0):
					m.d.sync += tckTimer.eq(tckTimer - 1)
					with m.If(cycleReady):
						m.d.sync += [
							self._bus.tms.eq(tms),
							self._bus.tdo.eq(tdo),
							mayUpdate.eq(0),
						]
				with m.Else():
					m.d.sync += [
						tckTimer.eq(half_cyc - 1),
						tdi.eq(self._bus.tdi),
					]
					m.next = "TCK-H"

		m.d.comb += [
			tms.eq(0),
			tdo.eq(1),
		]

		with m.FSM(name = "jtag"):
			with m.State("RESET"):
				with m.If(mayUpdate):
					with m.If(resetTimer != 0):
						m.d.comb += tms.eq(1)
						m.d.sync += resetTimer.eq(resetTimer - 1)
					with m.Else():
						m.d.sync += [
							resetTimer.eq(2),
							insn.eq(TAPInstruction.bypass),
						]
						m.d.comb += resetComplete.eq(1)
						m.next = "IDLE"
					m.d.comb += cycleReady.eq(1)
			with m.State("IDLE"):
				with m.If(mayUpdate):
					with m.If(resetIssue | idCodeIssue | pdiIssue):
						m.d.comb += tms.eq(1)
						m.next = "SELECT-DR"
					m.d.comb += cycleReady.eq(1)

			with m.State("SELECT-DR"):
				with m.If(mayUpdate):
					with m.If(resetIssue):
						m.d.comb += tms.eq(1)
						m.next = "SELECT-IR"
					with m.Elif(
						(idCodeIssue & (insn == TAPInstruction.idCode)) |
						(pdiIssue & (insn == TAPInstruction.pdiCom))
					):
						m.next = "CAPTURE-DR"
					with m.Elif(idCodeIssue | pdiIssue):
						m.d.comb += tms.eq(1)
						m.next = "SELECT-IR"
					m.d.comb += cycleReady.eq(1)
			with m.State("CAPTURE-DR"):
				with m.If(mayUpdate):
					with m.If(idCodeIssue):
						m.d.sync += dataCounter.eq(31)
					with m.If(pdiIssue):
						m.d.sync += [
							dataOut.eq(pdiDataOut),
							dataCounter.eq(9),
						]
					m.next = "SHIFT-DR"
					m.d.comb += cycleReady.eq(1)
			with m.State("SHIFT-DR"):
				with m.If(mayUpdate):
					with m.If(dataCounter != 0):
						m.d.sync += dataCounter.eq(dataCounter - 1)
					with m.Else():
						m.d.comb += tms.eq(1)
						m.next = "EXIT-DR"
					m.d.sync += [
						dataOut.eq(Cat(dataOut[1:32], 0)),
						dataIn.eq(Cat(dataIn[1:32], tdi)),
					]
					m.d.comb += [
						tdo.eq(dataOut[0]),
						cycleReady.eq(1),
					]
			with m.State("EXIT-DR"):
				with m.If(mayUpdate):
					m.d.comb += tms.eq(1)
					m.next = "UPDATE-DR"
					m.d.comb += cycleReady.eq(1)
			with m.State("UPDATE-DR"):
				with m.If(mayUpdate):
					with m.If(idCodeIssue):
						m.d.sync += idCode.eq(dataIn)
						m.d.comb += idCodeReady.eq(1)
					with m.Elif(pdiIssue):
						m.d.sync += pdiDataIn.eq(dataIn[23:32])
						m.d.comb += pdiReady.eq(1)
					m.next = "IDLE"
					m.d.comb += cycleReady.eq(1)

			with m.State("SELECT-IR"):
				with m.If(mayUpdate):
					with m.If(resetIssue):
						m.d.comb += tms.eq(1)
						m.next = "RESET"
					with m.Elif(idCodeIssue | pdiIssue):
						m.next = "CAPTURE-IR"
					m.d.comb += cycleReady.eq(1)
			with m.State("CAPTURE-IR"):
				with m.If(mayUpdate):
					with m.If(idCodeIssue):
						m.d.sync += insnNext.eq(TAPInstruction.idCode)
					with m.Elif(pdiIssue):
						m.d.sync += insnNext.eq(TAPInstruction.pdiCom)
					m.d.sync += insnCounter.eq(3)
					m.next = "SHIFT-IR"
					m.d.comb += cycleReady.eq(1)
			with m.State("SHIFT-IR"):
				with m.If(mayUpdate):
					with m.If(insnCounter != 0):
						m.d.sync += [
							insnCounter.eq(insnCounter - 1),
							insnNext.eq(Cat(insnNext[1:4], 0)),
						]
					with m.Else():
						m.d.comb += tms.eq(1)
						m.next = "EXIT-IR"
					m.d.comb += [
						tdo.eq(insnNext[0]),
						cycleReady.eq(1),
					]
			with m.State("EXIT-IR"):
				with m.If(mayUpdate):
					m.d.comb += tms.eq(1)
					m.next = "UPDATE-IR"
					m.d.comb += cycleReady.eq(1)
			with m.State("UPDATE-IR"):
				with m.If(mayUpdate):
					with m.If(idCodeIssue):
						m.d.sync += insn.eq(TAPInstruction.idCode)
					with m.Elif(pdiIssue):
						m.d.sync += insn.eq(TAPInstruction.pdiCom)

					with m.If(idCodeIssue | pdiIssue):
						m.d.comb += tms.eq(1)
						m.next = "SELECT-DR"
					with m.Else():
						m.next = "IDLE"
					m.d.comb += cycleReady.eq(1)
		return m

class PDIController(Elaboratable):
	def __init__(self, *, tap : JTAGTAP):
		self._tap = tap
		self.issue = Signal()
		self.complete = Signal()
		self.needData = Signal()
		self.dataReady = Signal()

		self.cmd = Signal(8)
		self.dataIn = Signal(8)
		self.dataOut = Signal(8)

	def elaborate(self, platform) -> Module:
		m = Module()

		pdiIssue = self.issue
		pdiComplete = self.complete
		pdiNeedData = self.needData
		pdiDataReady = self.dataReady
		tapIssue = self._tap.pdiIssue
		tapReady = self._tap.pdiReady
		tapDataIn = self._tap.pdiDataIn
		tapDataOut = self._tap.pdiDataOut

		opcode = Signal(PDIOpcodes)
		readCount = Signal(32)
		writeCount = Signal(32)
		repCount = Signal(32)
		repeatData = Signal(32)
		sizeA = Signal(5)
		sizeB = Signal(5)
		updateCounts = Signal()
		updateRepeat = Signal()
		newCommand = Signal()

		m.d.comb += [
			opcode.eq(self.cmd[5:8]),
			sizeA.eq(self.cmd[2:4] + 1),
			sizeB.eq(self.cmd[0:2] + 1),
			updateCounts.eq(0),
			updateRepeat.eq(0),
		]

		with m.FSM(name = "pdi"):
			with m.State("IDLE"):
				with m.If(pdiIssue):
					m.d.sync += [
						tapDataOut.eq(Cat(self.cmd, self.cmd.xor())),
						tapIssue.eq(1),
						newCommand.eq(1),
					]
					m.d.comb += updateCounts.eq(1)
					m.next = "SEND-CMD"
			with m.State("SEND-CMD"):
				with m.If(tapReady):
					m.d.sync += [
						tapIssue.eq(0),
						newCommand.eq(0),
					]
					with m.If(writeCount != 0):
						m.d.sync += pdiNeedData.eq(1)
						m.next = "SEND-DATA"
					with m.Else():
						m.next = "RECV-DATA"
			with m.State("SEND-DATA"):
				with m.If(pdiDataReady):
					m.d.sync += [
						pdiNeedData.eq(0),
						tapDataOut.eq(Cat(self.dataOut, self.dataOut.xor())),
						tapIssue.eq(1),
						writeCount.eq(writeCount - 1),
					]
					m.next = "WAIT-DATA"
			with m.State("RECV-DATA"):
				pass
			with m.State("WAIT-DATA"):
				with m.If(tapReady):
					m.d.sync += tapIssue.eq(0)
					with m.If(writeCount != 0):
						m.d.sync += pdiNeedData.eq(1)
						m.next = "SEND-DATA"
					with m.Elif(readCount != 0):
						m.next = "RECV-DATA"
					with m.Else():
						m.d.comb += pdiComplete.eq(1)
						m.next = "IDLE"

		with m.FSM(name = "insn"):
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
					m.d.sync += repeatData.eq(Cat(tapDataIn[0:8], repeatData[0:24]))
					with m.If(writeCount == 1):
						m.next = "UPDATE-REPEAT"
			with m.State("UPDATE-REPEAT"):
				with m.Switch(self.cmd[0:2]):
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

class JTAGPDIInteractiveSubtarget(Elaboratable):
	def __init__(self, *, pads, in_fifo, out_fifo, period_cyc):
		self._pads = pads
		self._in_fifo = in_fifo
		self._out_fifo = out_fifo
		self._period_cyc = period_cyc

	def elaborate(self, platform) -> Module:
		m = Module()
		bus = m.submodules.bus = JTAGBus(pads = self._pads)
		tap = m.submodules.tap = JTAGTAP(bus = bus, period_cyc = self._period_cyc)
		pdi = m.submodules.pdi = PDIController(tap = tap)
		in_fifo = self._in_fifo
		out_fifo = self._out_fifo

		header = Signal(8, decoder = Header)
		pdiCmd = pdi.cmd
		pdiDataIn = pdi.dataIn
		pdiDataOut = pdi.dataOut

		resetIssue = tap.resetIssue
		resetComplete = tap.resetComplete
		idCodeIssue = tap.idCodeIssue
		idCodeReady = tap.idCodeReady
		pdiIssue = pdi.issue
		pdiComplete = pdi.complete
		pdiNeedData = pdi.needData
		pdiDataReady = pdi.dataReady

		with m.FSM():
			with m.State("IDLE"):
				with m.If(out_fifo.r_rdy):
					m.next = "READ-HEADER"
			with m.State("READ-HEADER"):
				m.d.comb += out_fifo.r_en.eq(1)
				m.d.sync += header.eq(out_fifo.r_data)
				m.next = "DECODE-HEADER"
			with m.State("DECODE-HEADER"):
				with m.Switch(header):
					with m.Case(Header.Reset):
						m.d.sync += resetIssue.eq(1)
						m.next = "WAIT-RESET"
					with m.Case(Header.IDCode):
						m.d.sync += idCodeIssue.eq(1)
						m.next = "WAIT-IDCODE"
					with m.Case(Header.PDI):
						m.next = "READ-PDI-CMD"
					with m.Default():
						m.next = "IDLE"

			with m.State("WAIT-RESET"):
				with m.If(resetComplete):
					m.d.sync += resetIssue.eq(0)
					m.d.comb += [
						in_fifo.w_data.eq(1),
						in_fifo.w_en.eq(1),
					]
					m.next = "IDLE"

			with m.State("WAIT-IDCODE"):
				with m.If(idCodeReady):
					m.d.sync += idCodeIssue.eq(0)
					m.next = "SEND-IDCODE-3"
			with m.State("SEND-IDCODE-3"):
				m.d.comb += [
					in_fifo.w_data.eq(tap.idCode[24:32]),
					in_fifo.w_en.eq(1),
				]
				m.next = "SEND-IDCODE-2"
			with m.State("SEND-IDCODE-2"):
				m.d.comb += [
					in_fifo.w_data.eq(tap.idCode[16:24]),
					in_fifo.w_en.eq(1),
				]
				m.next = "SEND-IDCODE-1"
			with m.State("SEND-IDCODE-1"):
				m.d.comb += [
					in_fifo.w_data.eq(tap.idCode[8:16]),
					in_fifo.w_en.eq(1),
				]
				m.next = "SEND-IDCODE-0"
			with m.State("SEND-IDCODE-0"):
				m.d.comb += [
					in_fifo.w_data.eq(tap.idCode[0:8]),
					in_fifo.w_en.eq(1),
				]
				m.next = "IDLE"

			with m.State("READ-PDI-CMD"):
				with m.If(out_fifo.r_rdy):
					m.d.comb += out_fifo.r_en.eq(1)
					m.d.sync += [
						pdiIssue.eq(1),
						pdiCmd.eq(out_fifo.r_data),
					]
					m.next = "HANDLE-PDI"
			with m.State("HANDLE-PDI"):
				with m.If(out_fifo.r_rdy & pdiNeedData):
					m.d.comb += [
						out_fifo.r_en.eq(1),
						pdiDataReady.eq(1),
					]
					m.d.sync += [
						pdiIssue.eq(1),
						pdiDataOut.eq(out_fifo.r_data),
					]
				with m.Elif(pdiComplete):
					m.d.sync += pdiIssue.eq(0)
					m.next = "IDLE"

		return m
