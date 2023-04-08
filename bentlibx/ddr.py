#
# Copyright 2023 Bentwire
# Just me messing with DDR primitives.
# 
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *
from litex.soc.integration.doc import AutoDoc, ModuleDoc
from litex.soc.cores.code_8b10b import *

class DDR_8IO(Module, AutoCSR, AutoDoc):

    def __init__(self, pina, pinb, ddr_data_in = None, ddr_data_out = None, 
                 clock_domain="sys", ddr_clock_domain="sys4x"):
        self.reset = Signal()
        self.calib = Signal()
        self.clock_pol = Signal()
        ddr_o   = Signal()
        ddr_i   = Signal()
        ddr_i_dlyd = Signal()
        ddr_oen = Signal()
        ddr_data_i = Signal(8)
        ddr_data_o = Signal(8)
        ddr_data_oen = Signal(4)

        self.specials += Instance("OSER8",
                p_TXCLK_POL = 0b0,
                i_RESET = ResetSignal(clock_domain),
                i_PCLK  = ClockSignal(clock_domain),
                i_FCLK  = ClockSignal(ddr_clock_domain),
                i_TX0   = ddr_data_oen[0],
                i_TX1   = ddr_data_oen[1],
                i_TX2   = ddr_data_oen[2],
                i_TX3   = ddr_data_oen[3],
                i_D0    = ddr_data_o[0],
                i_D1    = ddr_data_o[1],
                i_D2    = ddr_data_o[2],
                i_D3    = ddr_data_o[3],
                i_D4    = ddr_data_o[4],
                i_D5    = ddr_data_o[5],
                i_D6    = ddr_data_o[6],
                i_D7    = ddr_data_o[7],
                o_Q0    = ddr_o, #ddr0.o,
                o_Q1    = ddr_oen, #ddr0.oe,
            )
        self.specials += Instance("IDES8",
                                  i_RESET = ResetSignal(clock_domain),
                                  i_PCLK  = ClockSignal(clock_domain),
                                  i_FCLK  = ClockSignal(ddr_clock_domain),
                                  i_CALIB = self.calib,
                                  i_D = ddr_i_dlyd,
                                  o_Q0 = ddr_data_i[0],
                                  o_Q1 = ddr_data_i[1],
                                  o_Q2 = ddr_data_i[2],
                                  o_Q3 = ddr_data_i[3],
                                  o_Q4 = ddr_data_i[4],
                                  o_Q5 = ddr_data_i[5],
                                  o_Q6 = ddr_data_i[6],
                                  o_Q7 = ddr_data_i[7],
                                  )
        
        ddr_i_sdtap = Signal(reset=1)
        ddr_i_setn  = Signal()
        ddr_i_value = Signal()
        ddr_i_df    = Signal()
        ddr_i_lag   = Signal()
        ddr_i_lead  = Signal()

        ddr_o_df    = Signal()

        # self.specials += Instance("IEM",
        #                           i_D = ddr_i,
        #                           i_RESET = ResetSignal(clock_domain),
        #                           i_CLK = ClockSignal(clock_domain),
        #                           i_MCLK = ClockDomain(ddr_clock_domain),
        #                           o_LAG = ddr_i_lag,
        #                           o_LEAD = ddr_i_lead
        #                           )
        
        self.specials += Instance("IODELAY",
                                  i_DI = ddr_i,
                                  o_DO = ddr_i_dlyd,
                                  i_SDTAP = ddr_i_sdtap,
                                  i_VALUE = ddr_i_value,
                                  i_SETN = ddr_i_setn,
                                  o_DF = ddr_i_df)
        if pinb is None:        
            self.specials += Instance("IOBUF",
                                    i_I = ddr_o,
                                    i_OEN = ddr_oen,
                                    o_O = ddr_i,
                                    io_IO = pina)
                                    #io_IOB = pinb)
        else:
            self.specials += Instance("TLVDS_IOBUF",
                                    i_I = ddr_o,
                                    i_OEN = ddr_oen,
                                    o_O = ddr_i,
                                    io_IO = pina,
                                    io_IOB = pinb)
        
        self._csr_din      = CSRStatus(8, name="in", description="Input Value", reset=0)
        stat_bits          = Signal(6)
        ctrl_bits          = Signal(3)
        self._csr_stat     = CSRStatus(6, name="stat", description="DF LAG and LEAD for IO", reset=0)
        self._csr_dout     = CSRStorage(8, name="out", description="Output Value", reset=0)
        self._csr_dout_oen = CSRStorage(4, name="oen", description="Output Enable Value", reset=0b1111)
        self._csr_calib    = CSRStorage(name="calib", reset=0)

        fields = []
        fields.append(CSRField(name=F"sdtap", description=F"Enable Dynamic Delay", reset=0, values=[
            ("0", "Static", "Static built in delay"),
            ("1", "Dynamic", "Dynamic delay controlled by value and setn bits."),
            ]))

        fields.append(CSRField(name=F"value", description=F"Pulse for delay inc/dec (depends on setn)", reset=0, values=[
            ("0", "0"),
            ("1", "1"),
            ]))

        fields.append(CSRField(name=F"setn", description=F"Increment or decrement delay", reset=0, values=[
            ("0", "INC", "Increment delay on value pulse"),
            ("1", "DEC", "Decrement delay on value p[ulse"),
            ]))
        self._csr_ddr_i_ctrl = CSRStorage(name="input_control", fields=fields, reset=0)

        # Clock domain stuff
        n = 0 if clock_domain == "sys" else 2
        self.specials += [
            MultiReg(self._csr_dout.storage, ddr_data_o, n=n),
            MultiReg(self._csr_dout_oen.storage, ddr_data_oen, n=n),
            MultiReg(ddr_data_i, self._csr_din.status, n=n),
            MultiReg(stat_bits, self._csr_stat.status, n=n),
            MultiReg(self._csr_calib.storage, self.calib, n=n),
            MultiReg(self._csr_ddr_i_ctrl.storage, ctrl_bits, n=n)
        ]
        self.comb += stat_bits.eq(Cat(ddr_i_df, ddr_i_lag, ddr_i_lead, ddr_o_df))
        self.comb += ddr_i_sdtap.eq(ctrl_bits[0])
        self.comb += ddr_i_value.eq(ctrl_bits[1])
        self.comb += ddr_i_setn.eq(ctrl_bits[2])
        sync = getattr(self.sync, clock_domain)
        # sync += If(~self.reset,
        #            _ddr_data_i.eq(ddr_data_i),
        #            ddr_data_o.eq(_ddr_data_o),
        #         ).Else(
        #             _ddr_data_i.eq(0),
        #             ddr_data_o.eq(ddr_data_i),
        #         )

class DDR8_10O(Module, AutoCSR, AutoDoc):
    def __init__(self, pin, pinb=None, clock_domain="sys", par_clock_domain="sys8x_5", ser_clock_domain="sys8x"):

        self.reset      = Signal()

        # self.clk        = ClockSignal(clock_domain)
        # self.reset      = ResetSignal(clock_domain)
        # self.ser_clk    = ClockSignal(ser_clock_domain)
        # self.par_clk    = ClockDomain(par_clock_domain)
        # self.ser_reset  = ResetSignal(ser_clock_domain)
        
        ser_out         = Signal()
        ser_data        = Signal(10)
        self.data       = Signal(8, reset=0x3c)
        self.k          = Signal(reset=1)

        self.specials += Instance("OSER10",
                                  i_D0 = ser_data[0],
                                  i_D1 = ser_data[1],
                                  i_D2 = ser_data[2],
                                  i_D3 = ser_data[3],
                                  i_D4 = ser_data[4],
                                  i_D5 = ser_data[5],
                                  i_D6 = ser_data[6],
                                  i_D7 = ser_data[7],
                                  i_D8 = ser_data[8],
                                  i_D9 = ser_data[9],
                                  i_FCLK = ClockSignal(ser_clock_domain),
                                  i_PCLK = ClockSignal(par_clock_domain),
                                  i_RESET = ResetSignal(ser_clock_domain),
                                  o_Q = ser_out)
        if pinb is None:        
            self.specials += Instance("OBUF",
                                    i_I = ser_out,
                                    o_O = pin)
                                    #io_IOB = pinb)
        else:
            self.specials += Instance("TLVDS_OBUF",
                                    i_I = ser_out,
                                    o_O = pin,
                                    o_OB = pinb)

        self.submodules.encoder = ClockDomainsRenamer(par_clock_domain)(Encoder())

        self.comb += ser_data.eq(self.encoder.output[0])
        self.comb += self.encoder.d[0].eq(self.data)
        self.comb += self.encoder.k[0].eq(self.k)        

class DDR10_8I(Module, AutoCSR, AutoDoc):
    def __init__(self, pin, pinb=None, clock_domain="sys", par_clock_domain="sys8x_5", ser_clock_domain="sys8x", with_csr=True):

        #self.reset  = Signal()
        self.calib  = Signal()
        self.df     = Signal()
        self.sdtap  = Signal()
        self.setn   = Signal()
        self.value  = Signal()
        self.data   = Signal(8)
        self.k      = Signal()

        self.lag     = Signal()
        self.lead    = Signal()
        self.locked  = Signal()
        self.invalid = Signal()

        self.clk        = ClockSignal(clock_domain)
        self.reset      = ResetSignal(clock_domain)
        self.ser_clk    = ClockSignal(ser_clock_domain)
        self.par_clk    = ClockSignal(par_clock_domain)
        # self.ser_reset  = ResetSignal(ser_clock_domain)
        
        ser_in     = Signal()
        ser_in_dlyd     = Signal()
        ser_data        = Signal(10)
        self.ser_data        = Signal(10)

        self.updn = Signal()
        self.stop = Signal()
        self.step = Signal(8)
        self.move = Signal()
        self.load = Signal()
        self.lock = Signal()
        self.flag = Signal()

        clkout = Signal()
        
        # self.specials += Instance("DLL",
        #     p_SCAL_EN  = "false",
        #     i_RESET    = ResetSignal("init"),
        #     i_CLKIN    = clkout,
        #     i_UPDNCNTL = self.updn,
        #     i_STOP     = self.stop,
        #     o_STEP     = self.step,
        #     o_LOCK     = self.lock
        # )

        # self.specials += Instance("DLLDLY",
        #                           i_CLKIN   = self.ser_clk,
        #                           #i_DLLSTEP = ,
        #                           i_DIR     = self.updn,
        #                           i_LOADN   = self.load,
        #                           i_MOVE    = self.move,
        #                           o_CLKOUT  = clkout,
        #                           o_FLAG    = self.flag)
        
        # self.specials += Instance("IEM",
        #                           i_RESET = self.reset,
        #                           i_D = ser_in,
        #                           i_CLK = ClockSignal(ser_clock_domain),
        #                           i_MCLK = ClockSignal(par_clock_domain),
        #                           o_LAG = self.lag,
        #                           o_LEAD = self.lead)        

        lag    = Signal()
        lag_d  = Signal()
        lag_r  = Signal()
        lead   = Signal()
        lead_d = Signal()
        lead_r = Signal()

        self.specials += Instance("IEM",
                                  i_RESET = ResetSignal(par_clock_domain),
                                  i_D = ser_in_dlyd,
                                  i_CLK = ClockSignal(ser_clock_domain),
                                  i_MCLK = ClockSignal(par_clock_domain),
                                  o_LAG = self.lag,
                                  o_LEAD = self.lead)
        
        self.specials += Instance("IDES10",
                                  o_Q0 = ser_data[0],
                                  o_Q1 = ser_data[1],
                                  o_Q2 = ser_data[2],
                                  o_Q3 = ser_data[3],
                                  o_Q4 = ser_data[4],
                                  o_Q5 = ser_data[5],
                                  o_Q6 = ser_data[6],
                                  o_Q7 = ser_data[7],
                                  o_Q8 = ser_data[8],
                                  o_Q9 = ser_data[9],
                                  i_FCLK = ClockSignal(ser_clock_domain),
                                  i_PCLK = ClockSignal(par_clock_domain),
                                  i_RESET = ResetSignal(par_clock_domain),
                                  i_D = ser_in_dlyd,
                                  i_CALIB = self.calib)

        # self.specials += Instance("IDES10",
        #                           o_Q0 = self.ser_data[0],
        #                           o_Q1 = self.ser_data[1],
        #                           o_Q2 = self.ser_data[2],
        #                           o_Q3 = self.ser_data[3],
        #                           o_Q4 = self.ser_data[4],
        #                           o_Q5 = self.ser_data[5],
        #                           o_Q6 = self.ser_data[6],
        #                           o_Q7 = self.ser_data[7],
        #                           o_Q8 = self.ser_data[8],
        #                           o_Q9 = self.ser_data[9],
        #                           i_FCLK = ClockSignal(ser_clock_domain),
        #                           i_PCLK = ClockSignal(par_clock_domain),
        #                           i_RESET = ResetSignal(par_clock_domain),
        #                           i_D = ser_in_dlyd,
        #                           i_CALIB = self.calib)

        self.specials += Instance("IODELAY",
                                  p_C_STATIC_DLY = 0,
                                  i_DI = ser_in,
                                  o_DO = ser_in_dlyd,
                                  i_SDTAP = self.sdtap,
                                  i_VALUE = self.value,
                                  i_SETN = self.setn,
                                  o_DF = self.df)

        if pinb is None:
            self.specials += Instance("IBUF",
                                    i_I = pin,
                                    o_O = ser_in)
        else:
            self.specials += Instance("TLVDS_IBUF",
                                    i_I = pinb,
                                    i_IB = pin,
                                    o_O = ser_in)


        self.submodules.decoder = ClockDomainsRenamer(par_clock_domain)(Decoder())
        #self.submodules.decoder = Decoder()
        #self.comb += self.decoder.ce.eq(1)
        self.comb += self.decoder.input.eq(self.ser_data)
        self.comb += self.data.eq(self.decoder.d)
        self.comb += self.k.eq(self.decoder.k)        
        self.comb += self.invalid.eq(self.decoder.invalid)
#        self.comb += self.lag.eq(lag)
#        self.comb += self.lead.eq(lead)
        self.comb += self.ser_data.eq(~ser_data)
        # sync = getattr(self.sync, clock_domain)
        # sync += lag_d.eq(lag)
        # self.comb += lag_r.eq(lag & ~lag_d)
        # self.sync += self.lag.eq(lag_r)

        if with_csr:
            self.add_csr(clock_domain=clock_domain, par_clock_domain=par_clock_domain, ser_clock_domain=ser_clock_domain)

    def add_csr(self, clock_domain="sys", par_clock_domain="sys8x_5", ser_clock_domain="sys8x"):
        stat_bits          = Signal(3)
        ctrl_bits          = Signal(3)

        fields = []
        fields.append(CSRField(size=8, name=F"in", description=F"RX Data after decode", reset=0, values=[
            ("0", "0"),
            ("1", "1"),
            ]))

        fields.append(CSRField(name=F"k", description=F"Decoded K line.", reset=0, values=[
            ("0", "Data"),
            ("1", "Symbol"),
            ]))

        fields.append(CSRField(name=F"valid", description=F"Decoded data validity", reset=0, values=[
            ("0", "Data Invalid"),
            ("1", "Data Valid"),
            ]))

        self._csr_din = CSRStatus(fields=fields, name="in", description="Decoded serial rx data", reset=0)
        self._csr_raw = CSRStatus(10, name="raw", description="Raw serial rx data", reset=0)

        fields = []
        fields.append(CSRField(name=F"lag", description=F"IEM LAG Output", reset=0, values=[
            ("0", "0"),
            ("1", "1"),
            ]))

        fields.append(CSRField(name=F"lead", description=F"IEM LEAD Output", reset=0, values=[
            ("0", "0"),
            ("1", "1"),
            ]))

        fields.append(CSRField(name=F"df", description=F"Delay at min/max", reset=0, values=[
            ("0", "INC", "Increment delay on value pulse"),
            ("1", "DEC", "Decrement delay on value p[ulse"),
            ]))

        self._csr_stat     = CSRStatus(name="stat", fields=fields, description="DF LAG and LEAD for IO", reset=0)
        self._csr_calib    = CSRStorage(name="calib", reset=0)

        fields = []
        fields.append(CSRField(name=F"sdtap", description=F"Enable Dynamic Delay", reset=0, values=[
            ("0", "Static", "Static built in delay"),
            ("1", "Dynamic", "Dynamic delay controlled by value and setn bits."),
            ]))

        fields.append(CSRField(name=F"value", description=F"Pulse for delay inc/dec (depends on setn)", reset=0, values=[
            ("0", "0"),
            ("1", "1"),
            ]))

        fields.append(CSRField(name=F"setn", description=F"Increment or decrement delay", reset=0, values=[
            ("0", "INC", "Increment delay on value pulse"),
            ("1", "DEC", "Decrement delay on value pulse"),
            ]))
        self._csr_ctrl = CSRStorage(name="input_control", fields=fields, reset=0)

        regdata = Signal(8+2)
        self.comb += regdata.eq(self.data)
        self.comb += regdata[8].eq(self.k)
        self.comb += regdata[9].eq(self.invalid)
        # Clock domain stuff
        #n = 0 if par_clock_domain == "sys" else 2
        self.specials += [
            MultiReg(regdata, self._csr_din.status, "sys"),
            MultiReg(stat_bits, self._csr_stat.status, "sys"),
            MultiReg(self.ser_data, self._csr_raw.status, "sys"),
            MultiReg(self._csr_calib.storage, self.calib, par_clock_domain),
            MultiReg(self._csr_ctrl.storage, ctrl_bits, par_clock_domain)
        ]
        self.comb += stat_bits[0].eq(self.lag)
        self.comb += stat_bits[1].eq(self.lead)
        self.comb += stat_bits[2].eq(self.df)
        self.comb += self.sdtap.eq(ctrl_bits[0])
        self.comb += self.value.eq(ctrl_bits[1])
        self.comb += self.setn.eq(ctrl_bits[2])
        sync = getattr(self.sync, clock_domain)
