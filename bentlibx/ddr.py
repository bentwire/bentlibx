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

class DDR_IO(Module, AutoCSR, AutoDoc):

    def __init__(self, pin, ddr_data_in = None, ddr_data_out = None, 
                 clock_domain="sys", ddr_clock_domain="sys4x_i"):
        self.reset = Signal()
        self.calib = Signal()
        self.clock_pol = Signal()
        ddr_o   = Signal()
        ddr_i   = Signal()
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
                                  i_D = ddr_i,
                                  o_Q0 = ddr_data_i[0],
                                  o_Q1 = ddr_data_i[1],
                                  o_Q2 = ddr_data_i[2],
                                  o_Q3 = ddr_data_i[3],
                                  o_Q4 = ddr_data_i[4],
                                  o_Q5 = ddr_data_i[5],
                                  o_Q6 = ddr_data_i[6],
                                  o_Q7 = ddr_data_i[7],
                                  )
        
        self.specials += Instance("IOBUF",
                                  i_I = ddr_o,
                                  i_OEN = ddr_oen,
                                  o_O = ddr_i,
                                  io_IO = pin)
        
        self._csr_din      = CSRStatus(8, name="in", description="Input Value", reset=0)
        self._csr_dout     = CSRStorage(8, name="out", description="Output Value", reset=0)
        self._csr_dout_oen = CSRStorage(4, name="oen", description="Output Enable Value", reset=0)
        self._csr_calib    = CSRStorage(name="calib", reset=0)

        # Clock domain stuff
        n = 0 if clock_domain == "sys" else 2
        self.specials += [
            MultiReg(self._csr_dout.storage, ddr_data_o, n=n),
            MultiReg(self._csr_dout_oen.storage, ddr_data_oen, n=n),
            MultiReg(ddr_data_i, self._csr_din.status, n=n),
            MultiReg(self._csr_calib.storage, self.calib, n=n)
        ]
        
        sync = getattr(self.sync, clock_domain)
        # sync += If(~self.reset,
        #            _ddr_data_i.eq(ddr_data_i),
        #            ddr_data_o.eq(_ddr_data_o),
        #         ).Else(
        #             _ddr_data_i.eq(0),
        #             ddr_data_o.eq(ddr_data_i),
        #         )
