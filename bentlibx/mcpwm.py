#
# This file is part of LiteX.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg

from litex.soc.interconnect.csr import *

class MCPWM(Module, AutoCSR):
    """ Multi-Channel Pulse Width Modulation

    """
    def __init__(self, pads=None, clock_domain="sys", with_csr=True,
        num_channels     = 8,
        default_enable   = 1,
        default_polarity = 0,
        default_sync_updates = 0,
        default_width    = 512,
        default_period   = 1024):

        self.default_width = default_width
        self.default_enable = default_enable
        self.default_polarity = default_polarity
        self.default_sync_updates = default_sync_updates

        # Save ny=umber of channels
        self.n = num_channels

        # Create the PWM signals
        if pads is None:
            self.pwm = pads = Signal(self.n)
        else:
            self.n = len(pads)
            self.pwm = Signal(self.n)

        # Reset
        self.reset   = Signal()
        # Per channel enable (1 enabled 0 disabled)
        self.enable  = Signal(self.n, reset=default_enable*(2**self.n-1))
        # Per channel synchronus updates (1 update CH reg at beginning of cycle, 0 immediately update ch. on CSR write.)
        self.sync_updates  = Signal(self.n, reset=default_sync_updates*(2**self.n-1))
        # Per channel polarity (0: PWM Starts the cycle at 0 and toggles to 1 on match, 'Edge Aligned'
        # Per channel polarity (1: PWM Starts the cycle at 1 and toggles to 0 on match, 'Edge Aligned'
        self.polarity= Signal(self.n, reset=default_polarity*(2**self.n-1))
        # Timer enable
        self.enabled = Signal()
        # Timer period, timer counts from 0 to period and resets to 0, 'Edge Aligned'
        self.period  = Signal(32, reset=default_period)
        # Channel pulsewidth, See polarity for info.
        for i in range(self.n):
            setattr(self, """ch{i}""".format(i=i), Signal(32, reset=default_width))
            setattr(self, """_ch{i}""".format(i=i), Signal(32, reset=default_width))
     
        self.counter = Signal(32, reset_less=True)

        sync = getattr(self.sync, clock_domain)
        sync += [
                If(self.enabled & ~self.reset,
                    self.counter.eq(self.counter + 1),
                    If(self.counter == (self.period - 1),
                        self.counter.eq(0)
                    ),
                ).Else(
                    self.counter.eq(0)
                )
            ]
        for i in range(self.n):
            ch_shadow = getattr(self, "_ch{i}".format(i=i), Signal)
            ch = getattr(self, "ch{i}".format(i=i), Signal)
            sync += [
                    If(self.enable[i] & ~self.reset,
                        If(self.sync_updates[i] == 0,
                            ch.eq(ch_shadow),
                        ),
                        If(self.counter == getattr(self, """ch{i}""".format(i=i)),
                            self.pwm[i].eq(~self.polarity[i])
                        ).Else(
                            If(self.counter == 0,
                                self.pwm[i].eq(self.polarity[i]),
                                If(self.sync_updates[i] == 1,
                                    ch.eq(ch_shadow),
                                )
                            )
                        )
                    ).Else(
                        self.pwm[i].eq(self.polarity[i])
                    )
            ]
        
        self.comb += pads.eq(self.pwm)
        self.comb += self.enabled.eq(self.enable != 0)

        if with_csr:
            self.add_csr(clock_domain)

    def add_csr(self, clock_domain):
        fields = []
        for i in range(self.n):
            fields.append(CSRField(name="en{i}".format(i=i), description="Enable for channel {i}".format(i=i), reset=self.default_enable, values=[
                ("0", "DISABLE", "Disable channel {i}".format(i=i)),
                ("1", "ENABLE", "Enable channel {i}".format(i=i))
                ]))

        self._enable = CSRStorage(self.n, name="enable", fields=fields, description="""MCPWM Enable.\n
            Write ``1`` to enable MCPWM.""",
            reset = self.enable.reset)
        
        fields = []
        for i in range(self.n):
            fields.append(CSRField(name="pol{i}".format(i=i), description="Polarity for channel {i}".format(i=i), reset=self.default_polarity, values=[
                ("0", "POL0", "PWM Starts at 0 toggles to 1 at match".format(i=i)),
                ("1", "POL1", "PWM Starts at 1 toggles to 0 at match".format(i=i))
                ]))

        self._polarity = CSRStorage(self.n, name="polarity", fields=fields, description="""MCPWM Channel Polarity bits.""",
            reset = self.polarity.reset)
        
        fields = []
        for i in range(self.n):
            fields.append(CSRField(name="sync{i}".format(i=i), description="Enable synchronus updates on channel {i}".format(i=i), reset=self.default_polarity, values=[
                ("0", "DIS", "Channel {i} pulse width updates immediately on CSR write".format(i=i)),
                ("1", "EN", "Channel {i} pulse width updates on counter overflow.".format(i=i))
                ]))

        self._sync_updates = CSRStorage(self.n, name="sync_updates", fields=fields, description="""MCPWM Sync Update bits.""",
            reset = self.sync_updates.reset)

        self._counter = CSRStatus(32, name="counter", description="""MCPWM Counter.\n""", reset=0)

        self._period = CSRStorage(32, reset_less=True, description="""MCPWM Period.\n
            Defines the *Period* of the MCPWM in ``{cd}_clk`` cycles.""".format(cd=clock_domain),
            reset = self.period.reset)
        
        for i in range(self.n):
            setattr(self, """csr_ch{i}""".format(i=i), CSRStorage(32, name="""ch{i}""".format(i=i),  reset_less=True, description="""PWM Width.\n
            Defines the *Duty cycle* of the MCPWM. PWM is active high for *Width* ``{cd}_clk`` cycles and
            active low for *Period - Width* ``{cd}_clk`` cycles.""".format(cd=clock_domain),
            reset = getattr(self, """ch{i}""".format(i=i)).reset))


        n = 0 if clock_domain == "sys" else 2
        
        for i in range(self.n):
            self.specials += MultiReg(getattr(self, """csr_ch{i}""".format(i=i)).storage,  getattr(self, """_ch{i}""".format(i=i)),  n=n)
        
        self.specials += [
            MultiReg(self._enable.storage, self.enable, n=n),
            MultiReg(self._polarity.storage, self.polarity, n=n),
            MultiReg(self._sync_updates.storage, self.sync_updates, n=n),
            MultiReg(self._period.storage, self.period, n=n),
            MultiReg(self.counter, self._counter.status, n=n),
        ]
                    
