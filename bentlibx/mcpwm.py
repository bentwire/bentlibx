#
# This file is part of LiteX.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *
from litex.soc.integration.doc import AutoDoc, ModuleDoc

class Counter(Module, AutoCSR, AutoDoc):

    def __init__(self, clock_domain="sys", default_enable=1, default_sync=0, default_nbits=32, default_period=0, with_csr=True, with_interrupts=True):
        # Reset
        self.reset  = Signal()
        # Enable counter
        self.enable = Signal(reset=default_enable)
        # Enable Synchronus updates
        self.syncup = Signal(reset=default_sync)
        # Counter number of bits
        self.nbits = default_nbits
        # Counter
        self.counter = Signal(self.nbits)
        # Counter period (counter resets to 0 at this value, or switches direction if up/down counter"
        self.period = Signal(self.nbits, reset=default_period)
        # Period shadow register for sync updates
        self._period = Signal(self.nbits, reset=default_period)
   
        # Update counter
        sync = getattr(self.sync, clock_domain)
        sync += [
                If(self.enable & ~self.reset,
                    self.counter.eq(self.counter + 1),
                    If(self.syncup == 0,
                        self.period.eq(self._period)
                    ),
                    If(self.counter == (self.period - 1),
                        self.counter.eq(0),
                        If(self.syncup == 1,
                            self.period.eq(self._period)
                        )
                    ),
                ).Else(
                    self.period.eq(self._period),
                    self.counter.eq(0)
                )
            ]
        if with_csr:
            self.add_csr(clock_domain)
        if with_interrupts:
            self.add_evt(clock_domain)

    def add_csr(self, clock_domain):
        # Control register
        fields = []
        fields.append(CSRField(name=F"en", description=F"Enable Counter", reset=self.enable.reset, values=[
            ("0", "DIS", "Disable Counter"),
            ("1", "EN", "Enable Counter"),
            ]))

        fields.append(CSRField(name=F"sync", description=F"Enable Synchronus Period Updates", reset=self.syncup.reset, values=[
            ("0", "DIS", "Disable Syncup"),
            ("1", "EN", "Enable Syncup"),
            ]))

        self._csr_control = CSRStorage(2, name="control", fields=fields, description="Enable Counter: 1 Enable, 0 Disable",
            reset = self.enable.reset)

        # Counter register
        self._csr_counter = CSRStatus(32, name="value", description="Counter Value", reset=0)

        # Period register
        self._csr_period = CSRStorage(32, name="period", reset_less=True, description="Counter Period", reset = self.period.reset)

        # Clock domain stuff
        n = 0 if clock_domain == "sys" else 2
        control = Signal(2)
        self.specials += [
            MultiReg(self._csr_control.storage, control, n=n),
            MultiReg(self._csr_period.storage, self._period, n=n),
            MultiReg(self.counter, self._csr_counter.status, n=n),
        ]

        self.comb += [
            self.enable.eq(control[0]),
            self.syncup.eq(control[1])
        ]

    def add_evt(self, clock_domain):
        self.submodules.ev = EventManager()
        self.ev.ovf = EventSourcePulse(description="Counter overflow event.")
        self.ev.finalize()
        self.comb += self.ev.ovf.trigger.eq(self.counter == (self.period - 1))

class Channel(Module, AutoCSR, AutoDoc):
    def __init__(self, counter, iopin = None, clock_domain="sys", default_pol=0, default_enable=1, default_sync=0, default_nbits=32, default_pw=0, with_csr=True, with_interrupts=True):
        # Reset
        self.reset  = Signal()
        # Enable Channel
        self.enable = Signal(reset=default_enable)
        # Channel polarity
        self.pol = Signal(reset=default_pol)
        # Enable Synchronus updates
        self.syncup = Signal(reset=default_sync)
        # Counter number of bits
        self.nbits = default_nbits
        # Counter from the Counter module
        self.counter = counter
        # Channel pulse width, the output is asserted this many clk cycles.
        self.pw  = Signal(self.nbits, reset=default_pw)
        # Pulse width shadow register for sync updates
        self._pw = Signal(self.nbits, reset=default_pol)
  
        # The pin for this channel
        if iopin is None:
            self.iopin = iopin = Signal()
        else:
            self.iopin = iopin

        # Update Channel status
        sync = getattr(self.sync, clock_domain)
        sync += [
                If(self.enable & ~self.reset,
                    If(self.syncup == 0,
                        self.pw.eq(self._pw)
                    ),
                    If(self.counter == self.pw,
                        iopin.eq(~self.pol)
                    ),
                    If(self.counter == 0,
                        iopin.eq(self.pol),
                        If(self.syncup == 1,
                            self.pw.eq(self._pw)
                        )
                    )
                ).Else(
                    self.pw.eq(self._pw),
                    iopin.eq(self.pol),
                )
            ]

        if with_csr:
            self.add_csr(clock_domain)
        if with_interrupts:
            self.add_evt(clock_domain)

    def add_csr(self, clock_domain):
        # Control register
        fields = []
        fields.append(CSRField(name=F"en", description=F"Enable Channel", reset=self.enable.reset, values=[
            ("0", "DIS", "Disable Channel"),
            ("1", "EN", "Enable Channel"),
            ]))

        fields.append(CSRField(name=F"sync", description=F"Enable Synchronus Pulse Width Updates", reset=self.syncup.reset, values=[
            ("0", "DIS", "Disable Syncup"),
            ("1", "EN", "Enable Syncup"),
            ]))

        self._csr_control = CSRStorage(2, name="control", fields=fields, description="Control Register",
            reset = Cat(self.enable.reset, self.syncup.reset))

        # Pulse Width Register
        self._csr_pw = CSRStorage(self.nbits, name="pw", description="Pulse Width", reset=self._pw.reset)

        # Clock domain stuff
        n = 0 if clock_domain == "sys" else 2
        control = Signal(2)
        self.specials += [
            MultiReg(self._csr_control.storage, control, n=n),
            MultiReg(self._csr_pw.storage, self._pw, n=n),
        ]

        self.comb += [
            self.enable.eq(control[0]),
            self.syncup.eq(control[1])
        ]

    def add_evt(self, clock_domain):
        self.submodules.ev = EventManager()
        self.ev.chmatch = EventSourcePulse(description="Channel match event.")
        self.ev.finalize()
        self.comb += self.ev.chmatch.trigger.eq(self.counter == self.pw)

class AdvancedTimerCounter(Module, AutoCSR, AutoDoc):
    def __init__(self, pads=None, clock_domain="sys", counter_nbits=32, with_csr=True, num_channels=8, 
        default_enable   = 1,
        default_polarity = 0,
        default_sync     = 0,
        default_pw       = 512,
        default_period   = 1024):

        if pads is None:
            self.nchannels = num_channels
            self.pads      = pads = Signal(num_channels)
        else:
            self.nchannels = len(pads)
            self.pads      = pads

        self.submodules.ev = EventManager()
        self.ev.placeholder = EventSourceProcess(edge="rising", description="WHy do I need this?")
        self.submodules.counter = counter = Counter(default_enable=default_enable,
                                          default_sync=default_sync,
                                          default_period=default_period,
                                          default_nbits=counter_nbits)
        for i in range(self.nchannels):
            setattr(self.submodules, F"ch{i}", Channel(counter.counter, iopin=pads[i], with_interrupts=False,
                                                                                default_enable=default_enable,
                                                                                default_sync=default_sync,
                                                                                default_pw=default_pw,
                                                                                default_pol=default_polarity,
                                                                                default_nbits=counter_nbits))

        self.ev.finalize()

class MCPWM(Module, AutoCSR, AutoDoc):
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

        # Set up EvemtManager for interrupts to SoC core.
        self.submodules.ev = EventManager()
        self.submodules.evch = EventManager()
        # Counter overflow
        self.ev.ovf = EventSourceProcess(edge="rising", description="Counter overflow event.")
        # Channel match
#        for i in range(self.n):
#            setattr(self.ev[1],F"ch{i}", EventSourceProcess(name=F"ch{i}", edge="rising", description=F"Channel {i} match event."))
#
#        self.ev[0].finalize()
        self.ev.finalize()

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
        self.comb += self.ev.ovf.trigger.eq(self.counter == 0)

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
                    
