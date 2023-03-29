#
# Copyright 2023 Bentwire
# Based on: https://github.com/enjoy-digital/litex/blob/master/litex/soc/cores/pwm.py
# 
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg

from litex.soc.interconnect.csr import *
from litex.soc.interconnect.csr_eventmanager import *
from litex.soc.integration.doc import AutoDoc, ModuleDoc

class Counter(Module, AutoCSR, AutoDoc):

    def __init__(self, clock_domain="sys", default_enable=1, default_sync=0, default_nbits=32, default_period=0, 
                 with_csr=True, 
                 with_interrupts=True):
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
        # Counter period (counter resets to 0 at this value, or switches direction if up/down counter)
        self.period = Signal(self.nbits, reset=default_period)
        # Period shadow register for sync updates
        self._period = Signal(self.nbits, reset=default_period)
        # Counter direction
        # 0: Up
        # 1: Down
        self.dir = Signal()
        # Counter Mode register
        # 0: Up Counter
        # 1: Down Counter
        # 2: Up/Down counter
        # 3: One shot timer.
        self.mode = Signal(2, reset=0)
        modes = {
            0:  If(self.enable & ~self.reset,
                    self.counter.eq(self.counter + 1),
                    If(self.syncup == 0,
                        self.period.eq(self._period),
                    ),
                    If(self.counter == (self.period - 1),
                        self.counter.eq(0),
                        If(self.syncup == 1,
                            self.period.eq(self._period),
                        ),
                    ),
                ).Else(
                    self.period.eq(self._period),
                    self.counter.eq(0),
                ),
            1:  If(self.enable & ~self.reset,
                    self.counter.eq(self.counter - 1),
                    If(self.syncup == 0,
                        self.period.eq(self._period),
                    ),
                    If(self.counter == 0,
                        self.counter.eq(self.period),
                        If(self.syncup == 1,
                            self.period.eq(self._period),
                        ),
                    ),
                ).Else(
                    self.period.eq(self._period),
                    self.counter.eq(self.period),
                ),
            2:  If(self.enable & ~self.reset,
                    If(~self.dir,
                       self.counter.eq(self.counter + 1),
                    ).Else(
                        self.counter.eq(self.counter - 1)
                    ),
                    
                    If(self.syncup == 0,
                        self.period.eq(self._period)
                    ),

                    If(self.counter == (self.period - 1),
                        self.dir.eq(1),
                        If(self.syncup == 1,
                            self.period.eq(self._period)
                        )
                    ),
                    If(self.counter == 0,
                        self.dir.eq(0),
                        If(self.syncup == 1,
                            self.period.eq(self._period)
                        )
                    ),
                ).Else(
                    self.period.eq(self._period),
                    self.counter.eq(0),
                    self.dir.eq(0)
                ),
            3:  If(self.enable & ~self.reset, 
                    self.period.eq(self._period),
                    self.counter.eq(0)
                ).Else(
                    self.period.eq(self._period),
                    self.counter.eq(0)
                ),
            "default":  If(self.enable & ~self.reset, 
                    self.period.eq(self._period),
                    self.counter.eq(0)
                ).Else(
                    self.period.eq(self._period),
                    self.counter.eq(0)
                ),
        }

        # Update counter
        sync = getattr(self.sync, clock_domain)
        sync += Case(self.mode, modes)
        # sync += [
        #         If(self.enable & ~self.reset,
        #             self.counter.eq(self.counter + 1),
        #             If(self.syncup == 0,
        #                 self.period.eq(self._period)
        #             ),
        #             If(self.counter == (self.period - 1),
        #                 self.counter.eq(0),
        #                 If(self.syncup == 1,
        #                     self.period.eq(self._period)
        #                 )
        #             ),
        #         ).Else(
        #             self.period.eq(self._period),
        #             self.counter.eq(0)
        #         )
        #     ]
        if with_csr:
            self.add_csr(clock_domain)
        if with_interrupts:
            self.add_evt(clock_domain)

    def add_csr(self, clock_domain):
        # Control register
        fields = []
        print("ERST: {o}".format(o=self.enable.reset))
        fields.append(CSRField(name=F"ena", description=F"Enable Counter", reset=self.enable.reset, values=[
            ("0", "DIS", "Disable Counter"),
            ("1", "EN", "Enable Counter"),
            ]))

        fields.append(CSRField(name=F"sup", description=F"Enable Synchronus Period Updates", reset=self.syncup.reset, values=[
            ("0", "DIS", "Disable Syncup"),
            ("1", "EN", "Enable Syncup"),
            ]))

        fields.append(CSRField(size=2, name=F"mode", description=F"Set Timer Mode", reset=0, values=[
            ("0", "UP", "Count Up"),
            ("1", "DN", "Count Down"),
            ("2", "UPDN", "Count up/down"),
            ("3", "ONE", "One Shot mode"),
            ]))

        self._csr_control = CSRStorage(2+2, name="control", fields=fields, description="Counter Control Register",
            reset = 1) #Cat(self.enable.reset, self.syncup.reset, 0))

        # Counter register
        self._csr_counter = CSRStatus(32, name="value", description="Counter Value", reset=0)

        # Period register
        self._csr_period = CSRStorage(32, name="period", reset_less=True, description="Counter Period", reset = self.period.reset)

        # Clock domain stuff
        n = 0 if clock_domain == "sys" else 2
        control = Signal(2+2)
        self.specials += [
            MultiReg(self._csr_control.storage, control, n=n),
            MultiReg(self._csr_period.storage, self._period, n=n),
            MultiReg(self.counter, self._csr_counter.status, n=n),
        ]

        self.comb += [
            self.enable.eq(control[0]),
            self.syncup.eq(control[1]),
            self.mode.eq(Cat(control[2], control[3])),
        ]

    def add_evt(self, clock_domain):
        self.submodules.ev = EventManager()
        self.ev.ovf = EventSourcePulse(description="Counter overflow event.")
        self.ev.finalize()
        self.comb += self.ev.ovf.trigger.eq(self.counter == (self.period - 1))

class Channel(Module, AutoCSR, AutoDoc):
    def __init__(self, counter, iopin = None, clock_domain="sys", default_pol=0, default_enable=1, default_sync=0, default_nbits=32, default_mat=0, with_csr=True, with_control_reg=False, with_interrupts=True):
        # Reset
        self.reset  = Signal()
        # Edge detetct
        self.ioshift = Signal(2)
        # Enable Channel
        self.enable = Signal(reset=default_enable)
        # Channel polarity
        self.pol = Signal(reset=default_pol)

        # The pin for this channel
        if iopin is None:
            self.iopin = TSTriple()
        else:
            self.iopin = TSTriple()
            self.specials += self.iopin.get_tristate(iopin)

        # Enable Synchronus updates
        self.syncup = Signal(reset=default_sync)
        # Counter number of bits
        self.nbits = default_nbits
        # Counter from the Counter module
        self.counter = counter
        # Channel match value, also channel capture 1
        self.mat  = Signal(self.nbits, reset=default_mat)
        # Pulse width shadow register for sync updates
        self._mat = Signal(self.nbits, reset=default_mat)
        # Channel mode register
        # 0: PWM (Match mode)
        # 1: Input Capture Mode
        # 2: Output Compare Mode
        # 3: Undefined
        self.mode = Signal(2, reset=0)
        modes = {
            0: If(self.enable & ~self.reset,
                    If(self.syncup == 0,
                        self.mat.eq(self._mat)
                    ),
                    If(self.counter == self.mat,
                        self.iopin.o.eq(~self.iopin.o)
                    ),
                    If(self.counter == 0,
                        self.iopin.o.eq(self.pol),
                        If(self.syncup == 1,
                            self.mat.eq(self._mat)
                        )
                    )
                ).Else(
                    self.mat.eq(self._mat),
                    self.iopin.o.eq(self.pol),
                ),
            1: If(self.enable & ~self.reset,
                    If(Cat(self.ioshift[0], self.iopin.i) == Cat(self.pol, ~self.pol),
                       self.mat.eq(self.counter) # I'm pretty sure this is too nieve and will be off by a count
                    )  
                ),
            2: If(self.enable & ~self.reset,
                    If(self.syncup == 0,
                        self.mat.eq(self._mat)
                    ),
                    If(self.counter == self.mat,
                        self.iopin.o.eq(~self.pol)
                    ),
                    If(self.counter == 0,
                        self.iopin.o.eq(self.pol),
                        If(self.syncup == 1,
                            self.mat.eq(self._mat)
                        )
                    )
                ).Else(
                    self.mat.eq(self._mat),
                    self.iopin.o.eq(self.pol),
                ),
            3: If(self.enable & ~self.reset,
                    If(self.syncup == 0,
                        self.mat.eq(self._mat)
                    ),
                    If(self.counter == self.mat,
                        self.iopin.o.eq(~self.pol)
                    ),
                    If(self.counter == 0,
                        self.iopin.o.eq(self.pol),
                        If(self.syncup == 1,
                            self.mat.eq(self._mat)
                        )
                    )
                ).Else(
                    self.mat.eq(self._mat),
                    self.iopin.o.eq(self.pol),
                    
                ),
        }

        # Update Channel status
        sync = getattr(self.sync, clock_domain)
        sync += If(self.mode == 1, # Input capture mode
                   self.iopin.oe.eq(0),
                   self.ioshift.eq(Cat(self.ioshift[0], self.iopin.i))
                ).Else(self.iopin.oe.eq(1))
        sync += Case(self.mode, modes)

        if with_csr:
            self.add_csr(clock_domain, with_control_reg)
        if with_interrupts:
            self.add_evt(clock_domain)

    def add_csr(self, clock_domain, with_control_reg):
        # Control register
        if with_control_reg:
            fields = []
            fields.append(CSRField(name=F"ena", description=F"Enable Channel", reset=self.enable.reset, values=[
                ("0", "DIS", "Disable Channel"),
                ("1", "EN", "Enable Channel"),
                ]))

            fields.append(CSRField(name=F"sup", description=F"Enable Synchronus Pulse Width Updates", reset=self.syncup.reset, values=[
                ("0", "DIS", "Disable Syncup"),
                ("1", "EN", "Enable Syncup"),
                ]))

            fields.append(CSRField(name=F"pol", description=F"Set PWM Polaity", reset=self.pol.reset, values=[
                ("0", "L", "PWM Starts at 0 and toggles on match."),
                ("1", "H", "PWM Starts at 1 and toggles on match."),
                ]))

            self._csr_control = CSRStorage(2, name="control", fields=fields, description="Control Register",
                reset = 1) #Cat(self.enable.reset, self.syncup.reset))

        # Match register
        self._csr_mat = CSRStorage(self.nbits, name="match", description="""Channel Match Register\nWhen the counter equals this value the output will toggle.""", reset=self._mat.reset)

        # Clock domain stuff
        n = 0 if clock_domain == "sys" else 2
        if with_control_reg:
            control = Signal(2)
            self.specials += MultiReg(self._csr_control.storage, control, n=n),
            self.comb += [
                self.enable.eq(control[0]),
                self.syncup.eq(control[1])
            ]
        self.specials += MultiReg(self._csr_mat.storage, self._mat, n=n)


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
        default_mat       = 512,
        default_period   = 1024):

        if pads is None:
            self.nchannels = num_channels
            self.pads      = pads = Signal(num_channels)
        else:
            self.nchannels = len(pads)
            self.pads      = pads
        
        # Set up event managers
        self.submodules.chn_irq = EventManager()
        self.submodules.cnt_irq = EventManager()

        # Add counter events
        self.cnt_irq.of = EventSourcePulse(description="Counter Overflow")
        self.cnt_irq.uf = EventSourcePulse(description="Counter Underflow")

        # Create and wire the counter 
        self.submodules.cnt = counter = Counter(clock_domain=clock_domain, default_enable=default_enable,
                                          default_sync=default_sync,
                                          default_period=default_period,
                                          default_nbits=counter_nbits,
                                          with_interrupts=False)

        # Set up the enable, polarity and syncup register for each channel.
        # Enable bit for each channel.
        fields = []
        for i in range(self.nchannels):
            fields.append(CSRField(name=F"ena{i}", description=F"Enable Channel", reset=default_enable, values=[
                ("0", "DIS", "Disable Channel"),
                ("1", "EN", "Enable Channel"),
                ]))
        self._csr_chn_ena = CSRStorage(self.nchannels, name="chn_enable", fields=fields, description="Channel Enable Register",
            reset = default_enable*(2**self.nchannels-1)) #Replicate(default_enable, self.nchannels))
        
        # Polarity bit for each channel.
        fields = []
        for i in range(self.nchannels):
            fields.append(CSRField(name=F"pol{i}", description=F"Channel Output polarity.", reset=default_polarity, values=[
                ("0", "L", "Channel Starts at 0 and toggles on match."),
                ("1", "H", "Channel Starts at 1 and toggles on match."),
                ]))

        self._csr_chn_pol = CSRStorage(self.nchannels, name=F"chn_polarity", fields=fields, description="Channel Polarity  Register",
            reset = default_polarity*(2**self.nchannels-1)) #Replicate(default_polarity, self.nchannels))

        # Syncup bit for each channel.
        fields = []
        for i in range(self.nchannels):
            fields.append(CSRField(name=F"sup{i}", description=F"Enable Synchronus Pulse Width Updates", reset=default_sync, values=[
                ("0", "DIS", "Disable Syncronus Updates"),
                ("1", "EN", "Enable Syncronus Updates"),
                ]))

        self._csr_chn_sup = CSRStorage(self.nchannels, name="chn_syncup", fields=fields, description="Channel Synchronus Updates Register",
            reset = default_sync*(2**self.nchannels-1)) #Replicate(default_sync, self.nchannels))

        # Clock domain stuff
        n = 0 if clock_domain == "sys" else 2
        enable = Signal(self.nchannels)
        pol    = Signal(self.nchannels)
        syncup = Signal(self.nchannels)
        self.specials += MultiReg(self._csr_chn_ena.storage, enable, n=n)
        self.specials += MultiReg(self._csr_chn_pol.storage, pol, n=n)
        self.specials += MultiReg(self._csr_chn_sup.storage, syncup, n=n)

        # Create and wire each of the channels
        for i in range(self.nchannels):
            # Create a channel
            channel =  Channel(counter.counter, iopin=pads[i], clock_domain=clock_domain, with_interrupts=False,
                         default_enable=default_enable,
                         default_sync=default_sync,
                         default_mat=default_mat,
                         default_pol=default_polarity,
                         default_nbits=counter_nbits)
            # Add it to the submodules list
            setattr(self.submodules, F"ch{i}", channel)
            # Create an event source for the channel match event
            channelev = EventSourcePulse(name=F"mat{i}", description=F"Channel {i} match event.")
            # Add it to the EventManager
            setattr(self.chn_irq, F"ch{i}", channelev)
            # Wire it all together
            self.comb += channelev.trigger.eq(counter.counter == channel.mat)
            self.comb += channel.enable.eq(enable[i])
            self.comb += channel.pol.eq(pol[i])
            self.comb += channel.syncup.eq(syncup[i])

        self.chn_irq.finalize()
        self.cnt_irq.finalize()

        self.chn_irq.status.description = "Channel match event flags."
        self.chn_irq.pending.description = "Channel match event flags."
        self.chn_irq.enable.description = "Channel match event flags."

        # For now the counter and the channels all share 1 IRQ.
        shared = SharedIRQ(self.chn_irq, self.cnt_irq)
        self.submodules.ev = shared
