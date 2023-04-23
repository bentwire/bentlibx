#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2022 Icenowy Zheng <icenowy@aosc.io>
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.cores.clock.gowin_gw2a import GW2APLL
from litex.soc.integration.soc_core import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import *
#from litex.soc.cores.pwm import *
from bentlibx.mcpwm import *
from bentlibx.ddr import *
from litex.soc.cores.led import LedChaser, WS2812
from litex.soc.cores.gpio import GPIOIn
from litex.soc.cores.video import *

from liteeth.phy.rmii import LiteEthPHYRMII

from bentlibx.platforms import sipeed_tang_primer_20k

from litedram.common import PHYPadsReducer
from litedram.modules import MT41J128M16, AS4C128M16
from litedram.phy import GW2DDRPHY

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, with_video_pll=False):
        self.rst        = Signal()
        self.cd_sys     = ClockDomain()
        self.cd_por     = ClockDomain()
        self.cd_init    = ClockDomain()
        self.cd_sys2x   = ClockDomain()
        self.cd_sys2x_i = ClockDomain()
        self.cd_sys4x   = ClockDomain()
        self.cd_sys4x_i = ClockDomain()
        self.cd_sys8x   = ClockDomain()
        self.cd_sys8x_5 = ClockDomain()

        self.cd_par_clk    = ClockDomain()
        self.cd_par_clk5x  = ClockDomain()
        self.cd_par_clk5xp = ClockDomain()
        self.cd_recovery   = ClockDomain()
        

        # # #

        self.stop      = Signal()
        self.reset     = Signal()

        # Clock recovery stuff        
        self.duty       = Signal(4, reset=8)
        self.phase      = Signal(4, reset=0)
        self.fdly       = Signal(4, reset=0xf)
        self.rec_locked = Signal()

        # Clk
        clk27 = platform.request("clk27")

        # Power on reset (the onboard POR is not aware of reprogramming)
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(clk27)
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # PLL
        self.pll = pll = GW2APLL(devicename=platform.devicename, device=platform.device)
        self.comb += pll.reset.eq(~por_done)
        pll.register_clkin(clk27, 27e6)
        #pll.create_clkout(self.cd_sys2x_i, 2*sys_clk_freq)
        pll.create_clkout(self.cd_sys8x, 8*sys_clk_freq, margin=0.05)
        pll.create_clkout(self.cd_sys4x, 4*sys_clk_freq)
        #pll.create_clkout(self.cd_sys2x_i, 2*sys_clk_freq)

        
        # self.specials += Instance("rPLL",
        #                           p_FCLKIN    = "400", # MHz
        #                           p_IDIV_SEL  = 62, # Dev input clk by 2
        #                           p_FBDIV_SEL = 62, # div by 2
        #                           p_ODIV_SEL  = 2, # div by 2  this plus FBDIV makes the VCO run at 2x FIN
        #                           p_DYN_DA_EN = "false", # Enable dynamic duty and phase adjustment
        #                           p_CLKFB_SEL = "internal",
        #                           i_CLKIN = self.cd_sys8x.clk,
        #                           i_CLKFB = Signal(), #self.cd_recovery.clk, # Ignore when set to internal
        #                           i_RESET = self.reset,
        #                           i_RESET_P = Signal(),
        #                           i_FBDSEL = Signal(6), # Should be ignored per config
        #                           i_IDSEL = Signal(6),  # Should be ignored per config
        #                           i_ODSEL = Signal(6),  # Should be ignored per config
        #                           i_DUTYDA = self.duty,
        #                           i_PSDA = self.phase,
        #                           i_FDLY = self.fdly,
        #                           o_CLKOUT = self.cd_par_clk5x.clk,
        #                           o_CLKOUTP = self.cd_par_clk5xp.clk,
        #                           o_CLKOUTD = Signal(),
        #                           o_CLKOUTD3 = Signal(),
        #                           o_LOCK = self.rec_locked)

        # self.specials += Instance("CLKDIV",
        #                             p_DIV_MODE = "5",
        #                             i_CALIB    = 0,
        #                             i_HCLKIN   = self.cd_par_clk5x.clk,
        #                             i_RESETN   = 1,
        #                             o_CLKOUT   = self.cd_par_clk.clk),

        self.specials += [
            Instance("DHCEN",
                i_CLKIN  = self.cd_sys2x_i.clk,
                i_CE     = self.stop,
                o_CLKOUT = self.cd_sys2x.clk),
            Instance("CLKDIV",
                p_DIV_MODE = "2",
                i_CALIB    = 0,
                i_HCLKIN   = self.cd_sys4x.clk,
                i_RESETN   = ~self.reset,
                o_CLKOUT   = self.cd_sys2x_i.clk),
            Instance("CLKDIV",
                p_DIV_MODE = "2",
                i_CALIB    = 0,
                i_HCLKIN   = self.cd_sys2x_i.clk,
                i_RESETN   = ~self.reset,
                o_CLKOUT   = self.cd_sys.clk),
            Instance("CLKDIV",
                p_DIV_MODE = "5",
                i_CALIB    = 0,
                i_HCLKIN   = self.cd_sys8x.clk,
                i_RESETN   = ~self.reset,
                o_CLKOUT   = self.cd_sys8x_5.clk),
            AsyncResetSynchronizer(self.cd_sys, ~pll.locked | self.reset),
        ]
        # self.specials += Instance("CLKDIV",
        #                             p_DIV_MODE = "5",
        #                             i_CALIB    = 0,
        #                             i_HCLKIN   = self.cd_sys8x,
        #                             i_RESETN   = ~self.reset,
        #                             o_CLKOUT   = self.cd_sys8x_5)

        # Init clock domain
        self.comb += self.cd_init.clk.eq(clk27)
        self.comb += self.cd_init.rst.eq(pll.reset)

        # Video PLL
        if with_video_pll:
            self.video_pll = video_pll = GW2APLL(devicename=platform.devicename, device=platform.device)
            video_pll.register_clkin(clk27, 27e6)
            self.cd_hdmi   = ClockDomain()
            self.cd_hdmi5x = ClockDomain()
            video_pll.create_clkout(self.cd_hdmi5x, 125e6, margin=3e-3)
            #video_pll.create_clkout(self.cd_hdmi5x, 371.25e6, margin=5e-3)
            self.specials += Instance("CLKDIV",
                p_DIV_MODE = "5",
                i_RESETN   = 1, # Disable reset signal.
                i_CALIB    = 0, # No calibration.
                i_HCLKIN   = self.cd_hdmi5x.clk,
                o_CLKOUT   = self.cd_hdmi.clk
            )

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(SoCCore):
    def __init__(self, sys_clk_freq=48e6,
        with_spi_flash      = False,
        with_led_chaser     = True,
        with_rgb_led        = False,
        with_buttons        = True,
        with_video_terminal = False,
        with_ethernet       = False,
        with_etherbone      = False,
        eth_ip              = "192.168.2.2",
        eth_dynamic_ip      = False,
        dock                = "standard",
        **kwargs):

        assert dock in ["standard", "lite"]

        platform = sipeed_tang_primer_20k.Platform(dock, toolchain="gowin")

        if dock == "lite":
            with_led_chaser = False # No leds on core board nor on dock lite.

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, with_video_pll=with_video_terminal)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteX SoC on Tang Primer 20K", **kwargs)

        # DDR3 SDRAM -------------------------------------------------------------------------------
        # FIXME: WIP / Untested.
        if not self.integrated_main_ram_size:
            self.ddrphy = GW2DDRPHY(
                pads         = PHYPadsReducer(platform.request("ddram"), [0, 1]),
                sys_clk_freq = sys_clk_freq
            )
            self.ddrphy.settings.rtt_nom = "disabled"
            #self.ddrphy.settings.add_electrical_settings(rtt_nom=40)
            self.comb += self.crg.stop.eq(self.ddrphy.init.stop)
            self.comb += self.crg.reset.eq(self.ddrphy.init.reset)
            self.add_sdram("sdram",
                phy           = self.ddrphy,
                module        = MT41J128M16(sys_clk_freq, "1:2"),
                #module        = AS4C128M16(sys_clk_freq, "1:2"),
                l2_cache_size = 0
            )
            # ./sipeed_tang_primer_20k.py --cpu-variant=lite --uart-name=crossover+uartbone --csr-csv=csr.csv --build --load
            # litex_server --uart --uart-port=/dev/ttyUSB2
            # litex_term crossover
            # litescope_cli
            if kwargs["uart_name"] == "crossover+uartbone":
                from litescope import LiteScopeAnalyzer
                analyzer_signals = [
                    self.ddrphy.dfi.p0,
                    self.ddrphy.dfi.p0.wrdata_en,
                    self.ddrphy.dfi.p1.rddata_en,
                ]
                self.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 128,
                clock_domain = "sys",
                samplerate   = sys_clk_freq,
                csr_csv      = "analyzer.csv"
                )

        # SPI Flash --------------------------------------------------------------------------------
        if with_spi_flash:
            from litespi.modules import W25Q32JV as SpiFlashModule
            from litespi.opcodes import SpiNorFlashOpCodes as Codes
            #self.add_spi_flash(mode="1x", module=SpiFlashModule((Codes.READ_1_1_1, Codes.PP_1_1_1, Codes.WREN, Codes.SE, Codes.CHIP_ERASE)))
            self.add_spi_flash(mode="1x", module=SpiFlashModule(Codes.READ_1_1_1, erase_cmd=Codes.CHIP_ERASE, program_cmd=Codes.PP_1_1_1))

        # Ethernet / Etherbone ---------------------------------------------------------------------
        if with_ethernet or with_etherbone:
            from liteeth.phy.rmii import LiteEthPHYRMII
            self.ethphy = LiteEthPHYRMII(
                clock_pads = self.platform.request("eth_clocks"),
                pads       = self.platform.request("eth"),
                refclk_cd  = None
            )
            if with_ethernet:
                self.add_ethernet(phy=self.ethphy, dynamic_ip=eth_dynamic_ip, with_timing_constraints=False)
            if with_etherbone:
                self.add_etherbone(phy=self.ethphy, ip_address=eth_ip, with_timing_constraints=False)

        # Video ------------------------------------------------------------------------------------
        if with_video_terminal:
            hdmi_pads = platform.request("hdmi")
            self.comb += hdmi_pads.hdp.eq(1)
            self.videophy = VideoHDMIPHY(hdmi_pads, clock_domain="hdmi")
            #self.add_video_colorbars(phy=self.videophy, timings="640x480@60Hz", clock_domain="hdmi")
            self.add_video_terminal(phy=self.videophy, timings="640x480@75Hz", clock_domain="hdmi")
            #self.add_video_terminal(phy=self.videophy, timings="1280x720@60Hz", clock_domain="hdmi")

        # Leds -------------------------------------------------------------------------------------
#        if with_led_chaser:
#            led_pads = platform.request_all("led")
#            pwm = Signal(32)  
#            self.pwm0 = MCPWM(pads=pwm)
#            self.comb += led_pads.eq(~pwm)
        #pwm = Signal(8)
        #pmod0_pads = platform.request("pmod", 0)
        #pmod1_pads = platform.request("pmod", 1)
        pmod1d_tx0 = platform.request("pmod1d", 0)
        pmod1d_rx0 = platform.request("pmod1d", 1)
        pmod1d_tx1 = platform.request("pmod1d", 2)
        #pmod1d_rx1 = platform.request("pmod1d", 3)
        led_pads = platform.request_all("led")

        #self.advpwm0 = AdvancedTimerCounter(pads=pmod0_pads, clock_domain="sys2x_i")
        self.ddr_o0 = DDR8_10O(pin=pmod1d_tx0.p, pinb=pmod1d_tx0.n)

        self.ddr_i0 = DDR10_8I(pin=pmod1d_rx0.p, pinb=pmod1d_rx0.n)

        self.ddr_io1 = DDR_8IO(pina=pmod1d_tx1.p, pinb=pmod1d_tx1.n)
        
        # self.ddr_o0 = DDR8_10O(pin=pmod1_pads[0])

        # self.ddr_i0 = DDR10_8I(pin=pmod1_pads[1])

        # self.ddr_io1 = DDR_8IO(pina=pmod1_pads[2])
        # Send K.28.1 Comma
        self.ddr_o0.k.eq(1)
        self.ddr_o0.data.eq(0x3c)
        self.comb += led_pads.eq(Cat(~self.ddr_i0.invalid, ~self.ddr_i0.k, ~self.ddr_i0.lag, ~self.ddr_i0.lead, ~self.ddr_i0.df, ~self.ddr_i0.value))
        # self.comb += led_pads[0].eq(~self.ddr_i0.invalid)
        # self.comb += led_pads[1].eq(~self.ddr_i0.k)
        # self.comb += led_pads[2].eq(~self.ddr_i0.lag)
        # self.comb += led_pads[3].eq(~self.ddr_i0.lead)
        # self.comb += led_pads[4].eq(1)
        # self.comb += led_pads[5].eq(1)

        # if self.irq.enabled:
        #    self.irq.add("advpwm0", use_loc_if_exists=True)


#            self.leds = LedChaser(
#                pads         = platform.request_all("led"),
#                sys_clk_freq = sys_clk_freq
#            )
#            self.leds.add_pwm()

        # RGB Led ----------------------------------------------------------------------------------
        if with_rgb_led:
            self.rgb_led = WS2812(
                pad          = platform.request("rgb_led"),
                nleds        = 1,
                sys_clk_freq = sys_clk_freq
            )
            self.bus.add_slave(name="rgb_led", slave=self.rgb_led.bus, region=SoCRegion(
                origin = 0x2000_0000,
                size   = 4,
            ))

        # Buttons ----------------------------------------------------------------------------------
        # Reset needs these:
        self.buttons = platform.request_all("btn_n")
        self.reset = Signal()
        self.comb += self.reset.eq(~self.buttons[0])
        #self.comb += ResetSignal("sys").eq(self.reset)
        self.comb += ResetSignal("por").eq(self.reset)
        self.comb += ResetSignal("sys2x").eq(self.reset)
        self.comb += ResetSignal("sys4x").eq(self.reset)
        if with_buttons:
            self.buttons = GPIOIn(pads=~self.buttons)


# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    from litex.soc.doc import generate_svd
    parser = LiteXArgumentParser(platform=sipeed_tang_primer_20k.Platform, description="LiteX SoC on Tang Primer 20K.")
    parser.add_target_argument("--dock",         default="standard",       help="Dock version (standard (default) or lite.")
    parser.add_target_argument("--flash",        action="store_true",      help="Flash Bitstream.")
    parser.add_target_argument("--sys-clk-freq", default=48e6, type=float, help="System clock frequency.")
    sdopts = parser.target_group.add_mutually_exclusive_group()
    sdopts.add_argument("--with-spi-sdcard",            action="store_true", help="Enable SPI-mode SDCard support.")
    sdopts.add_argument("--with-sdcard",                action="store_true", help="Enable SDCard support.")
    parser.add_target_argument("--with-spi-flash",      action="store_true", help="Enable SPI Flash (MMAPed).")
    parser.add_target_argument("--with-video-terminal", action="store_true", help="Enable Video Terminal (HDMI).")
    ethopts = parser.target_group.add_mutually_exclusive_group()
    ethopts.add_argument("--with-ethernet",         action="store_true",    help="Add Ethernet.")
    ethopts.add_argument("--with-etherbone",        action="store_true",    help="Add EtherBone.")
    parser.add_target_argument("--eth-ip",          default="192.168.1.50", help="Etherbone IP address.")
    parser.add_target_argument("--eth-dynamic-ip",  action="store_true",    help="Enable dynamic Ethernet IP addresses setting.")
    args = parser.parse_args()

    soc = BaseSoC(
        sys_clk_freq        = args.sys_clk_freq,
        with_spi_flash      = args.with_spi_flash,
        with_video_terminal = args.with_video_terminal,
        with_ethernet       = args.with_ethernet,
        with_etherbone      = args.with_etherbone,
        eth_ip              = args.eth_ip,
        eth_dynamic_ip      = args.eth_dynamic_ip,
        dock                = args.dock,
        **parser.soc_argdict
    )
    if args.with_spi_sdcard:
        soc.add_spi_sdcard()
    if args.with_sdcard:
        soc.add_sdcard()

    builder = Builder(soc, **parser.builder_argdict)
    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, builder.get_bitstream_filename(mode="flash", ext=".fs"), external=True)
    
    svd_dir = os.path.join(builder.output_dir, "svd")
    if not os.path.exists(svd_dir):
        os.mkdir(svd_dir)
    generate_svd(soc, svd_dir)

if __name__ == "__main__":
    main()
