# MicroPython SH1106 OLED driver (I2C).
# 128x64 panels with 132-column GDDRAM (typical 1.3"). Also for CH1115/CH1116
# clones and marketplace listings sold as "SSH1106".

from micropython import const
import framebuf

SET_CONTRAST = const(0x81)
SET_ENTIRE_ON = const(0xA4)
SET_NORM_INV = const(0xA6)
SET_DISP = const(0xAE)
SET_DISP_START_LINE = const(0x40)
SET_SEG_REMAP = const(0xA0)
SET_MUX_RATIO = const(0xA8)
SET_COM_OUT_DIR = const(0xC0)
SET_DISP_OFFSET = const(0xD3)
SET_COM_PIN_CFG = const(0xDA)
SET_DISP_CLK_DIV = const(0xD5)
SET_PRECHARGE = const(0xD9)
SET_VCOM_DESEL = const(0xDB)
SET_CHARGE_PUMP = const(0x8D)
# Visible 128 columns are centred in 132-wide RAM.
_COL_OFFSET = const(2)


class SH1106(framebuf.FrameBuffer):
    def __init__(self, width, height, external_vcc=False, rotate180=False):
        self.width = width
        self.height = height
        self.external_vcc = external_vcc
        self.rotate180 = bool(rotate180)
        self.pages = self.height // 8
        self.buffer = bytearray(self.pages * self.width)
        super().__init__(self.buffer, self.width, self.height, framebuf.MONO_VLSB)
        self.init_display()

    def init_display(self):
        if self.rotate180:
            seg = SET_SEG_REMAP
            com = SET_COM_OUT_DIR
        else:
            seg = SET_SEG_REMAP | 0x01
            com = SET_COM_OUT_DIR | 0x08
        for cmd in (
            SET_DISP,
            SET_DISP_START_LINE,
            seg,
            SET_MUX_RATIO,
            self.height - 1,
            com,
            SET_DISP_OFFSET,
            0x00,
            SET_COM_PIN_CFG,
            0x12,
            SET_DISP_CLK_DIV,
            0x80,
            SET_PRECHARGE,
            0x22 if self.external_vcc else 0xF1,
            SET_VCOM_DESEL,
            0x35,
            SET_CONTRAST,
            0xFF,
            SET_ENTIRE_ON,
            SET_NORM_INV,
            SET_CHARGE_PUMP,
            0x10 if self.external_vcc else 0x14,
            SET_DISP | 0x01,
        ):
            self.write_cmd(cmd)
        self.fill(0)
        self.show()

    def poweroff(self):
        self.write_cmd(SET_DISP)

    def poweron(self):
        self.write_cmd(SET_DISP | 0x01)

    def contrast(self, contrast):
        self.write_cmd(SET_CONTRAST)
        self.write_cmd(contrast)

    def invert(self, invert):
        self.write_cmd(SET_NORM_INV | (invert & 1))

    def show(self):
        # Page addressing: reset column pointer each page (SH1106 quirk).
        for page in range(self.pages):
            self.show_page(page)

    def show_page(self, page):
        """Transfer one 8-row page (~3 ms @ 400 kHz) instead of the full frame."""
        page = int(page) % self.pages
        self.write_cmd(0xB0 | page)
        self.write_cmd(0x00 | (_COL_OFFSET & 0x0F))
        self.write_cmd(0x10 | (_COL_OFFSET >> 4))
        start = self.width * page
        self.write_data(self.buffer[start : start + self.width])


class SH1106_I2C(SH1106):
    def __init__(
        self, width, height, i2c, addr=0x3C, external_vcc=False, rotate180=False
    ):
        self.i2c = i2c
        self.addr = addr
        self.temp = bytearray(2)
        self.write_list = [b"\x40", None]
        super().__init__(width, height, external_vcc, rotate180=rotate180)

    def write_cmd(self, cmd):
        self.temp[0] = 0x80
        self.temp[1] = cmd
        self.i2c.writeto(self.addr, self.temp)

    def write_data(self, buf):
        self.write_list[1] = buf
        self.i2c.writevto(self.addr, self.write_list)
