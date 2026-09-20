# Copyright 2026 OpenTCA contributors
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# SPDX-License-Identifier: GPL-3.0-or-later
"""CHIRP clone-mode driver for the TCA-152A (a.k.a. TCA PRC-152A) handheld.

TARGET VARIANT: 2023+ generation, GPS model, DOM 2025 (MCU TA3782F /
SinOne SC95F761x). TCA-152 models differ internally — pre-2023 units are quite
different, and there are separate GPS / non-GPS 2023+ variants. This codeplug
format is not assumed valid for other variants without re-verification.

Memory layout reverse-engineered in the OpenTCA project; see MEMORY_MAP.md.
Fields marked EXPERIMENTAL/UNVERIFIED below still need on-hardware confirmation.
Uploading (write to radio) is EXPERIMENTAL — see the warning on sync_out().
"""

import logging
import struct

from chirp import chirp_common, directory, memmap, bitwise, errors, util
from chirp.settings import (
    RadioSetting, RadioSettingGroup, RadioSettings,
    RadioSettingValueList, RadioSettingValueBoolean,
    RadioSettingValueInteger, RadioSettingValueString,
)

LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Memory layout (see MEMORY_MAP.md). File offset == radio config address.
# ---------------------------------------------------------------------------
MEM_FORMAT = """
#seekto 0x0010;
struct {
  lbcd rxfreq[4];      // BCD LE, 10 Hz units  (freq_MHz = value * 10 Hz)
  lbcd txfreq[4];
  u8   rxtone[2];      // BCD LE 0.1 Hz CTCSS, or DCS with flag nibble
  u8   txtone[2];
  u8   unknown0;       // 0x00 in samples
  u8   flags_a;        // bit2 busy-lock, bit6 SPEC, bit7 special QT/DQT
  u8   flags_b;        // bit3 narrow band (clear = wide)
  u8   power;          // 0=low 1=mid 2=high
} memory[128];

#seekto 0x0CA0;
struct {
  u8   ca0;            // bit0 Priority TX, bit7 GPS ON
  u8   ca1;            // bit1 Dyn Mic, bit2 Beep, bit3 Batt Save, bit5 Earphone, bit7 ScanMode(hi)
  u8   ca2;            // bit0 Work Mode, bit2 ChanName, bit3 FM ForbidRX, bit6 Light(hi), bit7 FM WorkMode
  u8   ca3;            // bit3 TX-LED, bit4 RX-LED, bit5 Auto Lock, bit6 InfoType(hi)
  u8   ca4;
  u8   ca5;
  u8   fm_channel;     // 0xCA6: FM tuner current channel 1..25
  u8   ca7;            // low nibble VOX Gain, bit5 Roger, bit6 RX GPS, bit7 TX GPS
  u8   step;           // 0xCA8 high nibble: 0=2.5k .. 6=50k, 7=100k
  u8   squelch;        // 0xCA9: 0..9
  u8   timeout;        // 0xCAA: index 0..9
  u8   rpt_flags;      // 0xCAB: bit0 Relay, bit1 RPT-SPK, bit2 RPT-PTT, bit3 RPT-MD
  u8   rpt_rl;         // 0xCAC: 0..9
  u8   cad;
  u8   vox_delay;      // 0xCAE: 0..4
  u8   caf;
  u8   channel_group;  // 0xCB0: 0-based (value = group - 1)
} settings;

#seekto 0x0CD0;
struct {
  lbcd freq[4];        // BCD LE, 0.1 MHz units (freq_MHz = value / 10)
} fm_memory[25];

#seekto 0x0D40;
struct {
  char name[8];        // ASCII, padded 0xFF / space / null
} names[128];

#seekto 0x1150;
struct {
  ul32 code;           // per-channel "CODE" (endianness UNVERIFIED)
} codes[128];

#seekto 0x13F0;
u8 scanadd[16];        // 128-bit scan-list bitmask; bit set = in scan list
"""

POWER_LEVELS = [
    chirp_common.PowerLevel("Low", watts=1.00),
    chirp_common.PowerLevel("Mid", watts=4.00),
    chirp_common.PowerLevel("High", watts=8.00),
]

# Flag masks within the channel record
_BUSYLOCK = 0x04    # flags_a bit2
_SPEC = 0x40        # flags_a bit6
_SPECIAL_QT = 0x80  # flags_a bit7
_NARROW = 0x08      # flags_b bit3

VOX_LIST = ["OFF", "1", "2", "3", "4", "5"]
STEP_LIST = ["2.50K", "5.00K", "6.25K", "10.00K", "12.50K",
             "25.00K", "50.00K", "100.00K"]
FM_WORKMODE_LIST = ["CH", "VFO"]
TIMEOUT_LIST = ["OFF", "30S", "60S", "90S", "120S", "150S",
                "180S", "210S", "240S", "270S"]
WORKMODE_LIST = ["Channel", "VFO"]   # 0xCA2 bit0 (polarity assumed)
PRIORITYTX_LIST = ["EDIT", "BUSY"]   # 0xCA0 bit0 (polarity assumed)
RPTMD_LIST = ["1", "2"]              # 0xCAB bit3
SCANMODE_LIST = ["TO", "CO", "SE"]   # 0xCA1 bits6-7 (label order assumed)

# Clone protocol (from OpenTCA programmer/com.py + config_read capture)
_MAGIC = b"\x50\x53\x4f\x4a\x55\x49\x4d"
_ACK = b"\x06"
_BLOCK = 0x10           # bytes per read/write block
_MEMSIZE = 0x1DB0       # full dump size seen from the radio


def _bcd2int(byte):
    return (byte >> 4) * 10 + (byte & 0x0F)


def _int2bcd(val):
    return ((val // 10) << 4) | (val % 10)


def _decode_tone(raw):
    """raw: 2 bytes little-endian BCD. Returns a chirp tone tuple pieces."""
    b0, b1 = raw[0], raw[1]
    if (b0 == 0xFF and b1 == 0xFF) or (b0 == 0 and b1 == 0):
        return None
    val = _bcd2int(b1) * 100 + _bcd2int(b0)
    if val >= 8000:
        # DCS: low 3 digits = code, leading digit = polarity flag.
        # Only "normal" (flag 8) confirmed; inverted assumed 9. UNVERIFIED.
        code = val % 1000
        pol = "R" if (val // 1000) >= 9 else "N"
        return ("DTCS", code, pol)
    return ("Tone", val / 10.0, None)


def _encode_tone(kind, value, pol):
    if kind is None:
        return b"\xff\xff"
    if kind == "Tone":
        val = int(round(value * 10))
    else:  # DTCS
        flag = 9 if pol == "R" else 8
        val = flag * 1000 + int(value)
    return bytes([_int2bcd(val % 100), _int2bcd(val // 100)])


@directory.register
class TCA152ARadio(chirp_common.CloneModeRadio):
    """TCA-152A / PRC-152A."""
    VENDOR = "TCA"
    MODEL = "152A"
    VARIANT = "2023+ GPS"
    BAUD_RATE = 19200
    NEEDS_COMPAT_SERIAL = False

    _memsize = _MEMSIZE

    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.experimental = (
            "This driver is EXPERIMENTAL, built from a reverse-engineered "
            "memory map. Uploading to the radio is UNVERIFIED — back up your "
            "config with the official software first, and verify writes at "
            "your own risk."
        )
        rp.pre_download = "Turn the radio on and connect the programming cable."
        rp.pre_upload = rp.pre_download
        return rp

    def get_features(self):
        rf = chirp_common.RadioFeatures()
        rf.has_settings = True
        rf.has_bank = False
        rf.has_ctone = True
        rf.has_cross = True
        rf.has_rx_dtcs = True
        rf.has_tuning_step = False
        rf.has_name = True
        rf.valid_name_length = 8
        rf.valid_characters = chirp_common.CHARSET_ASCII
        rf.valid_modes = ["FM", "NFM"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_cross_modes = ["Tone->Tone", "Tone->DTCS", "DTCS->Tone",
                                "DTCS->", "->Tone", "->DTCS", "DTCS->DTCS"]
        rf.valid_power_levels = POWER_LEVELS
        rf.valid_duplexes = ["", "-", "+", "off"]
        rf.valid_bands = [(136000000, 174000000),
                          (400000000, 520000000)]
        rf.valid_skips = ["", "S"]
        rf.memory_bounds = (1, 128)
        return rf

    # -- clone protocol -----------------------------------------------------

    def _start_session(self):
        self.pipe.write(_MAGIC)
        if self.pipe.read(1) != _ACK:
            raise errors.RadioError("Radio did not ACK the handshake.")
        self.pipe.write(b"\x02")
        ident = self.pipe.read(8)
        LOG.debug("Radio ident: %s", util.hexprint(ident))
        self.pipe.write(_ACK)
        if self.pipe.read(1) != _ACK:
            raise errors.RadioError("Radio did not ACK ident.")
        return ident

    def sync_in(self):
        try:
            self._start_session()
            data = b""
            addr = 0x0000
            while addr < self._memsize:
                self.pipe.write(struct.pack(">cHc", b"R", addr,
                                            bytes([_BLOCK])))
                resp = self.pipe.read(4 + _BLOCK + 1)
                # 0x57 <addr hi lo> <block> <data...> <term>
                if len(resp) < 4 + _BLOCK:
                    raise errors.RadioError(
                        "Short read at 0x%04X" % addr)
                data += resp[4:4 + _BLOCK]
                addr += _BLOCK
                if self.status_fn:
                    s = chirp_common.Status()
                    s.cur = addr
                    s.max = self._memsize
                    s.msg = "Downloading from radio"
                    self.status_fn(s)
        except errors.RadioError:
            raise
        except Exception as e:
            raise errors.RadioError("Failed to download: %s" % e)
        self._mmap = memmap.MemoryMapBytes(data)
        self.process_mmap()

    def sync_out(self):
        # !!! EXPERIMENTAL / UNVERIFIED !!!
        # The write framing below mirrors the read protocol ('W' opcode,
        # 16-byte blocks). It has NOT been validated against the official
        # software's write capture. A wrong checksum/block size could
        # corrupt the codeplug (recoverable by re-writing with the
        # official tool). Verify with a serial sniff before trusting it.
        try:
            self._start_session()
            for addr in range(0, self._memsize, _BLOCK):
                chunk = self._mmap[addr:addr + _BLOCK]
                cks = sum(chunk) & 0xFF
                frame = struct.pack(">cHc", b"W", addr, bytes([_BLOCK])) \
                    + bytes(chunk) + bytes([cks])
                self.pipe.write(frame)
                if self.pipe.read(1) != _ACK:
                    raise errors.RadioError(
                        "Radio did not ACK write at 0x%04X" % addr)
                if self.status_fn:
                    s = chirp_common.Status()
                    s.cur = addr
                    s.max = self._memsize
                    s.msg = "Uploading to radio"
                    self.status_fn(s)
        except errors.RadioError:
            raise
        except Exception as e:
            raise errors.RadioError("Failed to upload: %s" % e)

    def process_mmap(self):
        self._memobj = bitwise.parse(MEM_FORMAT, self._mmap)

    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number - 1])

    # -- memory <-> chirp ---------------------------------------------------

    def _get_scan(self, index):
        return bool(self._memobj.scanadd[index // 8] & (1 << (index % 8)))

    def _set_scan(self, index, add):
        byte = index // 8
        mask = 1 << (index % 8)
        if add:
            self._memobj.scanadd[byte] |= mask
        else:
            self._memobj.scanadd[byte] &= ~mask

    def get_memory(self, number):
        index = number - 1
        _mem = self._memobj.memory[index]
        _nam = self._memobj.names[index]
        mem = chirp_common.Memory()
        mem.number = number

        if _mem.rxfreq.get_raw() == b"\xff\xff\xff\xff":
            mem.empty = True
            return mem

        mem.freq = int(_mem.rxfreq) * 10
        txfreq = int(_mem.txfreq) * 10
        if _mem.txfreq.get_raw() == b"\xff\xff\xff\xff":
            mem.duplex = "off"
            mem.offset = 0
        elif txfreq == mem.freq:
            mem.duplex = ""
            mem.offset = 0
        elif txfreq > mem.freq:
            mem.duplex = "+"
            mem.offset = txfreq - mem.freq
        else:
            mem.duplex = "-"
            mem.offset = mem.freq - txfreq

        # name
        mem.name = _nam.name.get_raw().replace(b"\xff", b"") \
            .split(b"\x00")[0].decode("ascii", "ignore").rstrip()

        # tones
        rx = _decode_tone(bytes(_mem.rxtone))
        tx = _decode_tone(bytes(_mem.txtone))
        self._apply_tones(mem, rx, tx)

        # flags
        mem.mode = "NFM" if (_mem.flags_b & _NARROW) else "FM"
        try:
            mem.power = POWER_LEVELS[_mem.power]
        except IndexError:
            mem.power = POWER_LEVELS[-1]
        mem.skip = "" if self._get_scan(index) else "S"

        # radio-specific extras
        mem.extra = RadioSettingGroup("extra", "Extra")
        busy = RadioSetting("busy_lock", "Busy Lock",
                            RadioSettingValueBoolean(
                                bool(_mem.flags_a & _BUSYLOCK)))
        spec = RadioSetting("spec", "SPEC",
                            RadioSettingValueBoolean(
                                bool(_mem.flags_a & _SPEC)))
        sqt = RadioSetting("special_qt", "Special QT/DQT",
                           RadioSettingValueBoolean(
                               bool(_mem.flags_a & _SPECIAL_QT)))
        code = RadioSetting("code", "Channel Code (hex)",
                            RadioSettingValueString(
                                0, 8, "%08X" % int(self._memobj.codes[index].code)))
        for s in (busy, spec, sqt, code):
            mem.extra.append(s)
        return mem

    def _apply_tones(self, mem, rx, tx):
        # rx/tx: None or ("Tone", hz, None) or ("DTCS", code, pol)
        def kind(t):
            return t[0] if t else None
        if rx is None and tx is None:
            mem.tmode = ""
        elif kind(tx) == "Tone" and rx is None:
            mem.tmode = "Tone"
            mem.rtone = tx[1]
        elif kind(tx) == "Tone" and kind(rx) == "Tone":
            mem.tmode = "TSQL"
            mem.ctone = rx[1]
            mem.rtone = tx[1]
        elif kind(tx) == "DTCS" and kind(rx) == "DTCS":
            mem.tmode = "DTCS"
            mem.dtcs = tx[1]
            mem.rx_dtcs = rx[1]
            mem.dtcs_polarity = (tx[2] + rx[2])
        else:
            mem.tmode = "Cross"
            txs = {"Tone": "Tone", "DTCS": "DTCS", None: ""}[kind(tx)]
            rxs = {"Tone": "Tone", "DTCS": "DTCS", None: ""}[kind(rx)]
            mem.cross_mode = "%s->%s" % (txs, rxs)
            if kind(tx) == "Tone":
                mem.rtone = tx[1]
            elif kind(tx) == "DTCS":
                mem.dtcs = tx[1]
            if kind(rx) == "Tone":
                mem.ctone = rx[1]
            elif kind(rx) == "DTCS":
                mem.rx_dtcs = rx[1]

    def set_memory(self, mem):
        index = mem.number - 1
        _mem = self._memobj.memory[index]
        _nam = self._memobj.names[index]

        if mem.empty:
            _mem.set_raw(b"\xff" * 16)
            _nam.set_raw(b"\xff" * 8)
            self._set_scan(index, False)
            return

        _mem.rxfreq.set_value(mem.freq // 10)
        if mem.duplex == "off":
            _mem.txfreq.set_raw(b"\xff\xff\xff\xff")
        elif mem.duplex == "+":
            _mem.txfreq.set_value((mem.freq + mem.offset) // 10)
        elif mem.duplex == "-":
            _mem.txfreq.set_value((mem.freq - mem.offset) // 10)
        else:
            _mem.txfreq.set_value(mem.freq // 10)

        # name (8 chars, pad 0xFF)
        nm = mem.name.strip()[:8].encode("ascii", "ignore")
        _nam.name.set_raw(nm.ljust(8, b"\xff"))

        # tones
        rx, tx = self._build_tones(mem)
        for i, b in enumerate(_encode_tone(*rx)):
            _mem.rxtone[i] = b
        for i, b in enumerate(_encode_tone(*tx)):
            _mem.txtone[i] = b

        # flags
        if mem.mode == "NFM":
            _mem.flags_b = int(_mem.flags_b) | _NARROW
        else:
            _mem.flags_b = int(_mem.flags_b) & ~_NARROW
        try:
            _mem.power = POWER_LEVELS.index(mem.power) if mem.power else 2
        except ValueError:
            _mem.power = 2
        self._set_scan(index, mem.skip != "S")

        for setting in mem.extra:
            if setting.get_name() == "busy_lock":
                _mem.flags_a = (int(_mem.flags_a) | _BUSYLOCK) if \
                    setting.value else (int(_mem.flags_a) & ~_BUSYLOCK)
            elif setting.get_name() == "spec":
                _mem.flags_a = (int(_mem.flags_a) | _SPEC) if \
                    setting.value else (int(_mem.flags_a) & ~_SPEC)
            elif setting.get_name() == "special_qt":
                _mem.flags_a = (int(_mem.flags_a) | _SPECIAL_QT) if \
                    setting.value else (int(_mem.flags_a) & ~_SPECIAL_QT)
            elif setting.get_name() == "code":
                try:
                    self._memobj.codes[index].code = int(str(setting.value), 16)
                except ValueError:
                    pass

    def _build_tones(self, mem):
        # returns (rx_args, tx_args) each a tuple for _encode_tone
        none = (None, 0, None)
        if mem.tmode == "":
            return none, none
        if mem.tmode == "Tone":
            return none, ("Tone", mem.rtone, None)
        if mem.tmode == "TSQL":
            return ("Tone", mem.ctone, None), ("Tone", mem.rtone, None)
        if mem.tmode == "DTCS":
            p = mem.dtcs_polarity
            return ("DTCS", mem.rx_dtcs, p[1]), ("DTCS", mem.dtcs, p[0])
        if mem.tmode == "Cross":
            txm, rxm = mem.cross_mode.split("->")
            p = mem.dtcs_polarity

            def side(kind, is_tx):
                if kind == "Tone":
                    return ("Tone", mem.rtone if is_tx else mem.ctone, None)
                if kind == "DTCS":
                    return ("DTCS", mem.dtcs if is_tx else mem.rx_dtcs,
                            p[0] if is_tx else p[1])
                return none
            return side(rxm, False), side(txm, True)
        return none, none

    # -- settings -----------------------------------------------------------

    # name -> (struct field, bit mask) for simple on/off flags
    _BOOL_SETTINGS = {
        "channel_name": ("ca2", 0x04), "auto_lock": ("ca3", 0x20),
        "battery_save": ("ca1", 0x08), "beep": ("ca1", 0x04),
        "roger": ("ca7", 0x20), "rx_led": ("ca3", 0x10),
        "tx_led": ("ca3", 0x08), "dynamic_mic": ("ca1", 0x02),
        "earphone": ("ca1", 0x20),
        "relay": ("rpt_flags", 0x01), "rpt_spk": ("rpt_flags", 0x02),
        "rpt_ptt": ("rpt_flags", 0x04),
        "gps_on": ("ca0", 0x80), "rx_gps": ("ca7", 0x40),
        "tx_gps": ("ca7", 0x80), "fm_forbid_rx": ("ca2", 0x08),
    }

    def get_settings(self):
        _s = self._memobj.settings
        basic = RadioSettingGroup("basic", "Basic")
        rpt = RadioSettingGroup("repeater", "Repeater")
        gps = RadioSettingGroup("gps", "GPS")
        fm = RadioSettingGroup("fm", "FM Radio")
        top = RadioSettings(basic, rpt, gps, fm)

        def add_bool(grp, name, label):
            field, mask = self._BOOL_SETTINGS[name]
            grp.append(RadioSetting(name, label, RadioSettingValueBoolean(
                bool(int(getattr(_s, field)) & mask))))

        # Basic
        basic.append(RadioSetting("squelch", "Squelch",
                     RadioSettingValueInteger(0, 9, int(_s.squelch))))
        basic.append(RadioSetting("timeout", "Time-out",
                     RadioSettingValueList(
                         TIMEOUT_LIST, current_index=min(int(_s.timeout), 9))))
        basic.append(RadioSetting("step", "Frequency Step",
                     RadioSettingValueList(
                         STEP_LIST, current_index=(int(_s.step) >> 4) & 0x0F)))
        basic.append(RadioSetting("work_mode", "Work Mode",
                     RadioSettingValueList(
                         WORKMODE_LIST, current_index=int(_s.ca2) & 0x01)))
        basic.append(RadioSetting("priority_tx", "Priority TX",
                     RadioSettingValueList(
                         PRIORITYTX_LIST, current_index=int(_s.ca0) & 0x01)))
        basic.append(RadioSetting("scan_mode", "Scan Mode",
                     RadioSettingValueList(
                         SCANMODE_LIST, current_index=(int(_s.ca1) >> 6) & 0x03)))
        basic.append(RadioSetting("channel_group", "Channel Group",
                     RadioSettingValueInteger(1, 8, int(_s.channel_group) + 1)))
        add_bool(basic, "channel_name", "Channel Name display")
        add_bool(basic, "auto_lock", "Auto Lock")
        add_bool(basic, "battery_save", "Battery Save")
        add_bool(basic, "beep", "Beep")
        add_bool(basic, "roger", "Roger")
        add_bool(basic, "rx_led", "RX-LED")
        add_bool(basic, "tx_led", "TX-LED")
        add_bool(basic, "dynamic_mic", "Dynamic Mic")
        add_bool(basic, "earphone", "Earphone Type")
        basic.append(RadioSetting("vox_gain", "VOX Gain",
                     RadioSettingValueList(
                         VOX_LIST, current_index=int(_s.ca7) & 0x0F)))
        basic.append(RadioSetting("vox_delay", "VOX Delay",
                     RadioSettingValueInteger(0, 4, int(_s.vox_delay))))

        # Repeater
        add_bool(rpt, "relay", "Relay")
        add_bool(rpt, "rpt_spk", "RPT-SPK")
        add_bool(rpt, "rpt_ptt", "RPT-PTT")
        rpt.append(RadioSetting("rpt_md", "RPT-MD",
                   RadioSettingValueList(
                       RPTMD_LIST, current_index=1 if int(_s.rpt_flags) & 0x08
                       else 0)))
        rpt.append(RadioSetting("rpt_rl", "RPT-RL",
                   RadioSettingValueInteger(0, 9, int(_s.rpt_rl))))

        # GPS
        add_bool(gps, "gps_on", "GPS ON")
        add_bool(gps, "rx_gps", "RX GPS")
        add_bool(gps, "tx_gps", "TX GPS")

        # FM Radio
        fm.append(RadioSetting("fm_channel", "FM Tuner Channel",
                  RadioSettingValueInteger(1, 25, int(_s.fm_channel))))
        fm.append(RadioSetting("fm_workmode", "FM Work Mode",
                  RadioSettingValueList(
                      FM_WORKMODE_LIST,
                      current_index=1 if int(_s.ca2) & 0x80 else 0)))
        add_bool(fm, "fm_forbid_rx", "FM Forbid Receive")

        # NOTE: Light Control, Scan Mode, Boot-Info Type are 3-state enums whose
        # full 2-bit encoding is not yet mapped; omitted until confirmed.
        return top

    def set_settings(self, settings):
        _s = self._memobj.settings
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue
            name = element.get_name()
            val = element.value
            if name in self._BOOL_SETTINGS:
                field, mask = self._BOOL_SETTINGS[name]
                cur = int(getattr(_s, field))
                setattr(_s, field, (cur | mask) if bool(val) else (cur & ~mask))
            elif name == "squelch":
                _s.squelch = int(val)
            elif name == "timeout":
                _s.timeout = TIMEOUT_LIST.index(str(val))
            elif name == "step":
                _s.step = (int(_s.step) & 0x0F) | (STEP_LIST.index(str(val)) << 4)
            elif name == "work_mode":
                _s.ca2 = (int(_s.ca2) & ~0x01) | WORKMODE_LIST.index(str(val))
            elif name == "priority_tx":
                _s.ca0 = (int(_s.ca0) & ~0x01) | PRIORITYTX_LIST.index(str(val))
            elif name == "scan_mode":
                _s.ca1 = (int(_s.ca1) & ~0xC0) | \
                    (SCANMODE_LIST.index(str(val)) << 6)
            elif name == "channel_group":
                _s.channel_group = int(val) - 1
            elif name == "vox_gain":
                _s.ca7 = (int(_s.ca7) & 0xF0) | VOX_LIST.index(str(val))
            elif name == "vox_delay":
                _s.vox_delay = int(val)
            elif name == "rpt_md":
                _s.rpt_flags = (int(_s.rpt_flags) & ~0x08) | \
                    (0x08 if str(val) == "2" else 0)
            elif name == "rpt_rl":
                _s.rpt_rl = int(val)
            elif name == "fm_channel":
                _s.fm_channel = int(val)
            elif name == "fm_workmode":
                _s.ca2 = (int(_s.ca2) | 0x80) if str(val) == "VFO" \
                    else (int(_s.ca2) & ~0x80)

    @classmethod
    def match_model(cls, filedata, filename):
        # Weak signature: exact codeplug size + the 16-byte 0xFF reserved
        # header. (No stronger magic is known yet.)
        return len(filedata) == _MEMSIZE and filedata[:16] == b"\xff" * 16
