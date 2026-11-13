# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2026 Vinay Shanbhag.  Part of ni-gpib-usb-hs (see LICENSE).
"""A convenience wrapper for the IFR (Marconi) 2031 10 kHz - 2.7 GHz signal generator.

Drives the generator over GPIB through :class:`NIUSBGPIB`. The command set
(IEEE 488.2 + the device-dependent mnemonics of chapter 3-2 of the 2030 Series
manual) is shared by the 2030 (1.35 GHz), 2031 (2.7 GHz) and 2032 (5.4 GHz), so
this wrapper works with all three.

    from ni_gpib_usb_hs import IFR2031
    with IFR2031(addr=25) as sg:
        print(sg.idn())
        sg.set_frequency(433.92e6)
        sg.set_level(-30)                # dBm
        sg.fm(3e3, source="INTF4")       # 3 kHz deviation, 1 kHz internal tone
        sg.modulation_on()
        sg.rf_on()

Notes on the instrument:

* ``*RST`` leaves the RF level at -144 dBm and the carrier at the maximum
  frequency; the RF output status is part of that default state.
* Per-modulator ``ON``/``OFF`` only switch that modulator "locally": it also
  needs to be part of the active modulation mode (:meth:`set_mode`) and
  modulation must be globally on (:meth:`modulation_on`).
* The generator has a 256 character input buffer; keep a single message under
  that (``write`` calls here are all well below it).
* ``*IDN?`` / ``*OPT?`` return arbitrary ASCII and so must be the last query
  in a message. Each query here is sent as its own message, so this is handled.
"""
from __future__ import annotations

import re
import time
from decimal import Decimal

from .controller import NIUSBGPIB, GpibError

# ---------------------------------------------------------------------------
# constants taken from the manual
# ---------------------------------------------------------------------------
RF_UNITS = ("DBM", "DBV", "DBMV", "DBUV", "V", "MV", "UV")
LF_UNITS = ("DBM", "DBV", "DBMV", "V", "MV", "UV")          # LFGL:UNITS
LF_LEVEL_SUFFIXES = ("V", "MV", "UV", "DBMV")               # LFGL:VALUE suffixes
WAVEFORMS = ("SIN", "TRI", "SQU")
MOD_SOURCES = (
    "INTF1", "INTF2", "INTF3", "INTF4", "INTF5", "INTF6",
    "EXT1DC", "EXT1AC", "EXT1ALC", "EXT2DC", "EXT2AC", "EXT2ALC",
)
SWEEP_TYPES = ("OFF", "CFRQ", "RFLV", "LFGF", "LFGL",
               "INTF1", "INTF2", "INTF3", "INTF4", "INTF5", "INTF6", "HOP")
SWEEP_MODES = ("SNGL", "CONT", "EXT")
FREQ_STANDARDS = ("INT0", "INT1", "INT5", "INT10", "EXT1", "EXT5", "EXT10")
TONE_STANDARDS = ("CCIR", "EURO", "DZVEI", "ZVEI1", "ZVEI2", "EEA", "EIA",
                  "NATEL", "TEMP", "USER1", "USER2", "DTMF")
TONE_CHANNELS = ("AM1", "AM2", "FM1", "FM2", "PM1", "PM2",
                 "TOTAL_AM", "TOTAL_FM", "TOTAL_PM", "NO_TONES")
LF_MONITOR = ("AM1S", "AM2S", "AMD", "ANG1S", "ANG2S", "ANGD", "OFF")
TRIGGER_FUNCS = ("SEQT", "FLSWP", "SSSWP", "VOID")           # DEVTRG
EXT_TRIGGER_FUNCS = TRIGGER_FUNCS + ("MEMUP", "MEMDN")       # EXTTRG

# per-parameter defaults for sweep start/stop/marker values
_SWEEP_UNITS = {"CFRQ": "HZ", "RFLV": "DBM", "LFGF": "HZ", "LFGL": "V", "INTF": "HZ"}

# memory store kinds -> (STO/RCL/ERASE sub-header, max index)
_STORES = {"full": ("FULL", 49), "part": ("PART", 49), "cfrq": ("CFRQ", 99),
           "seqt": ("SEQT", 19), "sweep": ("SWEEP", 19)}

# Valid modulation combinations (manual table); '1' is optional, so we normalise AM->AM1 etc.
_VALID_MODES = {frozenset(s) for s in (
    # single
    ("AM1",), ("FM1",), ("PM1",), ("WBFM",), ("PULSE",),
    # composite
    ("AM1", "AM2"), ("FM1", "FM2"), ("PM1", "PM2"),
    # dual
    ("AM1", "FM1"), ("AM1", "PM1"), ("AM1", "WBFM"),
    ("PULSE", "FM1"), ("PULSE", "PM1"), ("PULSE", "WBFM"),
    # dual composite
    ("AM1", "AM2", "FM1", "FM2"), ("AM1", "AM2", "PM1", "PM2"),
    ("AM1", "AM2", "WBFM"), ("PULSE", "FM1", "FM2"), ("PULSE", "PM1", "PM2"),
)}

# Status register bit meanings (manual pages 3-2-43 .. 3-2-46)
ESR_BITS = {0: "operation complete", 1: "request control", 2: "query error",
            3: "device dependent error", 4: "execution error", 5: "command error",
            6: "user request", 7: "power on"}
HARDWARE_BITS = {0: "reverse power protection tripped", 1: "fractional-n out-of-lock",
                 2: "vcxo out-of-lock", 3: "frequency standard missing",
                 4: "ext mod 1 ALC signal too low", 5: "ext mod 1 ALC signal too high",
                 6: "ext mod 2 ALC signal too low", 7: "ext mod 2 ALC signal too high",
                 14: "rf level uncalibrated", 15: "extended hysteresis"}
COUPLING_BITS = {0: "rf level restricted by requested AM",
                 1: "FM restricted by requested carrier frequency",
                 2: "wideband FM restricted by requested carrier frequency",
                 3: "AM2 depth restricted by AM1 depth",
                 4: "FM2 deviation restricted by FM1 deviation",
                 5: "PM2 deviation restricted by PM1 deviation",
                 6: "number of sweep steps restricted by other parameters"}
INSTRUMENT_BITS = {0: "sweep", 1: "tones", 2: "selfcal", 3: "dc fm null"}
STATUS_BYTE_BITS = {1: "instrument status (ssb)", 2: "coupling status (csb)",
                    3: "hardware status (hsb)", 4: "message available (mav)",
                    5: "event status (esb)", 6: "master summary / request service",
                    7: "error queue non-empty (erb)"}

# Error numbers reported by ERROR? (operating manual chapter 3-1 error tables).
ERROR_TEXT = {
    1: "RPP Tripped", 2: "Fractional N Out of Lock", 3: "Int. Standard Failure",
    4: "Ext. Standard Failure", 5: "Incorrect Ext. Standard", 6: "VCXO Out of Lock",
    7: "Ext1 Too Low", 8: "Ext1 Too High", 9: "Ext2 Too Low", 10: "Ext2 Too High",
    17: "RF Level limited by AM", 18: "FM limited by Carrier",
    19: "WBFM limited by Carrier", 20: "AM2 limited by AM1", 21: "FM2 limited by FM1",
    22: "PM2 limited by PM1", 23: "Steps limited by Span", 24: "FM Selfcal Error",
    25: "Internal Osc.1 Missing", 26: "Real Time Clock Problem",
    27: "Calibration Date Expired", 28: "Pad Calibration Checksum",
    29: "RF Calibration Checksum", 30: "FM Calibration Checksum",
    31: "Path/Source Calibration", 32: "Absolute Mod. Calibration",
    33: "Freq. Std. Calibration",
    46: "Recall Checksum", 47: "Incorrect Setup", 48: "Invalid Memory Number",
    49: "Modulation Not Enabled", 50: "Out of Range", 51: "Carrier Outside Limits",
    52: "RF Level Outside Limits", 53: "Mod Rate Outside Limits",
    54: "LF Freq. Outside Limits", 55: "LF Level Outside Limits",
    56: "AM Outside Limits", 57: "FM Outside Limits", 58: "PM Outside Limits",
    59: "WBFM Outside Limits", 60: "Carrier Step Too Big", 61: "RF Level Step Too Big",
    62: "Mod Rate Step Too Big", 63: "LF Freq. Step Too Big", 64: "LF Level Step Too Big",
    65: "AM Step Too Big", 66: "FM Step Too Big", 67: "PM Step Too Big",
    68: "Invalid Latch Number", 69: "Invalid Latch Data",
    70: "Sweep Start Out of Range", 71: "Sweep Stop Out of Range",
    72: "Sweep Steps Out of Range", 73: "Sweep Time Out of Range",
    74: "Sweep Marker Out of Range", 75: "Attenuator EAROM Read",
    76: "Attenuator EAROM Write", 79: "EAROM Write Error", 80: "EAROM Read Error",
    81: "EAROM Wrap Around Error", 82: "Continuous Tone Checksum",
    83: "Sequential Tone Checksum", 84: "Tone data Out of Range",
    85: "Tone Offset Out of Range", 86: "Clock Data Entry Error",
    87: "At Top Limit", 88: "At Bottom Limit", 89: "Ext. Trigger Disabled",
    90: "Int. Std. Not Selected", 91: "RF levelling fault",
    92: "REPEAT THIS CALIBRATION",
    102: "Mnemonic Fault", 103: "Block Definition", 104: "Block Size",
    105: "Numeric Syntax", 106: "Data Expected", 107: "Illegal Data",
    108: "Terminator Expected", 109: "GET Error", 110: "EOM Error",
    111: "Illegal Modulation Mode", 112: "No Such Monitor Mode", 113: "Cannot Monitor",
    114: "Instrument Mode Wrong", 115: "Lost Data After Comma",
    116: "Unterminated (query error)", 117: "Interrupted (query error)",
    118: "Deadlock (query error)", 119: "Missing Quote", 120: "Terminator Expected",
    121: "String Length", 122: "Illegal Tone Character", 123: "Illegal Duration Char",
    124: "Illegal Standard", 125: "Illegal Save Destination",
    126: "Illegal Seq Tones Mode", 127: "Overflow", 128: "Data Too Long",
    129: "Voltage Type Error", 130: "Sweep Not Possible", 131: "Unknown Cal Point",
    132: "Unknown RF Band", 133: "Unknown Instrument Mode", 134: "User Data Checksum",
    135: "Query Lost after arb. char", 136: "Unknown Freq. Standard",
    137: "User Data Locked", 138: "Trigger Unknown", 139: "Illegal Tones Operation",
    140: "Error in Char Data", 141: "Wrong RF units", 142: "Data Unknown",
    143: "Negative Value Illegal", 144: "Illegal Modulation Mode",
    145: "Unavailable Mod Source", 146: "Wrong Family For Command",
    147: "Not Suitable For Hopping", 148: "Hopping Sequence Full",
    171: "Main RAM Faulty", 172: "Main PROM Faulty", 173: "Microwave Board Error",
    174: "Attenuator Type Unknown", 175: "Wrong Attenuator fitted",
    255: "Error queue full",
}


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------
def _n(x) -> str:
    """Format a number for the instrument: plain decimal, never an exponent."""
    return format(Decimal(repr(float(x))), "f")


def _choice(value: str, allowed, name: str) -> str:
    v = str(value).upper()
    if v not in allowed:
        raise ValueError(f"{name} must be one of {', '.join(allowed)}; got {value!r}")
    return v


def _parse(resp: str):
    """Split a ``;``-separated response into ``[(key, value), ...]``.

    ``key`` is the last colon-separated element of each unit's header and
    ``value`` is the text after the first space ("" for bare mnemonics such as
    ``ON`` or ``MKRON``). E.g. ``":FM1:DEVN 25000.0;INTF1;ON;INC 1000.0"`` ->
    ``[("DEVN","25000.0"), ("INTF1",""), ("ON",""), ("INC","1000.0")]``.
    """
    pairs = []
    for unit in resp.strip().split(";"):
        unit = unit.strip()
        while unit:
            parts = unit.split(None, 1)
            head, val = parts[0], (parts[1].strip() if len(parts) > 1 else "")
            rest = ""
            # quirk in the manual: ":SWEEP:TYPE HOP:MODE SNGL" (no ';' between units)
            if val and not val.startswith(('"', "'")) and ":" in val:
                val, rest = val.split(":", 1)
            pairs.append((head.split(":")[-1].upper(), val.strip().strip("\"'")))
            unit = rest.strip()
    return pairs


def _flags(pairs):
    return {k for k, v in pairs if v == ""}


def _get(pairs, key, default=None):
    for k, v in reversed(pairs):
        if k == key:
            return v
    return default


def _bits(value: int, table: dict):
    return [name for bit, name in table.items() if value & (1 << bit)]


class IFR2031:
    """IFR / Marconi 2031 signal generator over GPIB.

    Args:
        addr: the generator's GPIB primary address (set from the UTIL menu on the
            instrument; it must match).
        gpib: an existing :class:`NIUSBGPIB` to share; if None, one is opened.
        controller_pad: controller GPIB address when opening our own bus.
        reset: send ``*RST`` + ``*CLS`` on connect (default True). Note ``*RST``
            sets -144 dBm and the maximum carrier frequency.
    """

    def __init__(self, addr: int = 25, gpib: NIUSBGPIB | None = None,
                 controller_pad: int = 0, reset: bool = True):
        self.addr = addr
        self._own_gpib = gpib is None
        self.gpib = gpib if gpib is not None else NIUSBGPIB(my_pad=controller_pad)
        if reset:
            self.reset()
        # Addressing the generator over the bus puts it in remote automatically
        # (the screen shows a single [LOCAL] key); there is no "go remote" command.

    # -- context manager --
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self):
        """Close the underlying GPIB controller, if we opened it ourselves."""
        if self._own_gpib and self.gpib is not None:
            self.gpib.close()
            self.gpib = None

    # -- raw access --
    def write(self, cmd: str):
        """Send a command (no reply expected)."""
        self.gpib.write(self.addr, cmd)

    def query(self, cmd: str) -> str:
        """Send a query and return the stripped string reply."""
        return self.gpib.query(self.addr, cmd)

    def query_float(self, cmd: str) -> float:
        s = self.query(cmd)
        try:
            return float(s)
        except ValueError as e:
            raise GpibError(f"expected a number from {cmd!r}, got {s!r}") from e

    def query_int(self, cmd: str) -> int:
        s = self.query(cmd)
        try:
            return int(float(s))
        except ValueError as e:
            raise GpibError(f"expected an integer from {cmd!r}, got {s!r}") from e

    def _query_pairs(self, cmd: str):
        return _parse(self.query(cmd))

    # -- IEEE 488.2 common commands --
    def idn(self) -> str:
        """``*IDN?`` -> ``'IFR,2031,<serial>,<firmware>'``."""
        return self.query("*IDN?")

    def options(self) -> list[str]:
        """Fitted options (``*OPT?``), e.g. ``['SECOND OSCILLATOR', 'PULSE MODULATION']``."""
        s = self.query("*OPT?")
        return [] if s.strip() == "0" else [o.strip() for o in s.split(",") if o.strip()]

    def reset(self):
        """``*RST`` + ``*CLS``."""
        self.write("*RST")
        self.write("*CLS")

    def clear(self):
        """Clear status registers and the error queue (``*CLS``)."""
        self.write("*CLS")

    def self_test(self) -> bool:
        """``*TST?`` - True when the GPIB interface and processor are working."""
        return self.query_int("*TST?") == 0

    def opc(self):
        """``*OPC`` - set the operation-complete bit in the standard event register."""
        self.write("*OPC")

    def wait_opc(self) -> bool:
        """``*OPC?`` - blocks until preceding operations are done; True when it returns 1."""
        return self.query_int("*OPC?") == 1

    def wai(self):
        """``*WAI`` - hold off overlapped commands until the preceding ones finish."""
        self.write("*WAI")

    def trigger(self):
        """``*TRG`` - same as Group Execute Trigger (action set by :meth:`device_trigger`)."""
        self.write("*TRG")

    # -- errors --
    def errors(self):
        """Drain ``ERROR?`` and return a list of ``(code, text)``.

        255 means the 100-entry queue overflowed.
        """
        out = []
        for _ in range(101):
            m = re.search(r"-?\d+", self.query("ERROR?"))
            if not m:
                break
            code = int(m.group())
            if code == 0:
                break
            out.append((code, ERROR_TEXT.get(code, "unknown error")))
        return out

    def check_errors(self):
        """Raise :class:`GpibError` if the error queue holds anything."""
        errs = self.errors()
        if errs:
            raise GpibError("; ".join(f"{c}: {t}" for c, t in errs))

    # -- instrument mode --
    def imode(self, mode: str) -> None:
        """Select ``'NORMAL'`` (signal generator) or ``'SWEEPER'`` operation."""
        self.write(f"IMODE {_choice(mode, ('NORMAL', 'SWEEPER'), 'mode')}")

    # -- carrier frequency --
    def set_frequency(self, hz: float, step_hz: float | None = None):
        cmd = f"CFRQ:VALUE {_n(hz)}HZ"
        if step_hz is not None:
            cmd += f";INC {_n(step_hz)}HZ"
        self.write(cmd)

    def frequency(self) -> float:
        """Carrier frequency in Hz."""
        return float(_get(self._query_pairs("CFRQ?"), "VALUE"))

    def frequency_step(self) -> float:
        return float(_get(self._query_pairs("CFRQ?"), "INC"))

    def set_frequency_step(self, hz: float):
        self.write(f"CFRQ:INC {_n(hz)}HZ")

    def frequency_up(self, xfer: bool = True):
        """Step the carrier up one step (``xfer`` makes the new value permanent)."""
        self.write("CFRQ:UP" + (";XFER" if xfer else ""))

    def frequency_down(self, xfer: bool = True):
        self.write("CFRQ:DN" + (";XFER" if xfer else ""))

    def frequency_return(self):
        """``CFRQ:RET`` - return to the original (pre-step) setting."""
        self.write("CFRQ:RET")

    def set_carrier_phase(self, steps: int):
        """Adjust carrier phase in steps of pi/128 rad (~1.4 deg), -255..+255."""
        steps = int(steps)
        if not -255 <= steps <= 255:
            raise ValueError("phase steps must be within +/-255")
        self.write(f"CFRQ:PHASE {steps}")

    # -- RF level --
    def set_level(self, level: float, units: str | None = None,
                  on: bool | None = None):
        """Set the RF output level.

        ``units`` is one of dBm/dBV/dBmV/dBuV/V/mV/uV. If omitted the suffix is
        left off and the instrument uses its current default units (dBm unless
        changed with :meth:`set_rf_units`). ``on`` also switches the output.
        """
        suffix = _choice(units, RF_UNITS, "units") if units else ""
        cmd = f"RFLV:VALUE {_n(level)}{suffix}"
        if on is not None:
            cmd += ";ON" if on else ";OFF"
        self.write(cmd)

    def rf_level_info(self) -> dict:
        """``{'units','type','value','step','on'}`` (value/step are in ``units``)."""
        p = self._query_pairs("RFLV?")
        return {
            "units": _get(p, "UNITS"),
            "type": _get(p, "TYPE"),          # EMF / PD (only present for voltage units)
            "value": float(_get(p, "VALUE")),
            "step": float(_get(p, "INC")),
            "on": "ON" in _flags(p),
        }

    def rf_level(self) -> float:
        """Current RF level, in the instrument's current units (see :meth:`rf_level_info`)."""
        return self.rf_level_info()["value"]

    def set_rf_units(self, units: str):
        self.write(f"RFLV:UNITS {_choice(units, RF_UNITS, 'units')}")

    def set_rf_type(self, kind: str):
        """EMF or PD for voltage-related units."""
        self.write(f"RFLV:TYPE {_choice(kind, ('EMF', 'PD'), 'type')}")

    def set_level_step(self, db: float):
        self.write(f"RFLV:INC {_n(db)}DB")

    def level_up(self, xfer: bool = True):
        self.write("RFLV:UP" + (";XFER" if xfer else ""))

    def level_down(self, xfer: bool = True):
        self.write("RFLV:DN" + (";XFER" if xfer else ""))

    def level_return(self):
        self.write("RFLV:RETN")

    def rf_on(self):
        self.write("RFLV:ON")

    def rf_off(self):
        self.write("RFLV:OFF")

    def rf_is_on(self) -> bool:
        return self.rf_level_info()["on"]

    def reset_reverse_power_trip(self):
        """``RPPR`` - reset the reverse-power protection trip."""
        self.write("RPPR")

    # RF level offset (per band)
    def set_level_offset(self, db: float, on: bool = True, enable: bool = True):
        """Set the RF offset (|dB| of cable/switch loss) for the current band."""
        self.write(f"RFLV:OFFS:VALUE {_n(db)}DB;{'ON' if on else 'OFF'};"
                   f"{'ENABLE' if enable else 'DISABLE'}")

    def level_offset_info(self) -> dict:
        """``{'carrier_hz','offset_db','on','enabled'}`` for the current band."""
        p = self._query_pairs("RFLV:OFFS?")
        vals = [v for k, v in p if k == "VALUE"]
        fl = _flags(p)
        return {"carrier_hz": float(vals[0]), "offset_db": float(vals[-1]),
                "on": "ON" in fl, "enabled": "ENABLE" in fl}

    def save_level_offsets(self):
        self.write("RFLV:OFFS:SAVE")

    # extended hysteresis (no mechanical attenuator switching)
    def set_hysteresis(self, enable: bool):
        self.write(f"RFLV:HYST:{'ENABLE' if enable else 'DISABLE'}")

    def hysteresis_enabled(self) -> bool:
        return self.query("RFLV:HYST?").strip().upper().endswith("ENABLE")

    # RF level limit
    def set_level_limit(self, value: float, units: str | None = None,
                        enable: bool = True, save: bool = False):
        suffix = _choice(units, RF_UNITS, "units") if units else ""
        cmd = (f"RFLV:LIMIT:VALUE {_n(value)}{suffix};"
               f"{'ENABLE' if enable else 'DISABLE'}")
        if save:
            cmd += ";SAVE"
        self.write(cmd)

    def level_limit_info(self) -> dict:
        p = self._query_pairs("RFLV:LIMIT?")
        return {"units": _get(p, "UNITS"), "type": _get(p, "TYPE"),
                "value": float(_get(p, "VALUE")), "enabled": "ENABLE" in _flags(p)}

    # -- modulation mode / global control --
    def set_mode(self, *modes: str):
        """Select the modulation mode, e.g. ``set_mode('FM')``, ``set_mode('AM','FM')``.

        Combinations are validated against the manual's table (``AM`` == ``AM1``).
        ``PULSE`` without option 002 selects low-intermodulation mode.
        """
        norm = []
        for m in modes:
            m = str(m).upper()
            m = {"AM": "AM1", "FM": "FM1", "PM": "PM1"}.get(m, m)
            norm.append(m)
        if frozenset(norm) not in _VALID_MODES or len(set(norm)) != len(norm):
            raise ValueError(f"invalid modulation combination: {modes!r}")
        self.write("MODE " + ",".join(norm))

    def mode(self) -> list[str]:
        """Current modulation mode as a list, e.g. ``['FM1', 'FM2']``."""
        resp = self.query("MODE?").strip()
        body = resp.split(None, 1)[1] if " " in resp else resp.split(":")[-1]
        return [m.strip().upper() for m in body.split(",") if m.strip()]

    def modulation_on(self):
        self.write("MOD:ON")

    def modulation_off(self):
        self.write("MOD:OFF")

    def modulation_enabled(self) -> bool:
        return self.query("MOD?").strip().upper().endswith("ON")

    # -- generic modulator helper (FM/PM/AM share the same shape) --
    def _modulator(self, hdr, key, value, unit, source, on, step, step_unit):
        parts = []
        if value is not None:
            parts.append(f"{hdr}:{key} {_n(value)}{unit}")
        if step is not None:
            parts.append(f"{'INC' if parts else hdr + ':INC'} {_n(step)}{step_unit}")
        if source is not None:
            src = _choice(source, MOD_SOURCES, "source")
            parts.append(src if parts else f"{hdr}:{src}")
        if on is not None:
            tok = "ON" if on else "OFF"
            parts.append(tok if parts else f"{hdr}:{tok}")
        if parts:
            self.write(";".join(parts))

    def _modulator_info(self, hdr, key) -> dict:
        p = self._query_pairs(f"{hdr}?")
        fl = _flags(p)
        src = next((f for f in fl if f.startswith(("INTF", "EXT"))), None)
        return {"value": float(_get(p, key)), "source": src,
                "on": "ON" in fl, "step": float(_get(p, "INC"))}

    # frequency modulation
    def fm(self, deviation_hz: float | None = None, channel: int = 1,
           source: str | None = None, on: bool | None = None,
           step_hz: float | None = None):
        """Set FM1/FM2 deviation (Hz), source, local on/off and step."""
        self._modulator(f"FM{self._ch(channel)}", "DEVN", deviation_hz, "HZ",
                        source, on, step_hz, "HZ")

    def fm_info(self, channel: int = 1) -> dict:
        """``{'value'(Hz),'source','on','step'}``."""
        return self._modulator_info(f"FM{self._ch(channel)}", "DEVN")

    # phase modulation
    def pm(self, deviation_rad: float | None = None, channel: int = 1,
           source: str | None = None, on: bool | None = None,
           step_rad: float | None = None):
        """Set PM1/PM2 deviation (radians), source, local on/off and step."""
        self._modulator(f"PM{self._ch(channel)}", "DEVN", deviation_rad, "RAD",
                        source, on, step_rad, "RAD")

    def pm_info(self, channel: int = 1) -> dict:
        return self._modulator_info(f"PM{self._ch(channel)}", "DEVN")

    # amplitude modulation
    def am(self, depth_pct: float | None = None, channel: int = 1,
           source: str | None = None, on: bool | None = None,
           step_pct: float | None = None):
        """Set AM1/AM2 depth (percent), source, local on/off and step."""
        self._modulator(f"AM{self._ch(channel)}", "DEPTH", depth_pct, "PCT",
                        source, on, step_pct, "PCT")

    def am_info(self, channel: int = 1) -> dict:
        return self._modulator_info(f"AM{self._ch(channel)}", "DEPTH")

    @staticmethod
    def _ch(channel) -> int:
        if int(channel) not in (1, 2):
            raise ValueError("channel must be 1 or 2")
        return int(channel)

    def modulator_step(self, kind: str, direction: str, channel: int = 1,
                       xfer: bool = True):
        """Step a modulator's value: ``kind`` FM/PM/AM, ``direction`` UP/DN/RETN."""
        kind = _choice(kind, ("FM", "PM", "AM"), "kind")
        d = _choice(direction, ("UP", "DN", "RETN"), "direction")
        self.write(f"{kind}{self._ch(channel)}:{d}" + (";XFER" if xfer and d != "RETN" else ""))

    # wideband FM
    def wbfm(self, deviation_hz: float | None = None, on: bool | None = None,
             coupling: str | None = None):
        """Wideband FM deviation (Hz), local on/off and ``coupling`` ('AC'/'DC')."""
        parts = []
        if deviation_hz is not None:
            parts.append(f"WBFM:DEVN {_n(deviation_hz)}HZ")
        if on is not None:
            tok = "ON" if on else "OFF"
            parts.append(tok if parts else f"WBFM:{tok}")
        if coupling is not None:
            c = _choice(coupling, ("AC", "DC"), "coupling")
            parts.append(c if parts else f"WBFM:{c}")
        if parts:
            self.write(";".join(parts))

    def wbfm_info(self) -> dict:
        p = self._query_pairs("WBFM?")
        fl = _flags(p)
        return {"value": float(_get(p, "DEVN")), "on": "ON" in fl,
                "coupling": "DC" if "DC" in fl else "AC"}

    def dcfm_null(self):
        """``DCFMNL`` - perform the DC FM / WBFM null operation."""
        self.write("DCFMNL")

    # pulse modulation (option 002)
    def pulse(self, on: bool):
        """Pulse modulation on; off selects low-intermodulation mode."""
        self.write(f"PULSE:{'ON' if on else 'OFF'}")

    def pulse_is_on(self) -> bool:
        return self.query("PULSE?").strip().upper().endswith("ON")

    def pulse_cw_burst_suppression(self, enable: bool):
        self.write(f"PULSE:CAL:{'ENABLE' if enable else 'DISABLE'}")

    def pulse_cw_burst_suppression_enabled(self) -> bool:
        return self.query("PULSE:CAL?").strip().upper().split(":")[-1] in ("ENABLE", "ENABLED")

    # -- modulation oscillators (INTF1..INTF6) --
    @staticmethod
    def _osc(n) -> str:
        n = int(n)
        if not 1 <= n <= 6:
            raise ValueError("oscillator must be 1..6")
        return f"INTF{n}"

    def mod_osc(self, n: int, freq_hz: float | None = None,
                waveform: str | None = None, step_hz: float | None = None,
                phase_deg: float | None = None):
        """Set modulation oscillator ``n`` (1-6): frequency, waveform (SIN/TRI/SQU), step, phase."""
        hdr = self._osc(n)
        parts = []
        if freq_hz is not None:
            parts.append(f"{hdr}:FREQ {_n(freq_hz)}HZ")
        if step_hz is not None:
            parts.append(f"INC {_n(step_hz)}HZ" if parts else f"{hdr}:INC {_n(step_hz)}HZ")
        if waveform is not None:
            w = _choice(waveform, WAVEFORMS, "waveform")
            parts.append(w if parts else f"{hdr}:{w}")
        if phase_deg is not None:
            parts.append(f"PHASE {_n(phase_deg)}DEG" if parts else f"{hdr}:PHASE {_n(phase_deg)}DEG")
        if parts:
            self.write(";".join(parts))

    def mod_osc_info(self, n: int) -> dict:
        """``{'freq','step','waveform'}``, or ``{'standard','tone'}`` when in CTCSS mode."""
        p = self._query_pairs(f"{self._osc(n)}?")
        for std in ("CTC1", "CTC2", "USER", "TEMP"):
            if _get(p, std) not in (None,):
                return {"standard": std, "tone": int(_get(p, std))}
        fl = _flags(p)
        wave = next((w for w in WAVEFORMS if w in fl), None)
        return {"freq": float(_get(p, "FREQ")), "step": float(_get(p, "INC")),
                "waveform": wave}

    def mod_osc_ctcss(self, n: int, standard: str, tone: int):
        """Select continuous tone ``tone`` (0-15) from CTC1/CTC2/USER/TEMP on oscillator ``n``."""
        std = _choice(standard, ("CTC1", "CTC2", "USER", "TEMP"), "standard")
        if not 0 <= int(tone) <= 15:
            raise ValueError("tone must be 0..15")
        self.write(f"{self._osc(n)}:{std} {int(tone)}")

    def mod_osc_step(self, n: int, direction: str, xfer: bool = True):
        d = _choice(direction, ("UP", "DN", "RETN"), "direction")
        self.write(f"{self._osc(n)}:{d}" + (";XFER" if xfer and d != "RETN" else ""))

    # -- CTCSS tone editing --
    def ctcss_load(self, standard: str):
        """Copy CTC1/CTC2/USER into TEMP for editing."""
        self.write(f"CTONES:EDIT:LOAD {_choice(standard, ('CTC1', 'CTC2', 'USER'), 'standard')}")

    def ctcss_edit(self, tone_number: int, freq_hz: float):
        if not 0 <= int(tone_number) <= 15:
            raise ValueError("tone number must be 0..15")
        self.write(f"CTONES:EDIT:TNUM {int(tone_number)};TFRQ {_n(freq_hz)}HZ")

    def ctcss_save(self):
        """Save TEMP to USER for non-volatile storage."""
        self.write("CTONES:EDIT:SAVE")

    def ctcss_edit_info(self) -> dict:
        p = self._query_pairs("CTONES:EDIT?")
        return {"tone": int(_get(p, "TNUM")), "freq_hz": float(_get(p, "TFRQ"))}

    # -- sequential tones --
    def seqt_sequence(self, tones: str, durations: str | None = None):
        """Set the tone sequence (up to 16 chars 0-9/A-F) and optional duration mask
        (up to 15 chars of ``-`` normal / ``E`` extended)."""
        tones = str(tones).upper()
        if len(tones) > 16:
            raise ValueError("tone sequence is limited to 16 characters")
        cmd = f'SEQT:SEQ "{tones}"'
        if durations is not None:
            if len(durations) > 15 or set(durations.upper()) - set("-E"):
                raise ValueError("duration mask: up to 15 chars of '-' or 'E'")
            cmd += f';DUR "{durations.upper()}"'
        self.write(cmd)

    def seqt_info(self) -> dict:
        p = self._query_pairs("SEQT?")
        return {"sequence": _get(p, "SEQ"), "durations": _get(p, "DUR")}

    def seqt_send(self, times: int = 1):
        """Send the sequence ``times`` times (1-9) or continuously (10)."""
        if not 1 <= int(times) <= 10:
            raise ValueError("times must be 1..9, or 10 for continuous")
        self.write(f"SEQT:SEND {int(times)}")

    def seqt_stop(self):
        self.write("SEQT:STOP")

    def seqt_mode(self, standard: str | None = None, channel: str | None = None):
        """Select the tone standard and/or modulation channel."""
        parts = []
        if standard is not None:
            parts.append(f"SEQT:MODE:STD {_choice(standard, TONE_STANDARDS, 'standard')}")
        if channel is not None:
            c = _choice(channel, TONE_CHANNELS, "channel")
            parts.append(f"MOD {c}" if parts else f"SEQT:MODE:MOD {c}")
        if parts:
            self.write(";".join(parts))

    def seqt_mode_info(self) -> dict:
        p = self._query_pairs("SEQT:MODE?")
        return {"standard": _get(p, "STD"), "channel": _get(p, "MOD")}

    def seqt_params(self, extended_ms: float | None = None, start_delay_ms: float | None = None,
                    shift_pct: float | None = None, repeat_tone: str | None = None,
                    tone_ms: float | None = None, gap_ms: float | None = None):
        """Set sequential tone parameters. ``tone_ms``/``gap_ms`` are the DTMF tone
        duration and inter-element gap (``SEQT:PARAM:TDUR`` / ``TGAP``)."""
        items = []
        if extended_ms is not None:
            items.append(f"EXTD {_n(extended_ms)}MS")
        if start_delay_ms is not None:
            items.append(f"SDLY {_n(start_delay_ms)}MS")
        if shift_pct is not None:
            items.append(f"SHFT {_n(shift_pct)}PCT")
        if repeat_tone is not None:
            items.append(f'RPTT "{str(repeat_tone).upper()}"')
        if tone_ms is not None:
            items.append(f"TDUR {_n(tone_ms)}MS")
        if gap_ms is not None:
            items.append(f"TGAP {_n(gap_ms)}MS")
        if items:
            self.write("SEQT:PARAM:" + ";".join(items))

    def seqt_params_info(self) -> dict:
        p = self._query_pairs("SEQT:PARAM?")
        return {k.lower(): v for k, v in p}

    def seqt_edit(self, tone_number: int | None = None, freq_hz: float | None = None,
                  tone_ms: float | None = None, gap_ms: float | None = None,
                  load: str | None = None, save: str | None = None):
        """Edit a tone of the TEMP standard: select tone, frequency, duration, gap;
        ``load`` copies a standard to TEMP first, ``save`` ('USER1'/'USER2') copies TEMP out."""
        items = []
        if load is not None:
            std = _choice(load, [s for s in TONE_STANDARDS if s not in ("TEMP", "DTMF")], "load")
            items.append(f"LOAD {std}")
        if tone_number is not None:
            items.append(f"TNUM {int(tone_number)}")
        if freq_hz is not None:
            items.append(f"TFRQ {_n(freq_hz)}HZ")
        if tone_ms is not None:
            items.append(f"TDUR {_n(tone_ms)}MS")
        if gap_ms is not None:
            items.append(f"TGAP {_n(gap_ms)}MS")
        if save is not None:
            items.append(f"SAVE {_choice(save, ('USER1', 'USER2'), 'save')}")
        if items:
            self.write("SEQT:EDIT:" + ";".join(items))

    def seqt_edit_info(self) -> dict:
        p = self._query_pairs("SEQT:EDIT?")
        return {"tone": int(_get(p, "TNUM")), "freq_hz": float(_get(p, "TFREQ")),
                "tone_ms": int(_get(p, "TDUR")), "gap_ms": int(_get(p, "TGAP"))}

    # -- LF output / LF generator --
    def lf_on(self):
        self.write("LF:ON")

    def lf_off(self):
        self.write("LF:OFF")

    def lf_select_generator(self):
        """Route the LF output to the LF generator."""
        self.write("LF:GEN")

    def lf_monitor(self, source: str, on: bool | None = None):
        """Route a modulation source to the LF output (AM1S/AM2S/AMD/ANG1S/ANG2S/ANGD/OFF)."""
        cmd = f"LF:MON {_choice(source, LF_MONITOR, 'source')}"
        if on is not None:
            cmd += ";ON" if on else ";OFF"
        self.write(cmd)

    def lf_info(self) -> dict:
        resp = self.query("LF?")
        p = _parse(resp)
        fl = _flags(p)
        mon = _get(p, "MON")
        return {"mode": "MON" if mon else "GEN", "source": mon, "on": "ON" in fl}

    def set_lf_frequency(self, hz: float, step_hz: float | None = None,
                         waveform: str | None = None):
        cmd = f"LFGF:VALUE {_n(hz)}HZ"
        if step_hz is not None:
            cmd += f";INC {_n(step_hz)}HZ"
        if waveform is not None:
            cmd += ";" + _choice(waveform, WAVEFORMS, "waveform")
        self.write(cmd)

    def lf_frequency(self) -> float:
        return float(_get(self._query_pairs("LFGF?"), "VALUE"))

    def set_lf_level(self, level: float, units: str = "MV", step_db: float | None = None):
        """LF generator level; ``units`` one of V, MV, UV, DBMV."""
        cmd = f"LFGL:VALUE {_n(level)}{_choice(units, LF_LEVEL_SUFFIXES, 'units')}"
        if step_db is not None:
            cmd += f";INC {_n(step_db)}DB"
        self.write(cmd)

    def set_lf_units(self, units: str):
        self.write(f"LFGL:UNITS {_choice(units, LF_UNITS, 'units')}")

    def lf_level_info(self) -> dict:
        """``{'units','value','step'}`` (value in the LF generator's current units)."""
        p = self._query_pairs("LFGL?")
        return {"units": _get(p, "UNITS"), "value": float(_get(p, "VALUE")),
                "step": float(_get(p, "INC"))}

    def lf_level(self) -> float:
        return self.lf_level_info()["value"]

    def lf_step(self, what: str, direction: str, xfer: bool = True):
        """Step the LF generator ``what`` ('FREQ' or 'LEVEL') UP/DN/RETN."""
        hdr = {"FREQ": "LFGF", "LEVEL": "LFGL"}[_choice(what, ("FREQ", "LEVEL"), "what")]
        d = _choice(direction, ("UP", "DN", "RETN"), "direction")
        self.write(f"{hdr}:{d}" + (";XFER" if xfer and d != "RETN" else ""))

    # -- memory --
    def store(self, kind: str, index: int):
        """Store to memory. ``kind``: full/part (0-49), cfrq (0-99), seqt/sweep (0-19)."""
        hdr, n = self._store(kind, index)
        self.write(f"STO:{hdr} {int(index)}")

    def recall(self, kind: str, index: int, keep_carrier: bool = False):
        """Recall from memory. ``keep_carrier`` uses FXCF/PXCF (recall without carrier frequency)
        for kind full/part."""
        hdr, n = self._store(kind, index)
        if keep_carrier:
            if hdr not in ("FULL", "PART"):
                raise ValueError("keep_carrier only applies to 'full' and 'part'")
            hdr = "FXCF" if hdr == "FULL" else "PXCF"
        self.write(f"RCL:{hdr} {int(index)}")

    def erase(self, kind: str = "all"):
        """Erase all stores of ``kind`` (full/part/cfrq/seqt/sweep) or everything with 'all'."""
        k = str(kind).lower()
        if k == "all":
            self.write("ERASE:ALL")
        elif k in _STORES:
            self.write(f"ERASE:{_STORES[k][0]}")
        else:
            raise ValueError(f"kind must be 'all' or one of {', '.join(_STORES)}")

    @staticmethod
    def _store(kind, index):
        k = str(kind).lower()
        if k not in _STORES:
            raise ValueError(f"kind must be one of {', '.join(_STORES)}")
        hdr, mx = _STORES[k]
        if not 0 <= int(index) <= mx:
            raise ValueError(f"{k} store index must be 0..{mx}")
        return hdr, mx

    # -- sweep --
    @staticmethod
    def _sweep_param(param: str) -> str:
        p = str(param).upper()
        if p.startswith("INTF"):
            p = "INTF"
        if p not in _SWEEP_UNITS:
            raise ValueError("sweep parameter must be CFRQ, RFLV, LFGF, LFGL or INTF")
        return p

    def set_sweep(self, param: str, start, stop, steps: int, time_ms: float,
                  units: str | None = None):
        """Configure a sweep's start/stop/steps/time-per-step.

        ``param``: 'CFRQ' (Hz), 'RFLV' (dBm by default), 'LFGF' (Hz), 'LFGL' (V by
        default) or 'INTF' (modulation oscillator, Hz). ``units`` overrides the
        suffix applied to start/stop. Remember to select the sweep type/mode and
        ``imode('SWEEPER')``.
        """
        p = self._sweep_param(param)
        u = (units or _SWEEP_UNITS[p]).upper()
        self.write(f"SWEEP:{p}:START {_n(start)}{u};STOP {_n(stop)}{u};"
                   f"STEP {int(steps)};TIME {_n(time_ms)}MS")

    def sweep_info(self, param: str) -> dict:
        """``{'start','stop','steps','time_ms'}`` for a sweep parameter."""
        p = self._sweep_param(param)
        pr = self._query_pairs(f"SWEEP:{p}?")
        return {"start": float(_get(pr, "START")), "stop": float(_get(pr, "STOP")),
                "steps": int(float(_get(pr, "STEP"))), "time_ms": float(_get(pr, "TIME"))}

    def set_sweep_marker(self, param: str, number: int, value, on: bool = True,
                         units: str | None = None):
        """Select marker ``number`` and set its value; ``on`` turns it on or off."""
        p = self._sweep_param(param)
        u = (units or _SWEEP_UNITS[p]).upper()
        self.write(f"SWEEP:{p}:MKRNUM {int(number)};VALUE {_n(value)}{u};"
                   f"{'MKRON' if on else 'MKROFF'}")

    def sweep_marker_info(self, param: str) -> dict:
        """Current marker: ``{'number','value','on'}``."""
        p = self._sweep_param(param)
        pr = self._query_pairs(f"SWEEP:{p}:VALUE?")
        return {"number": int(float(_get(pr, "MKRNUM"))), "value": float(_get(pr, "VALUE")),
                "on": "MKRON" in _flags(pr)}

    def sweep_markers(self, enable: bool):
        """Enable/disable sweep markers globally."""
        self.write("SWEEP:MKRON" if enable else "SWEEP:MKROFF")

    def set_sweep_mode(self, mode: str | None = None, sweep_type: str | None = None):
        """Select sweep mode (SNGL/CONT/EXT) and/or sweep type (OFF/CFRQ/RFLV/LFGF/LFGL/INTF1-6/HOP)."""
        parts = []
        if mode is not None:
            parts.append(f"SWEEP:MODE {_choice(mode, SWEEP_MODES, 'mode')}")
        if sweep_type is not None:
            t = _choice(sweep_type, SWEEP_TYPES, "sweep_type")
            parts.append(f"TYPE {t}" if parts else f"SWEEP:TYPE {t}")
        if parts:
            self.write(";".join(parts))

    def sweep_status(self) -> dict:
        """``{'mode','type','markers'}``."""
        p = self._query_pairs("SWEEP?")
        return {"mode": _get(p, "MODE"), "type": _get(p, "TYPE"),
                "markers": "MKRON" in _flags(p)}

    # sweep control
    def sweep_go(self):
        self.write("SWEEP:GO")

    def sweep_calc(self):
        """Initiate pre-calculation of the sweep table."""
        self.write("SWEEP:CALC")

    def sweep_halt(self):
        self.write("SWEEP:HALT")

    def sweep_continue(self):
        self.write("SWEEP:CONT")

    def sweep_reset(self):
        self.write("SWEEP:RESET")

    def sweep_xfer(self):
        """Transfer the paused sweep value to the main parameter."""
        self.write("SWEEP:XFER")

    def sweep_step_up(self):
        """One sweep step up while paused."""
        self.write("SWEEP:UP")

    def sweep_step_down(self):
        self.write("SWEEP:DN")

    def sweep_in_progress(self) -> bool:
        """Instrument condition register bit 0 (``SCR?``)."""
        return bool(self.scr() & 0x01)

    def run_sweep(self, timeout: float = 60.0, poll: float = 0.05) -> bool:
        """Start a single sweep and wait for 'end of sweep'.

        Uses the latched instrument event register (``SSR?`` bit 0), which is
        cleared first, so a fast sweep can't be missed. Returns True on completion,
        False on timeout. Only meaningful in single-sweep mode.
        """
        self.ssr()                       # clear any stale event bits
        self.sweep_go()
        return self.wait_sweep(timeout, poll)

    def wait_sweep(self, timeout: float = 60.0, poll: float = 0.05) -> bool:
        """Wait for the 'end of sweep' event. Clears the event register when it fires."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.ssr() & 0x01:
                return True
            time.sleep(poll)
        return False

    # frequency hopping
    def hop_sequence(self, stores, replace: bool = True):
        """Load a hopping sequence: carrier-frequency store numbers (0-99).

        ``replace=True`` starts with 255 (clears the existing sequence); otherwise the
        numbers are appended. Long sequences are sent in chunks that stay below the
        256-character input buffer.
        """
        nums = [int(s) for s in stores]
        if any(not 0 <= s <= 99 for s in nums):
            raise ValueError("hop sequence entries must be carrier store numbers 0..99")
        chunk, chunks, length = [], [], len("HOPSEQ ") + (4 if replace else 0)
        for s in nums:
            add = len(str(s)) + 1
            if length + add > 200 and chunk:
                chunks.append(chunk)
                chunk, length = [], len("HOPSEQ ")
            chunk.append(s)
            length += add
        if chunk or replace:
            chunks.append(chunk)
        for i, c in enumerate(chunks):
            items = ([255] if (replace and i == 0) else []) + c
            if items:
                self.write("HOPSEQ " + ",".join(str(x) for x in items))

    def hop_length(self) -> int:
        """Number of steps in the hopping sequence (``HOPSEQ?``)."""
        return self.query_int("HOPSEQ?")

    def set_hop_time(self, ms: float):
        self.write(f"SWEEP:HOP:TIME {_n(ms)}MS")

    def hop_time(self) -> float:
        m = re.search(r"TIME\s*(-?[\d.]+)", self.query("SWEEP:HOP?"), re.I)
        if not m:
            raise GpibError("could not parse SWEEP:HOP? reply")
        return float(m.group(1))

    # -- miscellaneous --
    def set_frequency_standard(self, std: str):
        """INT0/INT1/INT5/INT10 (internal; output) or EXT1/EXT5/EXT10 (external reference, MHz)."""
        self.write(f"FSTD {_choice(std, FREQ_STANDARDS, 'standard')}")

    def frequency_standard(self) -> str:
        return self.query("FSTD?").strip().split()[-1]

    def blank(self, which: int):
        """Toggle blanking: 0 carrier freq, 1 RF level, 2 mod freq, 3 modulation, 4 all."""
        if int(which) not in range(5):
            raise ValueError("which must be 0..4")
        self.write(f"BLANK {int(which)}")

    def backlight(self, on: bool):
        self.write(f"BACKL:{'ON' if on else 'OFF'}")

    def time(self) -> str:
        """Real-time clock, ``'HH:MM'``."""
        return self.query("TIME?").strip().strip("\"'")

    def date(self) -> str:
        """Real-time clock date, ``'YYYY-MM-DD'``."""
        return self.query("DATE?").strip().strip("\"'")

    def operating_hours(self) -> float:
        return self.query_float("OPER?")

    def elapsed_hours(self) -> float:
        return self.query_float("ELAPSED?")

    def device_trigger(self, function: str):
        """Action on ``*TRG``: SEQT, FLSWP, SSSWP or VOID."""
        self.write(f"DEVTRG {_choice(function, TRIGGER_FUNCS, 'function')}")

    def external_trigger(self, function: str):
        """Action on the external trigger socket: SEQT, FLSWP, SSSWP, MEMUP, MEMDN or VOID."""
        self.write(f"EXTTRG {_choice(function, EXT_TRIGGER_FUNCS, 'function')}")

    def lock_keyboard(self):
        """``KLOCK`` - disable the front panel (except RPP reset and Local)."""
        self.write("KLOCK")

    def unlock_keyboard(self):
        self.write("KUNLOCK")

    # -- status reporting --
    def stb(self) -> int:
        """``*STB?`` - status byte (not cleared by reading)."""
        return self.query_int("*STB?")

    def sre(self, value: int | None = None) -> int | None:
        """Write (``sre(mask)``) or read (``sre()``) the service request enable register."""
        if value is None:
            return self.query_int("*SRE?")
        self.write(f"*SRE {int(value)}")

    def esr(self) -> int:
        """``*ESR?`` - standard event status register (cleared by reading)."""
        return self.query_int("*ESR?")

    def ese(self, value: int | None = None) -> int | None:
        if value is None:
            return self.query_int("*ESE?")
        self.write(f"*ESE {int(value)}")

    def hcr(self) -> int:
        """Hardware condition register."""
        return self.query_int("HCR?")

    def hsr(self) -> int:
        """Hardware status (event) register (cleared by reading)."""
        return self.query_int("HSR?")

    def hse(self, value: int | None = None) -> int | None:
        if value is None:
            return self.query_int("HSE?")
        self.write(f"HSE {int(value)}")

    def ccr(self) -> int:
        """Coupling condition register."""
        return self.query_int("CCR?")

    def csr(self) -> int:
        """Coupling status (event) register (cleared by reading)."""
        return self.query_int("CSR?")

    def cse(self, value: int | None = None) -> int | None:
        if value is None:
            return self.query_int("CSE?")
        self.write(f"CSE {int(value)}")

    def scr(self) -> int:
        """Instrument condition register (bit0 sweeping, bit1 sending tones,
        bit2 selfcal, bit3 DC FM null)."""
        return self.query_int("SCR?")

    def ssr(self) -> int:
        """Instrument status (event) register (cleared by reading; bit0 = end of sweep)."""
        return self.query_int("SSR?")

    def sse(self, value: int | None = None) -> int | None:
        if value is None:
            return self.query_int("SSE?")
        self.write(f"SSE {int(value)}")

    # decoders
    def status_byte_flags(self) -> list[str]:
        return _bits(self.stb(), STATUS_BYTE_BITS)

    def standard_event_flags(self) -> list[str]:
        """Decode (and clear) the standard event status register."""
        return _bits(self.esr(), ESR_BITS)

    def hardware_flags(self) -> list[str]:
        """Decode the hardware *condition* register (current state, not latched)."""
        return _bits(self.hcr(), HARDWARE_BITS)

    def coupling_flags(self) -> list[str]:
        """Decode the coupling *condition* register."""
        return _bits(self.ccr(), COUPLING_BITS)
