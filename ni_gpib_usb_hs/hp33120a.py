# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2026 Vinay Shanbhag.  Part of ni-gpib-usb-hs (see LICENSE).
"""A convenience wrapper for the HP/Agilent 33120A 15 MHz function / arbitrary
waveform generator.

Drives the generator over GPIB through :class:`NIUSBGPIB`. Command coverage
follows the 33120A User's Guide, chapter 4 (SCPI Command Summary).

    from ni_gpib_usb_hs import HP33120A
    with HP33120A(addr=10) as fg:
        print(fg.idn())
        fg.sine(1e3, 2.0, 0.5)             # APPLy:SINusoid 1 kHz, 2 Vpp, +0.5 V
        fg.output_load("INF")              # high-Z load instead of 50 ohm
        fg.configure_burst(ncycles=3, rate=100)
        fg.burst(True)

Notes
-----
* Amplitude is interpreted in the unit selected with :meth:`voltage_unit`
  (default Vpp), and amplitude/offset values assume the termination set with
  :meth:`output_load` (default 50 ohm).
* The 33120A has a single output, so there is no ``OUTPut ON/OFF``. To silence
  it, use :meth:`dc` with 0 V offset (or :meth:`noise`/low amplitude).
* Only one modulation mode (AM, FM, burst, FSK, sweep) can be on at a time;
  enabling one turns the previous one off.
* Methods named after a setting (``frequency``, ``amplitude`` ...) set the
  value when given an argument and query it when called with none.
"""
from __future__ import annotations

import re
from typing import Iterable, Sequence

from .controller import NIUSBGPIB, GpibError

_SHAPES = {
    "sine": "SINusoid", "sin": "SINusoid", "sinusoid": "SINusoid",
    "square": "SQUare", "squ": "SQUare",
    "triangle": "TRIangle", "tri": "TRIangle",
    "ramp": "RAMP",
    "noise": "NOISe", "nois": "NOISe",
    "dc": "DC",
    "user": "USER", "arb": "USER",
}

BUILTIN_ARBS = ("SINC", "NEG_RAMP", "EXP_RISE", "EXP_FALL", "CARDIAC")

_ARB_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,7}$")


def _num(v) -> str:
    """Format a SCPI numeric parameter; strings (MIN/MAX/DEF/INF...) pass through."""
    if isinstance(v, str):
        return v
    return f"{float(v):.12g}"


def _onoff(v) -> str:
    if isinstance(v, str):
        return v.upper()
    return "ON" if v else "OFF"


def _arb_name(name: str) -> str:
    """Validate an arb-waveform name (<= 8 chars, letter first, then A-Z 0-9 _)."""
    if name.upper() == "VOLATILE":
        return "VOLATILE"
    if not _ARB_NAME_RE.match(name):
        raise ValueError(
            f"invalid arb name {name!r}: max 8 chars, first a letter, "
            "then letters/digits/underscore")
    return name.upper()


class HP33120A:
    """33120A function / arbitrary waveform generator over GPIB.

    Args:
        addr: the instrument's GPIB primary address (factory default 10; set
            from the front panel only).
        gpib: an existing :class:`NIUSBGPIB` to share; if None, one is opened.
        controller_pad: controller GPIB address when opening our own bus.
        reset: send ``*RST`` + ``*CLS`` on connect (default True).
    """

    def __init__(self, addr: int = 10, gpib: NIUSBGPIB | None = None,
                 controller_pad: int = 0, reset: bool = True):
        self.addr = addr
        self._own_gpib = gpib is None
        self.gpib = gpib if gpib is not None else NIUSBGPIB(my_pad=controller_pad)
        if reset:
            self.reset()
        # Note: SYSTem:REMote / SYSTem:LOCal / SYSTem:RWLock are RS-232-only on
        # the 33120A; over GPIB, addressing the instrument is enough, so they
        # are intentionally not exposed here.

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

    # -- raw SCPI --
    def write(self, cmd: str):
        """Send a SCPI command (no reply expected)."""
        self.gpib.write(self.addr, cmd)

    def query(self, cmd: str) -> str:
        """Send a SCPI query and return the stripped string reply."""
        return self.gpib.query(self.addr, cmd)

    def query_float(self, cmd: str) -> float:
        s = self.query(cmd)
        try:
            return float(s)
        except ValueError as e:
            raise GpibError(f"expected a number from {cmd!r}, got {s!r}") from e

    def query_int(self, cmd: str) -> int:
        return int(round(self.query_float(cmd)))

    def query_bool(self, cmd: str) -> bool:
        return self.query_int(cmd) != 0

    def query_str(self, cmd: str) -> str:
        """Query and strip the surrounding quotes some replies carry."""
        return self.query(cmd).strip().strip('"')

    # internal helpers: set a numeric/keyword setting, or query it (optionally
    # with a MINimum / MAXimum limit)
    def _setting(self, node: str, value=None, limit: str | None = None):
        if value is None:
            return self.query_float(f"{node}?" + (f" {limit}" if limit else ""))
        self.write(f"{node} {_num(value)}")

    def _keyword(self, node: str, value=None) -> str | None:
        if value is None:
            return self.query_str(f"{node}?")
        self.write(f"{node} {value}")
        return None

    def _state(self, node: str, on=None):
        if on is None:
            return self.query_bool(f"{node}?")
        self.write(f"{node} {_onoff(on)}")

    # -- common / system commands --
    def idn(self) -> str:
        """``*IDN?`` -> e.g. ``HEWLETT-PACKARD,33120A,0,X.X-X.X-X.X``."""
        return self.query("*IDN?")

    def reset(self):
        """``*RST`` + ``*CLS`` (``*RST`` alone does not clear the error queue)."""
        self.write("*RST")
        self.write("*CLS")

    def clear(self):
        """Clear status / error queue (``*CLS``)."""
        self.write("*CLS")

    def errors(self):
        """Drain and return the error queue as a list of ``(code, text)``.

        The queue holds up to 20 errors; ``+0,"No error"`` ends the drain.
        """
        out = []
        for _ in range(32):
            resp = self.query("SYSTem:ERRor?")
            code_str, _, text = resp.partition(",")
            try:
                code = int(code_str)
            except ValueError:
                break
            if code == 0:
                break
            out.append((code, text.strip().strip('"')))
        return out

    def check_errors(self):
        """Raise :class:`GpibError` if the error queue is non-empty."""
        errs = self.errors()
        if errs:
            raise GpibError("; ".join(f"{c}: {t}" for c, t in errs))

    def selftest(self) -> bool:
        """``*TST?`` -> True if the complete self-test passes (takes a few seconds)."""
        return self.query_int("*TST?") == 0

    def scpi_version(self) -> str:
        """``SYSTem:VERSion?`` (e.g. ``1993.0``)."""
        return self.query_str("SYSTem:VERSion?")

    def beep(self):
        """Sound the beeper once."""
        self.write("SYSTem:BEEPer")

    def display(self, on=None):
        """Turn the front-panel display on/off, or query its state."""
        return self._state("DISPlay", on)

    def display_text(self, text: str | None = None):
        """Show ``text`` on the front panel (<= 11 chars visible), or query it."""
        if text is None:
            return self.query_str("DISPlay:TEXT?")
        self.write('DISPlay:TEXT "%s"' % text.replace('"', "'"))

    def display_text_clear(self):
        self.write("DISPlay:TEXT:CLEar")

    def save_state(self, slot: int):
        """``*SAV`` to slot 0-3 (slot 0 is overwritten at power-down)."""
        self._check_slot(slot)
        self.write(f"*SAV {slot}")

    def recall_state(self, slot: int):
        """``*RCL`` slot 0-3 (error +810 if nothing was stored there)."""
        self._check_slot(slot)
        self.write(f"*RCL {slot}")

    def delete_state(self, slot: int):
        """``MEMory:STATe:DELete`` slot 0-3."""
        self._check_slot(slot)
        self.write(f"MEMory:STATe:DELete {slot}")

    @staticmethod
    def _check_slot(slot: int):
        if slot not in (0, 1, 2, 3):
            raise ValueError("state slot must be 0, 1, 2 or 3")

    # -- APPLy: function + frequency + amplitude + offset in one command --
    def apply(self, shape: str, frequency=None, amplitude=None, offset=None):
        """Output ``shape`` via ``APPLy:<shape>``.

        Parameters are positional-optional on the instrument: to give
        ``amplitude`` you must give ``frequency``, and to give ``offset`` you
        must give both. Any of them may be ``"MIN"``, ``"MAX"`` or ``"DEF"``.
        """
        node = _SHAPES.get(str(shape).lower())
        if node is None:
            raise ValueError(f"unknown shape {shape!r}; use one of {sorted(set(_SHAPES))}")
        if frequency is None and (amplitude is not None or offset is not None):
            raise ValueError("APPLy needs frequency to set amplitude/offset")
        if amplitude is None and offset is not None:
            raise ValueError("APPLy needs amplitude to set offset")
        args = [a for a in (frequency, amplitude, offset) if a is not None]
        cmd = f"APPLy:{node}"
        if args:
            cmd += " " + ",".join(_num(a) for a in args)
        self.write(cmd)

    def sine(self, frequency=None, amplitude=None, offset=None):
        self.apply("sine", frequency, amplitude, offset)

    def square(self, frequency=None, amplitude=None, offset=None):
        self.apply("square", frequency, amplitude, offset)

    def triangle(self, frequency=None, amplitude=None, offset=None):
        self.apply("triangle", frequency, amplitude, offset)

    def ramp(self, frequency=None, amplitude=None, offset=None):
        self.apply("ramp", frequency, amplitude, offset)

    def noise(self, amplitude="DEF", offset=None):
        """Noise output. (Frequency is ignored by the instrument but required.)"""
        self.apply("noise", "DEF", amplitude, offset)

    def dc(self, volts: float):
        """Output a DC level (set through the offset): +-5 V into 50 ohm, +-10 V open."""
        self.apply("dc", "DEF", "DEF", volts)

    def user(self, frequency=None, amplitude=None, offset=None):
        """Output the arb selected by :meth:`select_arb`."""
        self.apply("user", frequency, amplitude, offset)

    def apply_query(self) -> dict:
        """``APPLy?`` -> ``{"shape", "frequency", "amplitude", "offset"}``."""
        s = self.query_str("APPLy?")
        shape, _, rest = s.partition(" ")
        try:
            f, a, o = (float(x) for x in rest.split(","))
        except ValueError as e:
            raise GpibError(f"unexpected APPLy? reply {s!r}") from e
        return {"shape": shape, "frequency": f, "amplitude": a, "offset": o}

    # -- output configuration (low-level) --
    def shape(self, name: str | None = None):
        """Set (SIN/SQU/TRI/RAMP/NOIS/DC/USER) or query the output function."""
        if name is None:
            return self.query_str("FUNCtion:SHAPe?")
        node = _SHAPES.get(str(name).lower())
        if node is None:
            raise ValueError(f"unknown shape {name!r}")
        self.write(f"FUNCtion:SHAPe {node}")

    def frequency(self, value=None, limit: str | None = None):
        """Set output frequency in Hz, or query it (``limit`` = "MIN"/"MAX")."""
        return self._setting("FREQuency", value, limit)

    def amplitude(self, value=None, limit: str | None = None):
        """Set output amplitude (in :meth:`voltage_unit` units), or query it."""
        return self._setting("VOLTage", value, limit)

    def offset(self, value=None, limit: str | None = None):
        """Set DC offset in volts, or query it."""
        return self._setting("VOLTage:OFFSet", value, limit)

    def duty_cycle(self, value=None, limit: str | None = None):
        """Square-wave duty cycle in percent (20-80, or 40-60 above 5 MHz)."""
        return self._setting("PULSe:DCYCle", value, limit)

    def voltage_unit(self, unit: str | None = None):
        """Amplitude unit: ``"VPP"``, ``"VRMS"``, ``"DBM"`` or ``"DEF"``; or query."""
        return self._keyword("VOLTage:UNIT", unit.upper() if unit else None)

    def output_load(self, load=None, limit: str | None = None):
        """Output termination: ``50``, ``"INF"`` (high-Z), ``"MIN"``, ``"MAX"``.

        Queries return ``50`` or ``9.9E+37`` (high-Z). Changing the load rescales
        the programmed amplitude/offset without raising an error.
        """
        return self._setting("OUTPut:LOAD", load, limit)

    def sync(self, on=None):
        """Enable/disable (or query) the rear-panel SYNC output."""
        return self._state("OUTPut:SYNC", on)

    # -- AM --
    def configure_am(self, depth=None, function=None, frequency=None, source=None):
        """Set up internal AM; call :meth:`am` ``(True)`` afterwards to enable.

        depth: 0-120 %; function: SIN/SQU/TRI/RAMP/NOIS/USER; frequency: 10 mHz-20 kHz;
        source: ``"BOTH"`` (internal+external) or ``"EXT"``.
        """
        if function is not None:
            self.write(f"AM:INTernal:FUNCtion {_SHAPES[function.lower()]}")
        if frequency is not None:
            self.write(f"AM:INTernal:FREQuency {_num(frequency)}")
        if depth is not None:
            self.write(f"AM:DEPTh {_num(depth)}")
        if source is not None:
            self.write(f"AM:SOURce {source}")

    def am(self, on=None):
        return self._state("AM:STATe", on)

    def am_depth(self, value=None, limit=None):
        return self._setting("AM:DEPTh", value, limit)

    def am_frequency(self, value=None, limit=None):
        return self._setting("AM:INTernal:FREQuency", value, limit)

    def am_function(self, name=None):
        return self._keyword("AM:INTernal:FUNCtion", _SHAPES[name.lower()] if name else None)

    def am_source(self, source=None):
        return self._keyword("AM:SOURce", source)

    # -- FM --
    def configure_fm(self, deviation=None, function=None, frequency=None):
        """Set up internal FM; call :meth:`fm` ``(True)`` afterwards to enable.

        deviation: peak deviation 10 mHz-7.5 MHz (carrier must be >= deviation);
        function: SIN/SQU/TRI/RAMP/NOIS/USER; frequency: 10 mHz-10 kHz.
        """
        if function is not None:
            self.write(f"FM:INTernal:FUNCtion {_SHAPES[function.lower()]}")
        if frequency is not None:
            self.write(f"FM:INTernal:FREQuency {_num(frequency)}")
        if deviation is not None:
            self.write(f"FM:DEViation {_num(deviation)}")

    def fm(self, on=None):
        return self._state("FM:STATe", on)

    def fm_deviation(self, value=None, limit=None):
        return self._setting("FM:DEViation", value, limit)

    def fm_frequency(self, value=None, limit=None):
        return self._setting("FM:INTernal:FREQuency", value, limit)

    def fm_function(self, name=None):
        return self._keyword("FM:INTernal:FUNCtion", _SHAPES[name.lower()] if name else None)

    # -- burst --
    def configure_burst(self, ncycles=None, phase=None, rate=None,
                        source=None, trigger=None):
        """Set up burst modulation; call :meth:`burst` ``(True)`` afterwards.

        ncycles: 1-50000 (or "INF"); phase: -360..+360 deg; rate: internal burst
        rate 10 mHz-50 kHz; source: ``"INT"`` or ``"EXT"`` (external *gated*
        mode); trigger: ``"IMM"``, ``"EXT"`` or ``"BUS"`` (triggered mode).
        """
        if ncycles is not None:
            self.write(f"BM:NCYCles {_num(ncycles)}")
        if phase is not None:
            self.write(f"BM:PHASe {_num(phase)}")
        if rate is not None:
            self.write(f"BM:INTernal:RATE {_num(rate)}")
        if source is not None:
            self.write(f"BM:SOURce {source}")
        if trigger is not None:
            self.trigger_source(trigger)

    def burst(self, on=None):
        return self._state("BM:STATe", on)

    def burst_ncycles(self, value=None, limit=None):
        return self._setting("BM:NCYCles", value, limit)

    def burst_phase(self, value=None, limit=None):
        return self._setting("BM:PHASe", value, limit)

    def burst_rate(self, value=None, limit=None):
        return self._setting("BM:INTernal:RATE", value, limit)

    def burst_source(self, source=None):
        return self._keyword("BM:SOURce", source)

    # -- FSK --
    def configure_fsk(self, hop_frequency=None, rate=None, source=None):
        """Set up FSK; call :meth:`fsk` ``(True)`` afterwards to enable.

        The output alternates between the carrier (:meth:`frequency`) and
        ``hop_frequency`` at ``rate`` Hz (10 mHz-50 kHz) for ``source="INT"``,
        or follows the rear-panel FSK terminal for ``"EXT"``.
        """
        if hop_frequency is not None:
            self.write(f"FSKey:FREQuency {_num(hop_frequency)}")
        if rate is not None:
            self.write(f"FSKey:INTernal:RATE {_num(rate)}")
        if source is not None:
            self.write(f"FSKey:SOURce {source}")

    def fsk(self, on=None):
        return self._state("FSKey:STATe", on)

    def fsk_frequency(self, value=None, limit=None):
        return self._setting("FSKey:FREQuency", value, limit)

    def fsk_rate(self, value=None, limit=None):
        return self._setting("FSKey:INTernal:RATE", value, limit)

    def fsk_source(self, source=None):
        return self._keyword("FSKey:SOURce", source)

    # -- frequency sweep --
    def configure_sweep(self, start=None, stop=None, time=None,
                        spacing=None, trigger=None):
        """Set up a frequency sweep; call :meth:`sweep` ``(True)`` afterwards.

        start/stop: 10 mHz-15 MHz (100 kHz for triangle/ramp; start > stop
        sweeps down); time: 1 ms-500 s; spacing: ``"LIN"`` or ``"LOG"``;
        trigger: ``"IMM"``, ``"EXT"`` or ``"BUS"``.
        """
        if start is not None:
            self.write(f"FREQuency:STARt {_num(start)}")
        if stop is not None:
            self.write(f"FREQuency:STOP {_num(stop)}")
        if time is not None:
            self.write(f"SWEep:TIME {_num(time)}")
        if spacing is not None:
            self.write(f"SWEep:SPACing {spacing}")
        if trigger is not None:
            self.trigger_source(trigger)

    def sweep(self, on=None):
        return self._state("SWEep:STATe", on)

    def sweep_start(self, value=None, limit=None):
        return self._setting("FREQuency:STARt", value, limit)

    def sweep_stop(self, value=None, limit=None):
        return self._setting("FREQuency:STOP", value, limit)

    def sweep_time(self, value=None, limit=None):
        return self._setting("SWEep:TIME", value, limit)

    def sweep_spacing(self, spacing=None):
        return self._keyword("SWEep:SPACing", spacing)

    # -- triggering (burst and sweep only) --
    def trigger_source(self, source: str | None = None):
        """``TRIGger:SOURce`` ``IMM`` | ``EXT`` | ``BUS``, or query."""
        return self._keyword("TRIGger:SOURce", source.upper() if source else None)

    def trigger_slope(self, slope: str | None = None):
        """``TRIGger:SLOPe`` ``POS`` | ``NEG`` for the Ext Trig input, or query."""
        return self._keyword("TRIGger:SLOPe", slope.upper() if slope else None)

    def trigger(self):
        """Software trigger (``*TRG``); needs ``trigger_source("BUS")``."""
        self.write("*TRG")

    def wait(self):
        """``*WAI``: hold off later commands until pending operations finish."""
        self.write("*WAI")

    def wait_complete(self) -> bool:
        """``*OPC?`` -> True once a triggered burst/sweep has completed.

        Per the manual this is only meaningful in triggered burst/sweep mode.
        """
        return self.query_int("*OPC?") == 1

    # -- arbitrary waveforms --
    def select_arb(self, name: str | None = None):
        """Select (``FUNCtion:USER``) the active arb, or query it.

        Does not change the output; use :meth:`user` or ``shape("USER")``.
        Built-ins: SINC, NEG_RAMP, EXP_RISE, EXP_FALL, CARDIAC; or a stored
        user name, or ``"VOLATILE"``.
        """
        if name is None:
            return self.query_str("FUNCtion:USER?")
        self.write(f"FUNCtion:USER {_arb_name(name)}")

    def upload_arb(self, points: Iterable[float]):
        """Download 8-16000 floats in [-1, +1] to volatile memory (``DATA VOLATILE``).

        Overwrites whatever was in volatile memory. Persist it with
        :meth:`copy_arb`, and play it with ``select_arb("VOLATILE")`` + :meth:`user`.
        """
        vals = [float(p) for p in points]
        self._check_arb_len(len(vals))
        if any(abs(v) > 1.0 for v in vals):
            raise ValueError("arb points must be within [-1, +1]")
        self.write("DATA VOLATILE," + ",".join(f"{v:.6g}" for v in vals))

    def upload_arb_dac(self, points: Sequence[int]):
        """Download 8-16000 ints in [-2047, +2047] (12-bit DAC codes) as a text list.

        Faster than the float form. (The IEEE-488.2 binary-block variant is
        not used here; the text list form works on any byte-oriented link.)
        """
        vals = [int(p) for p in points]
        self._check_arb_len(len(vals))
        if any(abs(v) > 2047 for v in vals):
            raise ValueError("DAC codes must be within [-2047, +2047]")
        self.write("DATA:DAC VOLATILE," + ",".join(str(v) for v in vals))

    @staticmethod
    def _check_arb_len(n: int):
        if not 8 <= n <= 16000:
            raise ValueError(f"arb waveform needs 8..16000 points, got {n}")

    def copy_arb(self, name: str):
        """Copy volatile memory to non-volatile storage as ``name`` (max 4 user arbs)."""
        name = _arb_name(name)
        if name == "VOLATILE" or name in BUILTIN_ARBS:
            raise ValueError(f"cannot copy to {name}")
        self.write(f"DATA:COPY {name}")

    def delete_arb(self, name: str):
        """Delete VOLATILE or a user arb (built-ins and the active arb are refused)."""
        self.write(f"DATA:DELete {_arb_name(name)}")

    def delete_all_arbs(self):
        """Delete volatile and all user arbs (built-ins stay)."""
        self.write("DATA:DELete:ALL")

    @staticmethod
    def _split_names(s: str) -> list[str]:
        return [n.strip().strip('"') for n in s.split(",") if n.strip().strip('"')]

    def arb_catalog(self) -> list[str]:
        """All arb names: built-ins, VOLATILE (if loaded), and user arbs."""
        return self._split_names(self.query("DATA:CATalog?"))

    def arb_user_catalog(self) -> list[str]:
        """User arbs in non-volatile memory (empty list if none)."""
        return self._split_names(self.query("DATA:NVOLatile:CATalog?"))

    def arb_free_slots(self) -> int:
        """Free non-volatile arb slots (0-4)."""
        return self.query_int("DATA:NVOLatile:FREE?")

    def _arb_attr(self, attr: str, name: str | None) -> float:
        return self.query_float(f"DATA:ATTRibute:{attr}?" + (f" {_arb_name(name)}" if name else ""))

    def arb_points(self, name: str | None = None) -> int:
        return int(self._arb_attr("POINts", name))

    def arb_average(self, name: str | None = None) -> float:
        return self._arb_attr("AVERage", name)

    def arb_crest_factor(self, name: str | None = None) -> float:
        return self._arb_attr("CFACtor", name)

    def arb_peak_to_peak(self, name: str | None = None) -> float:
        return self._arb_attr("PTPeak", name)

    def byte_order(self, order: str | None = None):
        """``FORMat:BORDer`` ``NORM`` | ``SWAP`` (binary-block arb transfers only)."""
        return self._keyword("FORMat:BORDer", order.upper() if order else None)

    # -- status reporting (IEEE-488.2) --
    def stb(self) -> int:
        """``*STB?`` status byte."""
        return self.query_int("*STB?")

    def esr(self) -> int:
        """``*ESR?`` standard event status register (clears it)."""
        return self.query_int("*ESR?")

    def ese(self, mask: int | None = None):
        """``*ESE`` standard event enable register: set or query."""
        if mask is None:
            return self.query_int("*ESE?")
        self.write(f"*ESE {int(mask)}")

    def sre(self, mask: int | None = None):
        """``*SRE`` service request enable register: set or query."""
        if mask is None:
            return self.query_int("*SRE?")
        self.write(f"*SRE {int(mask)}")

    def psc(self, on: bool | None = None):
        """``*PSC`` power-on status clear (non-volatile): set or query."""
        if on is None:
            return self.query_bool("*PSC?")
        self.write(f"*PSC {1 if on else 0}")

    def opc(self):
        """``*OPC``: set the operation-complete bit when pending ops finish."""
        self.write("*OPC")

    # -- calibration info (read-only; calibration writes are deliberately omitted) --
    def calibration_count(self) -> int:
        """``CALibration:COUNt?`` number of calibrations performed."""
        return self.query_int("CALibration:COUNt?")

    def calibration_string(self) -> str:
        """``CALibration:STRing?`` stored calibration message."""
        return self.query_str("CALibration:STRing?")

    def calibration_secured(self) -> bool:
        """``CALibration:SECure:STATe?`` True if calibration is secured."""
        return self.query_bool("CALibration:SECure:STATe?")
