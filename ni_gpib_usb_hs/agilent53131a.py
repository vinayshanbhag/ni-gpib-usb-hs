# SPDX-License-Identifier: GPL-2.0-only
# Copyright (C) 2026 Vinay Shanbhag.  Part of ni-gpib-usb-hs (see LICENSE).
"""A convenience wrapper for the HP/Agilent/Keysight 53131A 225 MHz universal counter.

Drives the counter over GPIB through :class:`NIUSBGPIB`. Command syntax follows
the 53131A/132A Universal Counter Programming Guide. The 53132A shares the same
command set, so this wrapper works with it too (channel 3 is the optional
microwave input and only exists on units fitted with it).

    from ni_gpib_usb_hs import Agilent53131A
    with Agilent53131A(addr=3) as cnt:
        print(cnt.idn())
        print(cnt.frequency())                   # default autoranged Ch1 frequency
        print(cnt.frequency(10e6, 1e-3, channel=2))
        print(cnt.time_interval())               # Ch1 -> Ch2
        print(cnt.statistics_of("frequency", 50))

Notes
-----
* The factory GPIB address of the counter is 3 (front panel: hold *Recall*
  (Utility) while powering on, then step to ``HP-IB:``).
* ``MEASure`` / ``CONFigure`` default several counter settings (including
  trigger levels -> auto-trigger, and they *disable* math, statistics and limit
  testing). To change only the function and keep other settings, use
  :meth:`set_function`. To keep math/statistics/limits, configure first and
  enable them afterwards.
* Measurements block the counter (and your GPIB read) until they finish. Long gate
  times, very high resolutions or a missing input signal need a long enough
  timeout on the underlying :class:`NIUSBGPIB`.
* A reading of ``9.91E37`` is the SCPI "Not a Number"; it is returned here as
  ``float('nan')`` (the counter also queues error -230 / -221).
"""
from __future__ import annotations

import math

from .controller import NIUSBGPIB, GpibError

# SCPI "Not a Number" used by the 53131A when no valid result exists.
_SCPI_NAN = 9.91e37

# MEASure/CONFigure subsystem nodes (the optional [:VOLTage] prefix is omitted).
_FUNCTIONS = {
    "frequency": "FREQuency",
    "period": "PERiod",
    "ratio": "FREQuency:RATio",
    "time_interval": "TINTerval",
    "phase": "PHASe",
    "duty_cycle": "DCYCle",
    "pos_width": "PWIDth",
    "neg_width": "NWIDth",
    "rise_time": "RISE:TIME",
    "fall_time": "FALL:TIME",
    "vmax": "MAXimum",
    "vmin": "MINimum",
    "vpp": "PTPeak",
    "totalize_timed": "TOTalize:TIMed",
    "totalize_continuous": "TOTalize:CONTinuous",   # CONFigure only
}

# [:SENSe]:FUNCtion[:ON] strings (these keep all other counter settings).
_SENSE_FUNCTIONS = {
    "frequency": "FREQuency",
    "period": "PERiod",
    "ratio": "FREQuency:RATio",
    "time_interval": "TINTerval",
    "phase": "PHASe",
    "duty_cycle": "DCYCle",
    "pos_width": "PWIDth",
    "neg_width": "NWIDth",
    "rise_time": "RISE:TIME",
    "fall_time": "FALL:TIME",
    "vmax": "VOLTage:MAXimum",
    "vmin": "VOLTage:MINimum",
    "vpp": "VOLTage:PTPeak",
    "totalize": "TOTalize",
}


def _num(x) -> str:
    """Format a number for SCPI (``DEF``/strings pass through untouched)."""
    if isinstance(x, str):
        return x
    return f"{x:.12g}"


def _bool(x) -> str:
    return "ON" if x else "OFF"


def _check_channel(ch: int, allowed) -> int:
    if ch not in allowed:
        raise ValueError(f"channel must be one of {tuple(allowed)}, got {ch!r}")
    return int(ch)


def _percent(ref):
    """Reference level for duty cycle / pulse width / rise & fall time.

    ``None`` -> default; a number -> percent; a string is passed through, so use
    e.g. ``"1.5 V"`` for an absolute voltage reference (disables auto-trigger).
    """
    if ref is None:
        return None
    if isinstance(ref, str):
        return ref
    return f"{ref:.12g} PCT"


class Agilent53131A:
    """53131A/53132A universal counter over GPIB.

    Args:
        addr: the counter's GPIB primary address (factory default 3).
        gpib: an existing :class:`NIUSBGPIB` to share; if None, one is opened.
        controller_pad: controller GPIB address when opening our own bus.
        reset: send ``*RST`` + ``*CLS`` on connect (default True).
    """

    def __init__(self, addr: int = 3, gpib: NIUSBGPIB | None = None,
                 controller_pad: int = 0, reset: bool = True):
        self.addr = addr
        self._own_gpib = gpib is None
        self.gpib = gpib if gpib is not None else NIUSBGPIB(my_pad=controller_pad)
        if reset:
            self.reset()

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
            v = float(s)
        except ValueError as e:
            raise GpibError(f"expected a number from {cmd!r}, got {s!r}") from e
        return math.nan if v >= _SCPI_NAN else v

    def query_floats(self, cmd: str) -> list[float]:
        """Query a comma-separated list of numbers."""
        s = self.query(cmd)
        try:
            vals = [float(p) for p in s.split(",")]
        except ValueError as e:
            raise GpibError(f"expected numbers from {cmd!r}, got {s!r}") from e
        return [math.nan if v >= _SCPI_NAN else v for v in vals]

    def query_int(self, cmd: str) -> int:
        s = self.query(cmd)
        try:
            return int(float(s))
        except ValueError as e:
            raise GpibError(f"expected an integer from {cmd!r}, got {s!r}") from e

    # -- common commands --
    def idn(self) -> str:
        return self.query("*IDN?")

    def reset(self):
        """``*RST`` + ``*CLS`` (counter returns to Ch1 frequency, auto-trigger)."""
        self.write("*RST")
        self.write("*CLS")

    def clear(self):
        """Clear status / error queue (``*CLS``)."""
        self.write("*CLS")

    def status_preset(self):
        """``:STATus:PRESet`` — preset the Operation/Questionable enable registers."""
        self.write("STATus:PRESet")

    def wait(self) -> bool:
        """Block until all pending operations are complete (``*OPC?``)."""
        return self.query("*OPC?").strip() == "1"

    def errors(self):
        """Drain and return the SCPI error queue as a list of ``(code, text)``."""
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

    def scpi_version(self) -> str:
        return self.query("SYSTem:VERSion?")

    def display(self, enable: bool = True):
        """Turn the whole front-panel display on or off (``:DISPlay:ENABle``)."""
        self.write(f"DISPlay:ENABle {_bool(enable)}")

    # -- building MEASure / CONFigure arguments --
    @staticmethod
    def _source_list(channels) -> str:
        """``None`` -> '' (default), ``2`` -> '(@2)', ``(1, 2)`` -> '(@1),(@2)'."""
        if channels is None:
            return ""
        if isinstance(channels, int):
            return f"(@{channels})"
        return ",".join(f"(@{c})" for c in channels)

    def _args(self, params, channels) -> str:
        """Join parameters (defaulted from the right, ``None`` in the middle ->
        ``DEF``) and the optional source list."""
        params = list(params)
        while params and params[-1] is None:
            params.pop()
        parts = ["DEF" if p is None else _num(p) for p in params]
        src = self._source_list(channels)
        if src:
            parts.append(src)
        return ",".join(parts)

    def _node(self, function: str) -> str:
        return _FUNCTIONS.get(function, function)

    # -- one-shot measurements (MEASure? — self-configuring) --
    def measure(self, function: str, *params, channels=None) -> float:
        """One-shot reading via ``MEASure:<function>? [params][,(@ch)]``.

        ``function`` is a key of the supported set (e.g. ``"frequency"``) or a
        raw SCPI node like ``"FREQuency"``. ``params`` are the function's
        positional parameters (e.g. expected value, resolution); ``None`` means
        default. ``channels`` is an int or a tuple of ints for the source list.

        Note: ``MEASure`` is documented to be broken on 53131A firmware 3413 for
        max/min/peak-to-peak, so those use :meth:`_measure_peak` instead.
        """
        node = self._node(function)
        args = self._args(params, channels)
        return self.query_float(f"MEASure:{node}?" + (f" {args}" if args else ""))

    def frequency(self, expected=None, resolution=None, channel: int = 1) -> float:
        """Frequency in Hz. ``expected`` (Hz, within 10 %) and ``resolution`` (Hz,
        a power of ten = decade of the last digit) tune the arming digits.
        Ch1/2: 0.1 Hz-225 MHz; Ch3: 100 MHz-3 GHz."""
        _check_channel(channel, (1, 2, 3))
        return self.measure("frequency", expected, resolution, channels=channel)

    def period(self, expected=None, resolution=None, channel: int = 1) -> float:
        """Period in seconds (``expected`` in s, ``resolution`` in s)."""
        _check_channel(channel, (1, 2, 3))
        return self.measure("period", expected, resolution, channels=channel)

    def frequency_ratio(self, num: int = 1, den: int = 2,
                        expected=None, resolution=None) -> float:
        """Ratio of channel ``num`` / channel ``den``: 1/2, 1/3, 2/1 or 3/1."""
        if (num, den) not in ((1, 2), (1, 3), (2, 1), (3, 1)):
            raise ValueError("ratio must be 1/2, 1/3, 2/1 or 3/1")
        return self.measure("ratio", expected, resolution, channels=(num, den))

    def time_interval(self, start: int = 1, stop: int = 2) -> float:
        """Time interval in seconds from the ``start`` to the ``stop`` channel."""
        _check_channel(start, (1, 2))
        _check_channel(stop, (1, 2))
        return self.measure("time_interval", channels=(start, stop))

    def phase(self, a: int = 1, b: int = 2) -> float:
        """Phase of channel ``a`` relative to ``b`` in degrees."""
        _check_channel(a, (1, 2))
        _check_channel(b, (1, 2))
        return self.measure("phase", channels=(a, b))

    def duty_cycle(self, reference=None) -> float:
        """Positive duty cycle on Ch1 (``reference``: percent, default 50, or a
        string such as ``"1.5 V"``)."""
        return self.measure("duty_cycle", _percent(reference), channels=1)

    def pos_width(self, reference=None) -> float:
        """Positive pulse width on Ch1 in seconds."""
        return self.measure("pos_width", _percent(reference), channels=1)

    def neg_width(self, reference=None) -> float:
        """Negative pulse width on Ch1 in seconds."""
        return self.measure("neg_width", _percent(reference), channels=1)

    def rise_time(self, lower=None, upper=None) -> float:
        """Rise time on Ch1 in seconds (reference points default to 10 % / 90 %)."""
        return self.measure("rise_time", _percent(lower), _percent(upper), channels=1)

    def fall_time(self, lower=None, upper=None) -> float:
        """Fall time on Ch1 in seconds (reference points default to 10 % / 90 %)."""
        return self.measure("fall_time", _percent(lower), _percent(upper), channels=1)

    # Voltage peaks. These use CONFigure + INITiate + *WAI + FETCh? rather than
    # MEASure?/READ? so they are safe on 53131A firmware revision 3413 (the guide
    # lists this as the required work-around; it works on every revision).
    def _measure_peak(self, function: str, channel: int) -> float:
        _check_channel(channel, (1, 2))
        node = self._node(function)
        self.write("INITiate:CONTinuous OFF")
        self.write(f"CONFigure:{node} {self._source_list(channel)}")
        self.write("INITiate")
        self.write("*WAI")
        return self.query_float("FETCh?")

    def vmax(self, channel: int = 1) -> float:
        """Maximum input voltage (V)."""
        return self._measure_peak("vmax", channel)

    def vmin(self, channel: int = 1) -> float:
        """Minimum input voltage (V)."""
        return self._measure_peak("vmin", channel)

    def vpp(self, channel: int = 1) -> float:
        """Peak-to-peak input voltage (V)."""
        return self._measure_peak("vpp", channel)

    def totalize_timed(self, gate_time=None, channel: int = 1) -> float:
        """Count events on Ch1 for ``gate_time`` seconds (default 0.1 s).

        Gate time: 1 ms-9.99 ms in 10 us steps, or 10 ms-1000 s in 1 ms steps.
        """
        _check_channel(channel, (1,))
        return self.measure("totalize_timed", gate_time, channels=channel)

    # -- configured / repeated reads (CONFigure, then READ? or INITiate/FETCh?) --
    def configure(self, function: str, *params, channels=None):
        """``CONFigure:<function> [params][,(@ch)]`` — set up, don't measure.

        Defaults several settings and disables math/statistics/limit testing. Use
        :meth:`read` (or :meth:`initiate` + :meth:`fetch`) afterwards, with any
        ``[:SENSe]`` tweaks (trigger level, gate time, ...) in between.
        """
        node = self._node(function)
        args = self._args(params, channels)
        self.write(f"CONFigure:{node}" + (f" {args}" if args else ""))

    def configure_query(self) -> str:
        """``CONFigure?`` — function set by the last CONFigure/MEASure."""
        return self.query("CONFigure?")

    def configure_frequency(self, expected=None, resolution=None, channel: int = 1):
        _check_channel(channel, (1, 2, 3))
        self.configure("frequency", expected, resolution, channels=channel)

    def configure_period(self, expected=None, resolution=None, channel: int = 1):
        _check_channel(channel, (1, 2, 3))
        self.configure("period", expected, resolution, channels=channel)

    def configure_time_interval(self, start: int = 1, stop: int = 2):
        _check_channel(start, (1, 2))
        _check_channel(stop, (1, 2))
        self.configure("time_interval", channels=(start, stop))

    def read(self) -> float:
        """Initiate a fresh measurement and return it (``READ?``)."""
        return self.query_float("READ?")

    def fetch(self) -> float:
        """Return the last measurement without re-acquiring (``FETCh?``).

        Blocks while a measurement is in progress.
        """
        return self.query_float("FETCh?")

    def initiate(self):
        """Start the configured measurement(s) (``INITiate``)."""
        self.write("INITiate")

    def abort(self):
        """Abort a measurement in progress (``ABORt``)."""
        self.write("ABORt")

    def continuous(self, enable: bool = True):
        """Enable/disable continuous initiation (``INITiate:CONTinuous``)."""
        self.write(f"INITiate:CONTinuous {_bool(enable)}")

    def read_average(self, samples: int = 1) -> float:
        """Mean of ``samples`` ``READ?`` calls (NaNs ignored)."""
        n = max(1, int(samples))
        vals = [v for v in (self.read() for _ in range(n)) if not math.isnan(v)]
        return sum(vals) / len(vals) if vals else math.nan

    # -- continuous (manually gated) totalize --
    def totalize_start(self, channel: int = 1):
        """Start totalizing events on Ch1 until :meth:`totalize_stop`."""
        _check_channel(channel, (1,))
        self.configure("totalize_continuous", channels=channel)
        self.initiate()

    def totalize_stop(self) -> float:
        """Stop a continuous totalize and return the count."""
        self.abort()
        return self.fetch()

    # -- [:SENSe]:FUNCtion[:ON]  (change function, keep other settings) --
    def set_function(self, function: str, *channels):
        """Select the measurement function without defaulting other settings.

        ``set_function("frequency", 2)``, ``set_function("time_interval", 1, 2)``,
        ``set_function("ratio", 1, 3)``, ``set_function("vmax", 1)``. Follow with
        :meth:`initiate` / :meth:`fetch` or :meth:`read`.
        """
        node = _SENSE_FUNCTIONS.get(function, function)
        chans = ",".join(str(int(c)) for c in channels)
        self.write(f'SENSe:FUNCtion:ON "{node}{" " + chans if chans else ""}"')

    def function(self) -> str:
        """Currently selected sensor function (``:SENSe:FUNCtion:ON?``)."""
        return self.query("SENSe:FUNCtion:ON?").strip('"')

    # -- inputs (channels 1 and 2; channel 3 has no settings) --
    def input(self, channel: int = 1, coupling: str | None = None,
              impedance: float | None = None, attenuation: int | None = None,
              lowpass: bool | None = None):
        """Configure an input channel. Only the arguments given are sent.

        coupling: ``"AC"`` or ``"DC"``; impedance: 50 or 1e6 (ohm);
        attenuation: 1 or 10; lowpass: the 100 kHz low-pass filter on/off.
        """
        ch = _check_channel(channel, (1, 2))
        if coupling is not None:
            c = coupling.upper()
            if c not in ("AC", "DC"):
                raise ValueError("coupling must be 'AC' or 'DC'")
            self.write(f"INPut{ch}:COUPling {c}")
        if impedance is not None:
            if impedance not in (50, 1e6, 1000000):
                raise ValueError("impedance must be 50 or 1e6 ohm")
            self.write(f"INPut{ch}:IMPedance {_num(impedance)} OHM")
        if attenuation is not None:
            if attenuation not in (1, 10):
                raise ValueError("attenuation must be 1 or 10")
            self.write(f"INPut{ch}:ATTenuation {attenuation}")
        if lowpass is not None:
            self.write(f"INPut{ch}:FILTer:LPASs:STATe {_bool(lowpass)}")

    def input_settings(self, channel: int = 1) -> dict:
        """Read back coupling, impedance, attenuation and filter for a channel."""
        ch = _check_channel(channel, (1, 2))
        return {
            "coupling": self.query(f"INPut{ch}:COUPling?"),
            "impedance": self.query_float(f"INPut{ch}:IMPedance?"),
            "attenuation": self.query_int(f"INPut{ch}:ATTenuation?"),
            "lowpass": self.query_int(f"INPut{ch}:FILTer:LPASs:STATe?") == 1,
        }

    # -- trigger events (channels 1 and 2) --
    def trigger_level(self, channel: int, volts: float):
        """Set an absolute trigger level in volts (turns auto-trigger off)."""
        ch = _check_channel(channel, (1, 2))
        self.write(f"SENSe:EVENt{ch}:LEVel:ABSolute {_num(volts)} V")
        self.write(f"SENSe:EVENt{ch}:LEVel:ABSolute:AUTO OFF")

    def trigger_auto(self, channel: int, percent: float | None = None):
        """Enable auto-trigger; optionally set the level as a percentage (0-100)
        of the signal's peak-to-peak range."""
        ch = _check_channel(channel, (1, 2))
        self.write(f"SENSe:EVENt{ch}:LEVel:ABSolute:AUTO ON")
        if percent is not None:
            self.write(f"SENSe:EVENt{ch}:LEVel:RELative {_num(percent)} PCT")

    def trigger_slope(self, channel: int, slope: str = "POS"):
        """Trigger on the ``"POS"`` or ``"NEG"`` edge."""
        ch = _check_channel(channel, (1, 2))
        s = slope.upper()[:3]
        if s not in ("POS", "NEG"):
            raise ValueError("slope must be 'POS' or 'NEG'")
        self.write(f"SENSe:EVENt{ch}:LEVel:SLOPe {'POSitive' if s == 'POS' else 'NEGative'}")

    def trigger_hysteresis(self, channel: int, percent: float):
        """Hysteresis window as a percentage (0 = minimum, 100 = maximum)."""
        ch = _check_channel(channel, (1, 2))
        self.write(f"SENSe:EVENt{ch}:HYSTeresis:RELative {_num(percent)} PCT")

    def trigger_level_query(self, channel: int) -> float:
        """Current absolute trigger level in volts (Ch1, 2 or 3)."""
        _check_channel(channel, (1, 2, 3))
        return self.query_float(f"SENSe:EVENt{channel}:LEVel:ABSolute?")

    # -- arming / gate time (frequency, period, ratio) --
    def gate_time(self, seconds: float):
        """Set the gate time for frequency/period/ratio measurements."""
        self.write("SENSe:FREQuency:ARM:STOP:SOURce TIMer")
        self.write(f"SENSe:FREQuency:ARM:STOP:TIMer {_num(seconds)} S")

    def gate_digits(self, digits: int):
        """Arm by resolution: gate runs until ``digits`` digits are acquired."""
        self.write("SENSe:FREQuency:ARM:STOP:SOURce DIGits")
        self.write(f"SENSe:FREQuency:ARM:STOP:DIGits {int(digits)}")

    def totalize_gate_time(self, seconds: float):
        """Gate time for timed totalize via ``[:SENSe]:TOTalize:ARM``."""
        self.write("SENSe:TOTalize:ARM:STOP:SOURce TIMer")
        self.write(f"SENSe:TOTalize:ARM:STOP:TIMer {_num(seconds)} S")

    def expected_frequency(self, hz: float | None = None, channel: int = 1):
        """Tell the counter the approximate input frequency, or (``hz=None``)
        turn on its automatic pre-measurement frequency detection."""
        _check_channel(channel, (1, 2, 3))
        if hz is None:
            self.write(f"SENSe:FREQuency:EXPected{channel}:AUTO ON")
        else:
            self.write(f"SENSe:FREQuency:EXPected{channel} {_num(hz)} HZ")

    # -- timebase --
    def reference_source(self, source: str = "AUTO"):
        """Timebase source: ``"AUTO"`` (use external when present), ``"INT"``
        or ``"EXT"``."""
        s = source.upper()[:4]
        if s == "AUTO":
            self.write("SENSe:ROSCillator:SOURce:AUTO ON")
        elif s in ("INT", "INTE"):
            self.write("SENSe:ROSCillator:SOURce:AUTO OFF")
            self.write("SENSe:ROSCillator:SOURce INTernal")
        elif s in ("EXT", "EXTE"):
            self.write("SENSe:ROSCillator:SOURce:AUTO OFF")
            self.write("SENSe:ROSCillator:SOURce EXTernal")
        else:
            raise ValueError("source must be 'AUTO', 'INT' or 'EXT'")

    def reference_source_query(self) -> str:
        return self.query("SENSe:ROSCillator:SOURce?")

    def external_reference_frequency(self) -> float:
        """Frequency of the external reference oscillator in Hz."""
        return self.query_float("SENSe:ROSCillator:EXTernal:FREQuency?")

    # -- statistics (CALCulate3) --
    def statistics(self, count: int = 100) -> dict:
        """Take ``count`` readings of the *currently configured* measurement and
        return ``{"mean", "std_dev", "max", "min"}``.

        Configure the measurement first (``configure_*`` or :meth:`set_function`);
        CONFigure/MEASure disable statistics, so they are enabled here.
        """
        n = max(1, int(count))
        self.write("CALCulate3:AVERage:STATe ON")
        self.write(f"CALCulate3:AVERage:COUNt {n}")
        self.write("CALCulate3:AVERage:CLEar")
        self.write("TRIGger:COUNt:AUTO ON")
        self.initiate()
        self.query("*OPC?")                       # blocks until all n readings are done
        mean, sd, mx, mn = self.query_floats("CALCulate3:AVERage:ALL?")
        return {"mean": mean, "std_dev": sd, "max": mx, "min": mn}

    def statistics_of(self, function: str, count: int = 100, *params,
                      channels=None) -> dict:
        """Configure ``function`` then return :meth:`statistics` over ``count`` readings.

        ``cnt.statistics_of("frequency", 50, 10e6, 1e-3, channels=1)``
        """
        self.configure(function, *params, channels=channels)
        return self.statistics(count)

    def statistics_off(self):
        """Disable statistics and return to one measurement per ``INITiate``."""
        self.write("CALCulate3:AVERage:STATe OFF")
        self.write("TRIGger:COUNt:AUTO OFF")

    # -- math: scale & offset (CALCulate1) --
    def scale_offset(self, scale: float = 1.0, offset: float = 0.0,
                     enable: bool = True):
        """Apply ``result = scale * reading + offset`` (``:TRACe:DATA`` + math state)."""
        self.write(f"TRACe:DATA SCALE,{_num(scale)}")
        self.write(f"TRACe:DATA OFFSET,{_num(offset)}")
        self.write(f"CALCulate:MATH:STATe {_bool(enable)}")

    def math_off(self):
        self.write("CALCulate:MATH:STATe OFF")

    # -- limit testing (CALCulate2) --
    def limit_test(self, lower: float, upper: float, enable: bool = True):
        """Pass/fail test the results against ``lower`` .. ``upper``
        (in the result's units: Hz, s, deg, ...)."""
        self.write(f"CALCulate2:LIMit:LOWer {_num(lower)}")
        self.write(f"CALCulate2:LIMit:UPPer {_num(upper)}")
        self.write("CALCulate2:LIMit:CLEar:IMMediate")
        self.write(f"CALCulate2:LIMit:STATe {_bool(enable)}")

    def limit_off(self):
        self.write("CALCulate2:LIMit:STATe OFF")

    def limit_failed(self) -> bool:
        """True if the last tested measurement failed (``CALC2:LIM:FAIL?``)."""
        return self.query_int("CALCulate2:LIMit:FAIL?") == 1

    def limit_counts(self) -> dict:
        """Pass/fail tallies since the last limit clear."""
        return {
            "passed": self.query_int("CALCulate2:LIMit:PCOunt?"),
            "failed": self.query_int("CALCulate2:LIMit:FCOunt?"),
            "failed_lower": self.query_int("CALCulate2:LIMit:FCOunt:LOWer?"),
            "failed_upper": self.query_int("CALCulate2:LIMit:FCOunt:UPPer?"),
        }

    def limit_clear(self):
        self.write("CALCulate2:LIMit:CLEar:IMMediate")
