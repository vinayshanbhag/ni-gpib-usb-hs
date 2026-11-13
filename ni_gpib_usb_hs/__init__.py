# SPDX-License-Identifier: GPL-2.0-only
"""Pure-Python user-space driver for the NI GPIB-USB-HS, with an Agilent 34401A wrapper.

    from ni_gpib_usb_hs import NIUSBGPIB, Agilent34401A, GpibError
"""
from .controller import NIUSBGPIB, GpibError, NI_VID, NI_GPIB_USB_HS_PID
from .agilent34401a import Agilent34401A
from .agilent53131a import Agilent53131A
from .hp33120a import HP33120A
from .ifr2031 import IFR2031

__version__ = "0.1.0"
__all__ = ["NIUSBGPIB", "Agilent34401A", "Agilent53131A", "HP33120A", "IFR2031", "GpibError", "NI_VID", "NI_GPIB_USB_HS_PID"]
