"""
Inspire Hand RH56 Control Library

A Python library for controlling the Inspire Hand RH56dfq robotic hand.
"""

from .hand import InspireHand, Finger
from .exceptions import InspireHandError, ConnectionError, CommandError

__version__ = '0.1.0' 