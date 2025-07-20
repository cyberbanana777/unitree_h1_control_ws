from . import utils
from . import core
from . import rpc
from . import go2
from . import idl  # Если ошибка возникает здесь, проблема в модуле idl

__all__ = [
    "idl",
    "utils",
    "core",
    "rpc",
    "go2",
]
