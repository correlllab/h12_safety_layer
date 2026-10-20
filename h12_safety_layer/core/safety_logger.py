'''Operational logging interface for the safety layer'''

from typing import Protocol


class SafetyLogger(Protocol):
    '''Log safety messages independently of the runtime'''
    def info(self, message: str) -> None: ...

    def error(self, message: str) -> None: ...


class PrintSafetyLogger:
    '''Emit safety messages immediately in plain Python'''
    def info(self, message: str) -> None:
        print(message, flush=True)

    def error(self, message: str) -> None:
        print(message, flush=True)
