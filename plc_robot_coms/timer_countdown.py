#!/usr/bin/python
#
# common ;ibrary for timer instances 
#
from __future__ import annotations
import time
from datetime import timedelta

def time_ms() -> int:
    """Returns the current :py:func:`time.time` as milliseconds."""
    return round(time.time() * 1000)

class Countdown:
    """Utility class for counting down time. Exposes a simple API to initiate
    it with an initial timeout and to check whether is has expired."""
    def __init__(self, init_timeout: timedelta | None):
        if init_timeout is not None:
            self._timeout_ms = int(init_timeout / timedelta(milliseconds=1))
            self._start_time_ms = time_ms()
        else:
            self._timeout_ms = 0
            self._start_time_ms = 0
    @classmethod
    def from_seconds(cls, timeout_seconds: float) -> Countdown:
        return cls(timedelta(seconds=timeout_seconds))
    @classmethod
    def from_millis(cls, timeout_ms: int) -> Countdown:
        return cls(timedelta(milliseconds=timeout_ms))
    @property
    def timeout_ms(self) -> int:
        """Returns timeout as integer milliseconds."""
        return self._timeout_ms
    @property
    def timeout(self) -> timedelta:
        return timedelta(milliseconds=self._timeout_ms)
    @timeout.setter
    def timeout(self, timeout: timedelta) -> None:
        """Set a new timeout for the countdown instance."""
        self._timeout_ms = round(timeout / timedelta(milliseconds=1))
    def timed_out(self) -> bool:
        return round(time_ms() - self._start_time_ms) >= self._timeout_ms
    def busy(self) -> bool:
        return not self.timed_out()
    def reset(self, new_timeout: timedelta | None = None) -> None:
        if new_timeout is not None:
            self.timeout = new_timeout
        self.start()
    def start(self) -> None:
        self._start_time_ms = time_ms()
    def time_out(self) -> None:
        self._start_time_ms = 0
    def remaining_time(self) -> timedelta:
        """Remaining time left."""
        end_time = self._start_time_ms + self._timeout_ms
        current = time_ms()
        if end_time < current:
            return timedelta()
        return timedelta(milliseconds=end_time - current)
    def __repr__(self):
        return f"{self.__class__.__name__}(init_timeout={timedelta(milliseconds=self._timeout_ms)})"
    def __str__(self):
        return (
            f"{self.__class__.__class__} with"
            f" {timedelta(milliseconds=self._timeout_ms)} ms timeout,"
            f" {self.remaining_time()} time remaining"
        )

from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
	from typing import Any
	from collections.abc import Callable

from abc import ABC, abstractmethod
from contextlib import AbstractContextManager
from time import monotonic
from asyncio import sleep

# condition globals (for demonstrating condition statement use)
a = 100
b = 200
c = 50
d = 90
db = 1

# define here the conditions to test
def ab_less():
    return a < b

def ab_greater():
    return a > b

def cd_equal():
    return not ((c < (d-db)) or (c > (d+db)))

class AbstractCondition(ABC):
	@abstractmethod
	def __call__(self) -> bool:
		return False
	def __bool__(self) -> bool:
		return self()

class Condition(AbstractCondition):
	def __init__(self, condition: Callable[[], Any]):
		self.condition = condition
	def __call__(self):
		return bool(self.condition())

class Timer(AbstractCondition, AbstractContextManager):
	def __init__(self, timeout:float, reset=True):
		self.timeout = timeout
		if reset:
			self.reset()
		else:
			self.clear()
	def reset(self):
		self.expire = monotonic() + self.timeout
	def clear(self):
		self.expire = 0
	def __enter__(self):
		self.reset()
	def __exit__(self, *exc):
		self.clear()
	def left(self):
		return max(0, self.expire - monotonic())
	async def wait(self):
		while left := self.left():
			await sleep(left)
	def __call__(self):
		return monotonic() < self.expire	#always return False if timeout==0

class Timeout(Timer):
	def __call__(self):
		return monotonic() >= self.expire	#always return True if timeout==0

class Pulse(AbstractCondition):
	def __init__(self, hertz:float = 1):
		self.period = 1 / hertz
		self.switch = self.period / 2
	def __call__(self):
		return (monotonic() % self.period) < self.switch

# example of use of the timer classes above
def main():
    global a,b,c,d
    # check some conditions 
    c1 = Condition(ab_less) 
    c2 = Condition(ab_greater)  
    c3 = Condition(cd_equal) 
    print(f"c1 {c1} c2 {c2} c3 {c3}") 
    a = 1000
    c = 87
    print(f"c1 {c1} c2 {c2} c3 {c3}") 
    c = 100
    print(f"c1 {c1} c2 {c2} c3 {c3}")
    c = 89.1
    print(f"c1 {c1} c2 {c2} c3 {c3}")
    c = 100.9
    print(f"c1 {c1} c2 {c2} c3 {c3}")
    # show timer class
    t = Timer(900)
    # show countdown timer
    c = Countdown.from_millis(5000)
    # create pulse object @ 0.5Hz 
    p = Pulse(0.5)
    print(f"time left {t.left()}")
    print(f"c counting {c.busy()} timed_out {c.timed_out()}")
    for i in range(0,6):
        print(f"pulse {p()}")
        time.sleep(0.5)
    time.sleep(10)
    print(f"time left {t.left()}")
    t.reset()
    print(f"time left {t.left()}")
    print(f"c counting {c.busy()} timed_out {c.timed_out()}")

if __name__ == '__main__':
    main()
