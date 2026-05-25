"""Host timestamp in microseconds from CLOCK_MONOTONIC.

Must stay byte-for-byte comparable with the C++ Time::ts() (cpp/utils/time.hpp).
Postprocess aligns the Python camera stream against the C++ arm/hand/G1 streams
via the shared host_timestamp column, so both languages must read the same clock
(CLOCK_MONOTONIC), epoch, and units (microseconds).
"""
import time


def ts() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC) // 1000
