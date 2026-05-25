#pragma once

#include <cstdint>
#include <ctime>

// Host timestamp in microseconds from CLOCK_MONOTONIC.
//
// CLOCK_MONOTONIC is host-wide (shared by every process on the machine) and is
// immune to wall-clock/NTP jumps. This value MUST stay byte-for-byte comparable
// with the Python recorders' ts() (py/utils/host_time.py): postprocess aligns
// the camera (now a separate Python process) against the arm/hand/G1 streams
// (C++) via the shared host_timestamp column, so both languages must read the
// same clock, epoch, and units.
//
// Do NOT reintroduce a per-process origin: that put each process on its own
// timeline, which only worked while every recorder lived in this one process.
class Time {
public:
    static uint64_t ts() {
        struct timespec t;
        clock_gettime(CLOCK_MONOTONIC, &t);
        return static_cast<uint64_t>(t.tv_sec) * 1000000ull
             + static_cast<uint64_t>(t.tv_nsec) / 1000ull;
    }
};
