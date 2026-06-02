"""Threaded CSV writer for the Python recorders.

A port of cpp/utils/csv_saver.hpp: a background thread drains a queue and does
the file I/O, so the caller's hot path (the recorder's capture loop) never blocks
on disk. This works in CPython despite the GIL because file write/flush release
the GIL while blocked in the syscall — the producer thread keeps running.

`queue.Queue` provides the thread-safe handoff (the C++ deque + mutex +
condition_variable). Same constructor / write_line / close surface as the C++
class, so recorders port over unchanged.
"""
from __future__ import annotations

import queue
import threading


class CsvSaver:
    _SENTINEL = object()  # shutdown marker pushed by close()

    def __init__(self, path: str, header: str, flush_every: int = 1000) -> None:
        self._file = open(path, "w")
        self._file.write(header + "\n")
        self._flush_every = max(1, flush_every)
        self._queue: "queue.Queue[object]" = queue.Queue()
        self._closed = False
        self._thread = threading.Thread(
            target=self._writer_loop, name="csv-saver", daemon=True
        )
        self._thread.start()

    def write_line(self, line: str) -> None:
        # Non-blocking handoff; the writer thread does the disk I/O.
        if self._closed:
            return
        self._queue.put(line)

    def _writer_loop(self) -> None:
        count = 0
        while True:
            item = self._queue.get()
            if item is self._SENTINEL:
                break
            self._file.write(item + "\n")  # type: ignore[operator]
            count += 1
            if count % self._flush_every == 0:
                self._file.flush()
        self._file.flush()
        self._file.close()

    def close(self) -> None:
        # Flush everything already queued, then stop the writer thread.
        if self._closed:
            return
        self._closed = True
        self._queue.put(self._SENTINEL)
        self._thread.join()

    def __enter__(self) -> "CsvSaver":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
