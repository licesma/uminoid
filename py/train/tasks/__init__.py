from __future__ import annotations

from typing import Callable

from py.train.config import FinetuneConfig
from py.train.tasks import pen, plum, apple


TASKS: dict[str, Callable[[], FinetuneConfig]] = {
    "apple_full_june_8":             apple.full_june_8,
    "apple_first_80_june_8":         apple.f80_june_8,
    "plum_may_28":                   plum.plum_may_28,
    "pen_marker_may_26":             pen.pen_marker_may_26,
    "pen_marker_with_actions_may_28": pen.pen_marker_with_actions_may_28,
}
