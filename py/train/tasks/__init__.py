from __future__ import annotations

from typing import Callable

from py.train.config import FinetuneConfig
from py.train.tasks import pen, plum


TASKS: dict[str, Callable[[], FinetuneConfig]] = {
    "plum_may_28":                   plum.plum_may_28,
    "pen_marker_may_26":             pen.pen_marker_may_26,
    "pen_marker_with_actions_may_28": pen.pen_marker_with_actions_may_28,
}
