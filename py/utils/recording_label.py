"""Generate a timestamped session label, e.g. 'may_25_13:07'.

Port of cpp/utils/recording_label.hpp: localtime formatted "%B_%d_%H:%M",
lowercased. Used as the per-session directory name under the data dir.
"""
from __future__ import annotations

from datetime import datetime


def generate_recording_label() -> str:
    return datetime.now().strftime("%B_%d_%H:%M").lower()
