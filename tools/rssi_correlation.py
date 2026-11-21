"""Reproduce the firmware's 1 ms grid, gap masks and normalized correlation."""

import math
import numpy as np

MAX_LAG = 155
GRID_SIZE = 406


def grid_at(rows, index):
    """Use the logged analysis clock so the resampling phase matches the ESP32."""
    packets = [row for row in rows[:index + 1]
               if row.get("kind", 1) == 1 and math.isfinite(row.get("rssi_dbm", math.nan))]
    if not packets:
        return np.full(GRID_SIZE, np.nan, dtype=np.float32)
    times = []
    previous = None
    elapsed = 0
    for row in packets:
        timestamp = int(row["timestamp_us"])
        if previous is None:
            elapsed = timestamp
        else:
            delta = (timestamp - previous) & 0xFFFFFFFF
            if delta == 0 or delta >= 0x80000000:
                continue
            if delta >= 50000:
                times.clear()
            elapsed += delta
        times.append((elapsed, row["rssi_dbm"]))
        previous = timestamp

    endpoint = rows[index].get("analysis_timestamp_us", 0)
    if endpoint:
        delta = (int(endpoint) - int(previous)) & 0xFFFFFFFF
        if delta >= 0x80000000:
            delta -= 0x100000000
        end = times[-1][0] + delta
    else:
        # Legacy exports do not record the original grid phase.
        end = times[0][0] + ((times[-1][0] - times[0][0]) // 1000) * 1000
    targets = end - np.arange(GRID_SIZE - 1, -1, -1, dtype=np.int64) * 1000
    grid = np.full(GRID_SIZE, np.nan, dtype=np.float32)
    for timestamp, value in times:
        grid[targets == timestamp] = value
    for (a, x), (b, y) in zip(times, times[1:]):
        if b - a > 8000:
            continue
        mask = (targets >= a) & (targets <= b)
        fraction = (targets[mask] - a).astype(np.float32) / np.float32(b - a)
        grid[mask] = np.float32(x) + fraction * np.float32(y - x)
    return grid


def correlation_curve(grid, samples=250, first=9, last=155):
    """Match RssiRpmEstimator::analyze, including paired variance normalization."""
    result = np.full(MAX_LAG + 2, np.nan, dtype=np.float32)
    if not (2 <= samples <= 250 and 9 <= first <= last <= MAX_LAG):
        return result
    span = samples + last + 1
    window = np.array(grid[-span:], dtype=np.float32, copy=True)
    if len(window) != span:
        return result
    valid = np.isfinite(window)
    available = np.count_nonzero(valid)
    if available < span // 2:
        return result
    time = np.arange(span, dtype=np.float32) - np.float32((span - 1) / 2)
    window += np.float32(64)
    t, w = time[valid], window[valid]
    count = np.float32(available)
    time_sum = t.sum(dtype=np.float32)
    rssi_sum = w.sum(dtype=np.float32)
    slope = ((t * w).sum(dtype=np.float32) - time_sum * rssi_sum / count) / (
        (t * t).sum(dtype=np.float32) - time_sum * time_sum / count)
    window[valid] -= slope * time[valid]
    ref_start = last + 1
    for lag in range(first - 1, last + 2):
        paired = valid[ref_start:] & valid[ref_start - lag:span - lag]
        count = np.count_nonzero(paired)
        if count < max(32, samples // 2):
            continue
        x = window[ref_start:][paired]
        y = window[ref_start - lag:span - lag][paired]
        sx, sy = x.sum(dtype=np.float32), y.sum(dtype=np.float32)
        vx = (x * x).sum(dtype=np.float32) - sx * sx / count
        vy = (y * y).sum(dtype=np.float32) - sy * sy / count
        if vx >= count and vy >= count:
            result[lag] = ((x * y).sum(dtype=np.float32) - sx * sy / count) / np.sqrt(vx * vy)
    return result
