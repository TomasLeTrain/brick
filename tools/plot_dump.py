#!/usr/bin/env python3
"""Plot Brick binary snapshots, serial CSV exports, and simulation CSV files."""

import argparse
import csv
from dataclasses import dataclass
import io
import math
from pathlib import Path
import re
import struct
import warnings
import zlib

HEADER = struct.Struct("<8I")
RECORD = struct.Struct("<II8fIIIhBB")
BINARY_FIELDS = (
    "timestamp_us", "received_us", "ax_g", "ay_g", "az_g", "accel_rpm",
    "rssi_rpm", "control_rpm", "rssi_hz", "throttle", "estimator_us",
    "rx_dropped", "rssi_dropped", "rssi_dbm", "kind", "connected",
)
ALIASES = {"rssi": "rssi_dbm", "timestamp": "timestamp_us"}
PANELS = (
    ("RSSI (dBm)", ("rssi_dbm",)),
    ("RPM", ("accel_rpm", "rssi_rpm", "control_rpm", "true_rpm")),
    ("Acceleration (g)", ("ax_g", "ay_g", "az_g")),
    ("Estimator update (µs)", ("estimator_us",)),
    ("Packets/s", ("rssi_hz",)),
)


@dataclass
class Dump:
    rows: list
    metadata: str = ""


def read_binary(path):
    data = path.read_bytes()
    if len(data) < HEADER.size:
        raise ValueError("truncated binary header")
    magic, version, generation, count, record_size, crc, overwritten, paused = (
        HEADER.unpack_from(data)
    )
    if magic != 0x31524442 or version != 1 or record_size != RECORD.size:
        raise ValueError("unsupported binary dump format")
    payload = data[HEADER.size:]
    if count > 2048 or len(payload) != count * RECORD.size:
        raise ValueError("truncated binary dump or invalid record count")
    if zlib.crc32(payload) != crc:
        raise ValueError("binary dump CRC mismatch")
    rows = [dict(zip(BINARY_FIELDS, values)) for values in RECORD.iter_unpack(payload)]
    return [Dump(rows, f"generation={generation} overwritten={overwritten} paused={paused}")]


def read_csv(path, allow_incomplete=False):
    runs = []
    current = []
    columns = None
    metadata = ""
    marked = False
    expected = None
    errors = []

    def finish(complete):
        error = None
        if not complete:
            error = "incomplete export, missing # dump_end"
        elif expected is not None and len(current) != expected:
            error = f"expected {expected} records, received {len(current)}"
        if error and not allow_incomplete:
            errors.append(error)
        elif current:
            runs.append(Dump(current.copy(), metadata))

    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        line = line.strip()
        if line.startswith("# dump_begin"):
            if current:
                finish(not marked)
            current = []
            columns = None
            metadata = line.removeprefix("# dump_begin").strip()
            marked = True
            match = re.search(r"\bcount=(\d+)\b", metadata)
            expected = int(match[1]) if match else None
            continue
        if line == "# dump_end":
            finish(True)
            current = []
            marked = False
            columns = None
            expected = None
            continue
        if not line or line.startswith("#"):
            continue
        cells = next(csv.reader(io.StringIO(line)))
        normalized = [ALIASES.get(c.strip().lower(), c.strip().lower()) for c in cells]
        if "timestamp_us" in normalized or "time_s" in normalized:
            if columns is not None and current:
                finish(not marked)
                current = []
            columns = normalized
            continue
        if columns is None:
            continue  # Ignore startup, telemetry and benchmark status text.
        try:
            float(cells[0])
        except (ValueError, IndexError):
            continue
        if len(cells) != len(columns):
            raise ValueError(f"line {line_number} has {len(cells)} cells, expected {len(columns)}")
        try:
            current.append({key: float(value) if value else math.nan
                            for key, value in zip(columns, cells)})
        except ValueError as error:
            raise ValueError(f"invalid numeric record at line {line_number}") from error
    if current or marked:
        finish(not marked)
    if not runs:
        raise ValueError(errors[-1] if errors else "no dump records found")
    if errors:
        warnings.warn(f"ignored {len(errors)} incomplete exports; {errors[-1]}")
    return runs


def load(path, allow_incomplete=False):
    with path.open("rb") as stream:
        magic = stream.read(4)
    return read_binary(path) if magic == b"BDR1" else read_csv(path, allow_incomplete)


def unwrap(values):
    """Unwrap a uint32 microsecond clock without losing its initial epoch."""
    result = []
    previous = None
    current = 0
    for value in values:
        value = int(value)
        if previous is None:
            current = value
        else:
            delta = (value - previous) & 0xFFFFFFFF
            if delta >= 0x80000000:
                delta -= 0x100000000
            current += delta
        result.append(current)
        previous = value
    return result


def times(rows):
    if "time_s" in rows[0]:
        origin = rows[0]["time_s"]
        return [row["time_s"] - origin for row in rows]
    if "received_us" not in rows[0]:
        clock = unwrap([row["timestamp_us"] for row in rows])
        return [(value - clock[0]) / 1e6 for value in clock]

    received = unwrap([row["received_us"] for row in rows])
    radio_indices = [i for i, row in enumerate(rows) if row.get("kind") == 1]
    radio_times = unwrap([rows[i]["timestamp_us"] for i in radio_indices])
    aligned = received.copy()
    if radio_indices:
        # RSSI uses the Wi-Fi clock. Preserve its exact packet intervals and
        # anchor its first sample to callback delivery time on the local clock.
        anchor = received[radio_indices[0]] - radio_times[0]
        for index, radio in zip(radio_indices, radio_times):
            aligned[index] = radio + anchor
    for i, row in enumerate(rows):
        if row.get("kind") != 1:
            delta = (int(row["timestamp_us"]) - int(row["received_us"])) & 0xFFFFFFFF
            if delta >= 0x80000000:
                delta -= 0x100000000
            aligned[i] = received[i] + delta
    origin = min(aligned)
    return [(value - origin) / 1e6 for value in aligned]


def plot(dumps, start=None, end=None):
    import matplotlib.pyplot as plt

    available = {field for _, dump in dumps for row in dump.rows for field in row}
    panels = [(label, fields) for label, fields in PANELS if available.intersection(fields)]
    if not panels:
        raise ValueError("no supported RSSI, RPM, acceleration or timing columns")
    figure, axes = plt.subplots(len(panels), 1, sharex=True, squeeze=False,
                                figsize=(12, 2.3 * len(panels)), layout="constrained")
    for label, dump in dumps:
        clock = times(dump.rows)
        for axis, (ylabel, fields) in zip(axes[:, 0], panels):
            for field in fields:
                points = []
                for t, row in zip(clock, dump.rows):
                    kind = row.get("kind", 0)
                    if kind and ((field == "rssi_dbm" and kind != 1) or
                                 (field != "rssi_dbm" and kind != 2)):
                        continue
                    value = row.get(field, math.nan)
                    if not math.isfinite(value):
                        continue
                    if (start is None or t >= start) and (end is None or t <= end):
                        points.append((t, value))
                if points:
                    x, y = zip(*points)
                    axis.plot(x, y, linewidth=0.8, label=f"{label} {field}")
            axis.set_ylabel(ylabel)
            axis.grid(alpha=0.25)
    for axis in axes[:, 0]:
        if axis.lines:
            axis.legend(loc="upper right", fontsize=8)
    axes[-1, 0].set_xlabel("Time from first captured sample (s)")
    return figure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", type=Path, nargs="+", help="CSV exports or binary snapshots")
    parser.add_argument("--run", type=int, default=-1, help="run index within each file, default last")
    parser.add_argument("--start", type=float, help="first plotted second")
    parser.add_argument("--end", type=float, help="last plotted second")
    parser.add_argument("--save", type=Path, help="save a PNG, PDF or SVG instead of opening a window")
    parser.add_argument("--allow-incomplete", action="store_true", help="plot partial serial exports")
    args = parser.parse_args()
    try:
        if args.start is not None and args.end is not None and args.end <= args.start:
            raise ValueError("--end must exceed --start")
        selected = []
        for path in args.files:
            runs = load(path, args.allow_incomplete)
            try:
                dump = runs[args.run]
            except IndexError as error:
                raise ValueError(f"{path} contains {len(runs)} runs") from error
            selected.append((path.stem, dump))
            states = [row for row in dump.rows if row.get("kind", 0) != 1]
            dropped = {key: max((row.get(key, 0) for row in states), default=0)
                       for key in ("rx_dropped", "rssi_dropped")}
            print(f"{path}: {len(dump.rows)} records, {dump.metadata}, drops={dropped}")
        if args.save:
            import matplotlib
            matplotlib.use("Agg")
        figure = plot(selected, args.start, args.end)
        if args.save:
            figure.savefig(args.save, dpi=160)
        else:
            import matplotlib.pyplot as plt
            plt.show()
    except (OSError, ValueError) as error:
        parser.exit(2, f"{error}\n")


if __name__ == "__main__":
    main()
