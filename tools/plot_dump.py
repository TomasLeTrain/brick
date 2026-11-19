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
RECORD_V1 = struct.Struct("<II8fIIIhBB")
RECORD_V2 = struct.Struct("<II8fIIIhBB3fI3H2B")
RECORD = struct.Struct("<II8fIIIhBB3fI3H2BI")
BINARY_FIELDS = (
    "timestamp_us", "received_us", "ax_g", "ay_g", "az_g", "target_rpm",
    "rssi_rpm", "control_rpm", "rssi_hz", "throttle", "estimator_us",
    "rx_dropped", "rssi_dropped", "rssi_dbm", "kind", "connected",
    "fast_rpm", "long_rpm", "correlation", "analysis_timestamp_us",
    "comparison_samples", "min_lag", "max_lag", "tracking_mode", "hint_used",
    "motor_gap_us",
)
BINARY_FIELDS_V1 = BINARY_FIELDS[:5] + ("accel_rpm",) + BINARY_FIELDS[6:16]
ALIASES = {"rssi": "rssi_dbm", "timestamp": "timestamp_us"}
RPM_STATUS = {
    0: "warming up", 1: "packet rate too low", 2: "RSSI variance too low",
    3: "missing sample coverage", 4: "no credible period",
    5: "confirming period", 6: "tracking", 7: "holding previous RPM", 8: "stale",
    9: "period jump rejected",
}
PHASE_STATUS = {0: "unavailable", 1: "acquiring", 2: "phase locked",
                3: "dead reckoning", 4: "ambiguous antenna lobes",
                5: "steady RPM-only / RF monitoring", 6: "translation hold",
                7: "phase discontinuity rejected"}
HEADING_STATUS = {8: "heading unavailable", 9: "RPM dead reckoning",
                  10: "RPM delay compensation", 11: "RPM coasting", 12: "translation noise guard",
                  13: "translation coherent RPM trend"}
PANELS = (
    ("RSSI (dBm)", ("rssi_dbm",)),
    ("RPM", ("rssi_rpm", "fast_rpm", "long_rpm", "target_rpm", "phase_rpm", "heading_rpm", "true_rpm")),
    ("Heading (wrapped rad)", ("heading_rad",)),
    ("Heading error (rad)", ("heading_error_rad",)),
    ("RPM measurement age (ms)", ("rpm_measurement_age_ms",)),
    ("Phase (wrapped rad)", ("phase_rad", "true_phase_rad")),
    ("Phase innovation (rad)", ("phase_error_rad",)),
    ("Phase confidence", ("phase_confidence",)),
    ("Motor throttle", ("throttle",)),
    ("Timing (µs)", ("estimator_us", "motor_gap_us")),
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
    record = {1: RECORD_V1, 2: RECORD_V2, 3: RECORD, 4: RECORD, 5: RECORD}.get(version)
    fields = BINARY_FIELDS_V1 if version == 1 else BINARY_FIELDS[:25] if version == 2 else BINARY_FIELDS
    if magic != 0x31524442 or record is None or record_size != record.size:
        raise ValueError("unsupported binary dump format")
    payload = data[HEADER.size:]
    if count > 2048 or len(payload) != count * record.size:
        raise ValueError("truncated binary dump or invalid record count")
    if zlib.crc32(payload) != crc:
        raise ValueError("binary dump CRC mismatch")
    rows = [dict(zip(fields, values)) for values in record.iter_unpack(payload)]
    for row in rows:
        if version >= 3 and row["connected"] & 0x80:
            flags = row["connected"]
            row["connection_state"] = (flags >> 1) & 7
            row["controller_connected"] = bool(flags & 0x10)
            row["spin_enabled"] = bool(flags & 0x20)
            row["capture_switch"] = bool(flags & 0x40)
            row["connected"] = flags & 1
            row["rpm_status"] = row["tracking_mode"] >> 2
            row["tracking_mode"] &= 3
        if version == 5:
            row["heading_rad"] = row.pop("ax_g")
            row["heading_error_rad"] = row.pop("ay_g")
            row["rpm_measurement_age_ms"] = row.pop("az_g")
            row["heading_rpm"] = row["control_rpm"]
            row["heading_status"] = row["hint_used"] >> 1 if row["kind"] == 2 else -1
            row["hint_used"] &= 1
        if version == 4:
            row["phase_rad"] = row.pop("ax_g")
            row["phase_error_rad"] = row.pop("ay_g")
            row["phase_confidence"] = row.pop("az_g")
            row["phase_rpm"] = row["control_rpm"]
            row["phase_status"] = row["hint_used"] >> 1 if row["kind"] == 2 else -1
            row["hint_used"] &= 1
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


class DumpInspector:
    def __init__(self, figure, axes, curve_axis, slider, dumps, initial_time):
        from bisect import bisect_right
        self.figure = figure
        self.axes = axes
        self.curve_axis = curve_axis
        self.slider = slider
        self.dumps = [(label, dump, times(dump.rows)) for label, dump in dumps]
        self.bisect_right = bisect_right
        self.cursors = [axis.axvline(initial_time, color="black", alpha=.5)
                        for axis in axes]
        slider.on_changed(self.select)
        figure.canvas.mpl_connect("button_press_event", self.click)
        self.select(initial_time)

    def click(self, event):
        if event.inaxes in self.axes and event.xdata is not None:
            # Toolbar zoom/pan retains its normal mouse behavior.
            toolbar = getattr(self.figure.canvas.manager, "toolbar", None)
            if toolbar is None or not toolbar.mode:
                self.slider.set_val(max(self.slider.valmin,
                                        min(self.slider.valmax, event.xdata)))

    def select(self, selected_time):
        from rssi_correlation import grid_at, correlation_curve
        import numpy as np

        axis = self.curve_axis
        axis.clear()
        for cursor in self.cursors:
            cursor.set_xdata([selected_time, selected_time])
        notes = []
        for label, dump, clock in self.dumps:
            display = label + " " if len(self.dumps) > 1 else ""
            state_indices = [i for i, row in enumerate(dump.rows)
                             if row.get("kind", 0) != 1 and clock[i] <= selected_time]
            # Raw RSSI-only reference captures also support curve inspection.
            index = state_indices[-1] if state_indices else max(
                0, self.bisect_right(clock, selected_time) - 1)
            row = dump.rows[index]
            reason = row.get("rpm_status", -1)
            if math.isfinite(reason) and reason >= 0:
                notes.append(f"{label}: {RPM_STATUS.get(int(reason), 'unknown RPM status')}")
            phase_status = row.get("phase_status", -1)
            if math.isfinite(phase_status) and phase_status >= 0:
                notes.append(f"{label}: {PHASE_STATUS.get(int(phase_status), 'unknown phase status')}")
            heading_status = row.get("heading_status", -1)
            if math.isfinite(heading_status) and heading_status >= 0:
                notes.append(f"{label}: {HEADING_STATUS.get(int(heading_status), 'unknown heading status')}")
            if row.get("kind") == 2 and row.get("connected") == 0:
                notes.append(f"{label}: firmware was not ready to drive")
            grid = grid_at(dump.rows, index)
            samples = int(row.get("comparison_samples", 0)) or 250
            first = int(row.get("min_lag", 0)) or 9
            last = int(row.get("max_lag", 0)) or 155
            for name, n, lo, hi in (("tracking", samples, first, last),
                                    ("long", 250, 9, 155)):
                if name == "long" and (samples, first, last) == (250, 9, 155):
                    continue
                curve = correlation_curve(grid, n, lo, hi)
                if np.isfinite(curve).any():
                    axis.plot(np.arange(len(curve)), curve,
                              label=f"{display}{name} ({n} ms)")
                else:
                    notes.append(f"{label} {name}: insufficient history/variance")
            rpm = row.get("rssi_rpm", 0)
            if rpm > 0:
                axis.axvline(60000 / rpm, linestyle="--",
                             label=f"{display}used {rpm:.0f} RPM")
            prior = row.get("target_rpm", 0)
            if row.get("kind", 0) != 1:
                values = [f"{field}={row[field]:.1f} RPM"
                          for field in ("target_rpm", "rssi_rpm")
                          if math.isfinite(row.get(field, math.nan))]
                if values:
                    notes.append(f"{label}: " + ", ".join(values))
            if prior > 0:
                axis.axvline(60000 / prior, linestyle=":", alpha=.6,
                             label=f"{display}steady prior {prior:.0f} RPM")
        axis.set(xlabel="Lag (ms)", ylabel="Normalized correlation",
                 title=f"Latest analysis at cursor {selected_time:.3f} s",
                 xlim=(8, 156), ylim=(-1.05, 1.05))
        axis.grid(alpha=.25)
        if axis.lines:
            axis.legend(loc="upper right", fontsize=8)
        if notes:
            axis.text(.02, .02, "\n".join(notes), transform=axis.transAxes, fontsize=8)
        self.figure.canvas.draw_idle()


def plot(dumps, start=None, end=None, selected_time=None):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    available = {field for _, dump in dumps for row in dump.rows
                 for field, value in row.items() if math.isfinite(value) and
                 (row.get("kind") != 1 or field == "rssi_dbm")}
    panels = []
    for label, fields in PANELS:
        if "heading_rad" in available:
            if "heading_rad" in fields:
                fields += ("true_phase_rad",)
            elif "phase_rad" in fields:
                fields = tuple(f for f in fields if f != "true_phase_rad")
        if available.intersection(fields):
            panels.append((label, fields))
    if not panels:
        raise ValueError("no supported RSSI, RPM, throttle, packet-rate or timing columns")
    figure = plt.figure(figsize=(15, max(8, len(panels) * 1.55)))
    grid = figure.add_gridspec(len(panels), 2, width_ratios=(1.6, 1),
                              left=.07, right=.98, top=.96, bottom=.11,
                              hspace=.25, wspace=.22)
    axes = []
    for i, (ylabel, fields) in enumerate(panels):
        axis = figure.add_subplot(grid[i, 0], sharex=axes[0] if axes else None)
        axes.append(axis)
        for label, dump in dumps:
            clock = times(dump.rows)
            for field in fields:
                points = []
                for t, row in zip(clock, dump.rows):
                    kind = row.get("kind", 0)
                    if kind and ((field == "rssi_dbm" and kind != 1) or
                                 (field != "rssi_dbm" and kind != 2)):
                        continue
                    # Legacy CSV re-exports append unavailable phase fields;
                    # RSSI-row placeholders must not create phantom panels.
                    value = row.get(field, math.nan)
                    if not math.isfinite(value):
                        continue
                    if field == "phase_rad" and row.get("phase_status") == 0:
                        continue
                    if field in ("phase_rad", "true_phase_rad", "heading_rad"):
                        value = (value + math.pi) % (2 * math.pi) - math.pi
                    if (start is None or t >= start) and (end is None or t <= end):
                        points.append((t, value))
                if points:
                    x, y = zip(*points)
                    style = {"linewidth": .8}
                    if field == "target_rpm":
                        style.update(linestyle="--", linewidth=1.5,
                                     drawstyle="steps-post", zorder=3)
                    elif field == "rssi_rpm":
                        style.update(linewidth=1.8, zorder=4)
                    elif field == "fast_rpm":
                        style.update(linestyle=":", linewidth=1, alpha=.7)
                    axis.plot(x, y, label=f"{label} {field}" if len(dumps) > 1 else field,
                              **style)
        axis.set_ylabel(ylabel)
        axis.grid(alpha=.25)
        if axis.lines:
            axis.legend(loc="upper right", fontsize=7, ncol=2)
        if i < len(panels) - 1:
            axis.tick_params(labelbottom=False)
    axes[-1].set_xlabel("Time from first captured sample (s)")
    lower = start if start is not None else 0
    upper = end if end is not None else max(max(times(d.rows)) for _, d in dumps)
    upper = max(lower + .001, upper)
    axes[0].set_xlim(lower, upper)
    initial = max(lower, min(upper, selected_time if selected_time is not None else upper))
    curve_axis = figure.add_subplot(grid[:, 1])
    slider = Slider(figure.add_axes((.15, .035, .65, .025)), "Time (s)",
                    lower, upper, valinit=initial)
    figure._dump_inspector = DumpInspector(figure, axes, curve_axis, slider, dumps, initial)
    return figure


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", type=Path, nargs="+", help="CSV exports or binary snapshots")
    parser.add_argument("--run", type=int, default=-1, help="run index within each file, default last")
    parser.add_argument("--start", type=float, help="first plotted second")
    parser.add_argument("--end", type=float, help="last plotted second")
    parser.add_argument("--time", type=float, help="initial correlation cursor time in seconds")
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
        figure = plot(selected, args.start, args.end, args.time)
        if args.save:
            figure.savefig(args.save, dpi=160)
        else:
            import matplotlib.pyplot as plt
            plt.show()
    except (OSError, ValueError) as error:
        parser.exit(2, f"{error}\n")


if __name__ == "__main__":
    main()
