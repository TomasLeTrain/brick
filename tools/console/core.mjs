export const signedDelta = (a, b) => ((a >>> 0) - (b >>> 0)) | 0;
export const RPM_STATUS = [
  "warming up",
  "low packet rate",
  "insufficient data",
  "weak signal",
  "no period",
  "confirming period",
  "tracking",
  "holding RPM",
  "stale",
  "period jump rejected",
];
export function axisValue(value, deadzone = 0.12, gain = 1, invert = false) {
  if (!Number.isFinite(value)) return 0;
  deadzone = Number.isFinite(deadzone)
    ? Math.max(0, Math.min(0.5, deadzone))
    : 0.12;
  gain = Number.isFinite(gain) ? Math.max(0, Math.min(1, gain)) : 0;
  const v = Math.max(-1, Math.min(1, value));
  return Math.round(
    (Math.abs(v) <= deadzone
      ? 0
      : (Math.sign(v) * (Math.abs(v) - deadzone)) / (1 - deadzone)) *
      gain *
      (invert ? -1 : 1) *
      1000,
  );
}
export function controlLine(session, sequence, turn, orientation, enabled) {
  if (
    !Number.isInteger(turn) ||
    Math.abs(turn) > 1000 ||
    ![-1, 0, 1].includes(orientation)
  )
    throw Error("Invalid control");
  return `@C,1,${session >>> 0},${sequence >>> 0},${turn},${orientation},${enabled ? 1 : 0}\n`;
}
export class LineDecoder {
  constructor(limit = 4096) {
    this.partial = "";
    this.limit = limit;
    this.dropping = false;
  }
  push(text) {
    const lines = [];
    for (const c of text) {
      if (c === "\n" || c === "\r") {
        if (!this.dropping && this.partial) lines.push(this.partial);
        this.partial = "";
        this.dropping = false;
      } else if (!this.dropping) {
        if (this.partial.length >= this.limit) {
          this.dropping = true;
          this.partial = "";
        } else this.partial += c;
      }
    }
    return lines;
  }
}
export class LogCollector {
  constructor() {
    this.logs = [];
    this.current = null;
  }
  line(line) {
    if (line.startsWith("# dump_begin")) {
      const expected = /count=(\d+)/.exec(line);
      const log = {
        name: `Run ${this.logs.length + 1}`,
        text: line + "\n",
        rows: 0,
        complete: false,
        expected: expected ? Number(expected[1]) : null,
        error: "",
      };
      if (this.current && !this.current.complete)
        this.current.error = "Interrupted before dump_end";
      this.current = log;
      this.logs.push(log);
      if (this.logs.length > 6) this.logs.shift();
      return log;
    }
    const log = this.current;
    if (!log || log.complete || log.error || line.startsWith("@T "))
      return null;
    if (
      !(line.startsWith("#") || line.startsWith("kind,") || /^[12],/.test(line))
    )
      return null;
    if (log.text.length + line.length > 4 * 1024 * 1024) {
      log.error = "Capture exceeds 4 MiB limit";
      return log;
    }
    log.text += line + "\n";
    if (/^[12],/.test(line)) log.rows++;
    if (line === "# dump_end") {
      log.complete = log.expected === null || log.expected === log.rows;
      if (!log.complete) log.error = "Record count mismatch";
      this.current = null;
    }
    return log;
  }
}
export const TELEMETRY_FIELDS = [
  "v",
  "link",
  "have_telemetry",
  "rx_age_ms",
  "rpm",
  "target_rpm",
  "throttle",
  "rssi_hz",
  "correlation",
  "heading_rad",
  "heading_error_rad",
  "rpm_age_ms",
  "rssi_dbm",
  "control_age_ms",
  "control_sequence",
  "connection_state",
  "rpm_status",
  "heading_status",
  "inverted",
  "companion_active",
  "controller_connected",
  "spin_enabled",
  "rx_dropped",
  "rssi_dropped",
  "robot_tx_failures",
  "tx_attempts",
  "tx_completed",
  "tx_failures",
  "tx_max_us",
  "pc_active",
  "sender_inverted",
  "channel",
];
export function parseTelemetry(line) {
  if (!line.startsWith("@T ")) return null;
  try {
    const p = JSON.parse(line.slice(3));
    if (p.v !== 1 || !TELEMETRY_FIELDS.every((k) => Number.isFinite(p[k])))
      return null;
    return p;
  } catch {
    return null;
  }
}
export function parseCsv(text) {
  if (text.length > 4 * 1024 * 1024) throw Error("CSV exceeds 4 MiB limit");
  let keys = null;
  const rows = [];
  for (const line of text.split(/\r?\n/)) {
    if (!line || line.startsWith("#") || line.startsWith("@T ")) continue;
    if (line.startsWith("kind,")) {
      if (keys) throw Error("Import one dump at a time");
      keys = line.split(",");
      continue;
    }
    if (!keys || !/^[12],/.test(line)) continue;
    const values = line.split(",");
    if (values.length !== keys.length) throw Error("Incomplete CSV row");
    const row = Object.fromEntries(
      keys.map((key, i) => [key, Number(values[i])]),
    );
    if (![1, 2].includes(row.kind) || !Number.isFinite(row.timestamp_us))
      throw Error("Invalid event");
    if (rows.length >= 50000) throw Error("CSV exceeds 50,000 event limit");
    rows.push(row);
  }
  if (!rows.length) throw Error("No RSSI/control rows found");
  let priorReceived,
    localTime = 0,
    priorRadio,
    radioTime = 0,
    anchor;
  for (const row of rows) {
    const received = row.received_us ?? row.timestamp_us;
    localTime =
      priorReceived === undefined
        ? received
        : localTime + signedDelta(received, priorReceived);
    priorReceived = received;
    if (row.kind === 1) {
      radioTime =
        priorRadio === undefined
          ? row.timestamp_us
          : radioTime + signedDelta(row.timestamp_us, priorRadio);
      priorRadio = row.timestamp_us;
      if (anchor === undefined) anchor = localTime - radioTime;
      row.time = radioTime + anchor;
    } else
      row.time =
        row.received_us === undefined
          ? localTime
          : localTime + signedDelta(row.timestamp_us, received);
  }
  const origin = Math.min(...rows.map((r) => r.time));
  rows.forEach((r) => {
    r.time = (r.time - origin) / 1e6;
  });
  return rows;
}
// Desktop reconstruction of the same gap-masked, detrended normalized curve
// used by the firmware. JS uses doubles, so tiny float-rounding differences apply.
export function correlationAt(rows, index) {
  const row = rows[index],
    packets = [];
  let prior, elapsed;
  for (let i = 0; i <= index; i++) {
    const r = rows[i];
    if (r.kind !== 1 || !Number.isFinite(r.rssi_dbm)) continue;
    const dt = prior === undefined ? 0 : signedDelta(r.timestamp_us, prior);
    if (prior !== undefined && dt <= 0) continue;
    if (dt >= 50000) packets.length = 0;
    elapsed = elapsed === undefined ? r.timestamp_us : elapsed + dt;
    packets.push([elapsed, r.rssi_dbm]);
    prior = r.timestamp_us;
  }
  const result = Array(157).fill(NaN);
  if (!packets.length) return result;
  const end = row.analysis_timestamp_us
    ? elapsed + signedDelta(row.analysis_timestamp_us, prior)
    : packets[0][0] + Math.floor((elapsed - packets[0][0]) / 1000) * 1000;
  const grid = Array(406).fill(NaN),
    start = end - 405000;
  let p = 0;
  for (let i = 0; i < grid.length; i++) {
    const t = start + i * 1000;
    while (p + 1 < packets.length && packets[p + 1][0] <= t) p++;
    if (packets[p][0] === t) grid[i] = packets[p][1];
    else if (p + 1 < packets.length) {
      const [a, x] = packets[p],
        [b, y] = packets[p + 1];
      if (a <= t && t <= b && b - a <= 8000)
        grid[i] = x + ((t - a) / (b - a)) * (y - x);
    }
  }
  const n = row.comparison_samples || 250,
    first = row.min_lag || 9,
    last = row.max_lag || 155;
  if (n < 2 || n > 250 || first < 9 || last > 155 || first > last)
    return result;
  const span = n + last + 1,
    w = grid.slice(-span).map((v) => v + 64);
  let count = 0,
    st = 0,
    sw = 0,
    stt = 0,
    stw = 0;
  w.forEach((v, i) => {
    if (Number.isFinite(v)) {
      const t = i - (span - 1) / 2;
      count++;
      st += t;
      sw += v;
      stt += t * t;
      stw += t * v;
    }
  });
  if (count < Math.floor(span * 0.8)) return result;
  const slope = (stw - (st * sw) / count) / (stt - (st * st) / count);
  w.forEach((v, i) => {
    w[i] = v - slope * (i - (span - 1) / 2);
  });
  for (let lag = first - 1; lag <= last + 1; lag++) {
    let count = 0,
      sx = 0,
      sy = 0,
      xx = 0,
      yy = 0,
      xy = 0;
    for (let j = last + 1; j < span; j++) {
      const x = w[j],
        y = w[j - lag];
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      count++;
      sx += x;
      sy += y;
      xx += x * x;
      yy += y * y;
      xy += x * y;
    }
    const vx = xx - (sx * sx) / count,
      vy = yy - (sy * sy) / count;
    if (count >= Math.floor(n * 0.8) && vx >= count && vy >= count)
      result[lag] = (xy - (sx * sy) / count) / Math.sqrt(vx * vy);
  }
  return result;
}
