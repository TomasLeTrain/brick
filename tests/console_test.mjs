import { test } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import {
  axisValue,
  restoredHeadingGain,
  controlLine,
  LineDecoder,
  LogCollector,
  parseTelemetry,
  TELEMETRY_FIELDS,
  parseCsv,
  correlationAt,
} from "../tools/console/core.mjs";

test("heading mapping is bounded, centered and reversible", () => {
  assert.equal(axisValue(0.1), 0);
  assert.equal(axisValue(-1), -1000);
  assert.equal(axisValue(2), 1000);
  assert.equal(axisValue(0.56, 0.12, 0.5), 250);
  assert.equal(axisValue(0.56, 0.12, 0.5, true), -250);
  assert.equal(axisValue(NaN), 0);
  assert.equal(axisValue(1, NaN, NaN), 0);
  assert.equal(controlLine(1, 4, -500, 1, true), "@C,1,1,4,-500,1,1\n");
  assert.throws(() => controlLine(1, 4, 1001, 1, true));
});
test("fragmented serial and overflow cannot corrupt subsequent lines", () => {
  const d = new LineDecoder(16);
  assert.deepEqual(d.push("abc"), []);
  assert.deepEqual(d.push("d\r\nx\n"), ["abcd", "x"]);
  assert.deepEqual(d.push("a".repeat(100) + "\nok\n"), ["ok"]);
});
test("captures require dump_end and a matching row count", () => {
  const c = new LogCollector();
  c.line("# dump_begin generation=1 count=2");
  c.line("kind,timestamp_us");
  c.line("1,10");
  assert.equal(c.logs[0].complete, false);
  c.line("@T {}");
  c.line("2,20");
  c.line("# dump_end");
  assert.equal(c.logs[0].complete, true);
  assert(!c.logs[0].text.includes("@T"));
  c.line("# dump_begin count=2");
  c.line("1,10");
  c.line("# dump_end");
  assert.match(c.logs[1].error, /count mismatch/);
  assert.equal(c.logs[1].complete, false);
});
test("telemetry rejects bad or incompatible data", () => {
  const valid = {
    ...Object.fromEntries(TELEMETRY_FIELDS.map((k) => [k, 0])),
    v: 1,
    rpm: 1000,
    target_rpm: 1200,
    rssi_hz: 800,
    throttle: 0.2,
    rx_age_ms: 10,
  };
  assert.equal(parseTelemetry("@T " + JSON.stringify(valid)).rpm, 1000);
  assert.equal(parseTelemetry('@T {"v":2}'), null);
  assert.equal(parseTelemetry("@T broken"), null);
  assert.equal(
    parseTelemetry("@T " + JSON.stringify({ ...valid, correlation: null })),
    null,
  );
});
test("real CSV and correlation survive older clock domains and clean oscillations", () => {
  const rows = parseCsv(
    fs.readFileSync(new URL("../logs/log18.csv", import.meta.url), "utf8"),
  );
  assert.equal(rows.length, 2048);
  const end = rows.findLastIndex((r) => r.kind === 2);
  const curve = correlationAt(rows, end);
  assert(Math.max(...curve.filter(Number.isFinite)) > 0.8);
  assert.throws(
    () => parseCsv("kind,timestamp_us,rssi_dbm\n1,10\n"),
    /Incomplete/,
  );
  assert.throws(() => parseCsv("just words"), /No RSSI/);
});

test("heading sensitivity defaults low and migrates only the old full-speed default", () => {
  assert.equal(restoredHeadingGain({}), 25);
  assert.equal(restoredHeadingGain({ gain: "100", axis: "1" }), 25);
  assert.equal(restoredHeadingGain({ gain: "100", gainVersion: 2 }), 100);
  assert.equal(restoredHeadingGain({ gain: "65" }), 65);
  assert.equal(restoredHeadingGain({ gain: "bad" }), 25);
  assert.equal(restoredHeadingGain({ gain: "0" }), 1);
  assert(Math.abs(axisValue(0.2, 0.12, 0.25)) > 0);
});
