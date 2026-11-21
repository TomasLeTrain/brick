import {
  axisValue,
  controlLine,
  LineDecoder,
  LogCollector,
  parseTelemetry,
  parseCsv,
  correlationAt,
  RPM_STATUS,
} from "./core.mjs";
const $ = (id) => document.getElementById(id);
const colors = { teal: "#60e0c7", amber: "#f1bd6b", muted: "#91a3b6" };
let port = null,
  reader = null,
  writer = null,
  readTask = null,
  pumpTask = null;
let commands = [],
  pendingControl = null,
  session = 1,
  sequence = 0,
  armed = true;
let selectedPad = "",
  padSignature = "",
  buttonsBefore = [],
  orientation = -1;
let pendingOrientation = null,
  orientationAction = 0;
let latest = null,
  latestAt = 0,
  history = [],
  terminalLines = [],
  selectedLog = null,
  logRows = [];
let connecting = false,
  closing = false;
const captures = new LogCollector();
function notice(message, error = false) {
  $("notice").textContent = message;
  $("notice").classList.toggle("error", error);
}
function appendTerminal(line) {
  terminalLines.push(line);
  if (terminalLines.length > 800) terminalLines.shift();
}
function refreshConnection() {
  $("connect").disabled = !!port || connecting || closing;
  $("disconnect").disabled = !port || closing;
  $("serialState").textContent = port
    ? "Sender USB connected"
    : "Serial disconnected";
  $("serialDot").classList.toggle("on", !!port);
  $("arm").disabled = !port || !getPad();
  $("upright").disabled = $("inverted").disabled = !port;
  document
    .querySelectorAll("[data-command], #sendCommand")
    .forEach((b) => (b.disabled = !port));
  $("arm").textContent = armed
    ? "Disable companion heading"
    : "Enable companion heading";
  $("arm").classList.toggle("danger", armed);
  $("ownership").textContent = armed
    ? "Enabled by default. Connected, focused gamepad input takes heading; loss of focus, controller, or USB returns it to the radio. Input resumes automatically when available."
    : "Radio heading owns control. A 150 ms timeout returns it to the radio.";
  const requested = orientation < 0 ? latest?.sender_inverted : orientation;
  for (const [id, value] of [
    ["upright", 0],
    ["inverted", 1],
  ]) {
    $(id).classList.toggle(
      "selected",
      requested !== undefined && Number(requested) === value,
    );
    $(id).setAttribute(
      "aria-pressed",
      String(requested !== undefined && Number(requested) === value),
    );
  }
  $("orientationState").textContent =
    requested === undefined
      ? "Orientation not yet reported."
      : `Selected: ${requested ? "Inverted" : "Upright"} · robot: ${latest?.have_telemetry && latest.rx_age_ms < 1000 ? (latest.inverted ? "Inverted" : "Upright") : "not yet reported"}. The selected top face flashes for 300 ms.`;
}
function getPad() {
  return (
    Array.from(navigator.getGamepads?.() || []).find(
      (p) => p && p.connected !== false && String(p.index) === selectedPad,
    ) || null
  );
}
function effectiveTurn() {
  const pad = getPad();
  const deadman = Number($("deadman").value);
  const valid =
    armed &&
    pad &&
    Number.isFinite(pad.axes[Number($("axis").value)]) &&
    !document.hidden &&
    document.hasFocus() &&
    (deadman < 0 || pad.buttons[deadman]?.pressed);
  return {
    enabled: !!valid,
    turn: valid
      ? axisValue(
          pad.axes[Number($("axis").value)],
          Number($("deadzone").value),
          Number($("gain").value) / 100,
          $("invert").value === "1",
        )
      : 0,
  };
}
function queueControl(release = false) {
  if (!port) return;
  const { turn, enabled } = effectiveTurn();
  pendingControl = {
    turn: release ? 0 : turn,
    enabled: !release && enabled,
    at: performance.now(),
  };
  pump();
}
function pump() {
  if (pumpTask || !writer || !port) return;
  const activeWriter = writer;
  pumpTask = (async () => {
    while (
      port &&
      writer === activeWriter &&
      (commands.length || pendingControl)
    ) {
      await activeWriter.ready;
      let text = commands.shift(),
        sentAction = 0;
      if (!text && pendingControl) {
        const frame = pendingControl;
        pendingControl = null;
        if (performance.now() - frame.at > 80) continue;
        const selection = pendingOrientation;
        sentAction = selection === null ? 0 : orientationAction;
        text = controlLine(
          session,
          ++sequence,
          frame.turn,
          selection ?? -1,
          frame.enabled,
        );
      }
      if (text) {
        await activeWriter.write(new TextEncoder().encode(text));
        // Resolve selection at write time, so coalesced heartbeats cannot repeat
        // an action that has already reached the sender.
        if (sentAction && sentAction === orientationAction)
          pendingOrientation = null;
      }
    }
  })()
    .catch((error) => {
      if (port) {
        appendTerminal(`# Serial write failed: ${error.message}`);
        void disconnectSerial();
      }
    })
    .finally(() => {
      pumpTask = null;
    });
}
function command(c) {
  if (!port) {
    notice("Connect the sender before sending a command.", true);
    return;
  }
  if (
    c === "f" &&
    !confirm("Format the robot dump filesystem? This deletes saved captures.")
  )
    return;
  if (commands.length >= 16) {
    notice("Serial command queue is full.", true);
    return;
  }
  commands.push(c + "\n");
  appendTerminal("> " + c);
  pump();
}
async function connectSerial() {
  if (port || connecting || closing) return;
  if (!navigator.serial || !isSecureContext) {
    notice(
      "Use Chrome/Edge on localhost or HTTPS for Web Serial. Pair the Joy-Con in your OS first.",
      true,
    );
    return;
  }
  connecting = true;
  refreshConnection();
  try {
    const opened = await navigator.serial.requestPort();
    await opened.open({ baudRate: 115200 });
    port = opened;
    writer = opened.writable.getWriter();
    reader = opened.readable.getReader();
    session = crypto.getRandomValues(new Uint32Array(1))[0] || 1;
    sequence = 0;
    orientation = -1;
    pendingOrientation = null;
    orientationAction = 0;
    latest = null;
    latestAt = 0;
    commands = [];
    pendingControl = null;
    refreshConnection();
    notice(
      "USB connected. Companion heading is enabled by default; your saved controller mapping is restored. Upright/Inverted buttons also work with heading disabled.",
    );
    readTask = (async () => {
      const decoder = new TextDecoder(),
        lines = new LineDecoder();
      try {
        while (port === opened) {
          const { value, done } = await reader.read();
          if (done) break;
          for (const line of lines.push(
            decoder.decode(value, { stream: true }),
          ))
            receiveLine(line);
        }
      } catch (error) {
        if (port === opened)
          notice(`Serial disconnected: ${error.message}`, true);
      } finally {
        reader?.releaseLock();
        reader = null;
        if (port === opened) queueMicrotask(() => disconnectSerial());
      }
    })();
  } catch (error) {
    if (port) await disconnectSerial();
    notice(`Could not open serial: ${error.message}`, true);
  } finally {
    connecting = false;
    refreshConnection();
  }
}
async function disconnectSerial() {
  if (!port || closing) return;
  closing = true;
  const opened = port,
    oldReader = reader,
    oldWriter = writer;
  pendingControl = null;
  commands = [];
  port = null;
  if (captures.current)
    captures.current.error = "Serial disconnected before dump_end";
  refreshConnection();
  try {
    await oldReader?.cancel();
  } catch {}
  try {
    await readTask;
  } catch {}
  try {
    await oldWriter?.abort();
    await pumpTask;
    oldWriter?.releaseLock();
  } catch {}
  writer = null;
  try {
    await opened.close();
  } catch {}
  notice(
    "USB disconnected. Companion heading expires automatically; flip state remains latched on the sender.",
  );
  closing = false;
  refreshConnection();
  refreshLogs();
}
function receiveLine(line) {
  const telemetry = parseTelemetry(line);
  if (telemetry) {
    latest = telemetry;
    latestAt = performance.now();
    if (telemetry.have_telemetry && telemetry.rx_age_ms < 1000) {
      history.push({ ...telemetry, time: latestAt / 1000 });
      if (history.length > 600) history.shift();
    }
    refreshConnection();
  } else {
    appendTerminal(line);
    const changed = captures.line(line);
    if (changed) {
      selectedLog = changed;
      refreshLogs();
    }
  }
}
function refreshGamepads() {
  const pads = Array.from(navigator.getGamepads?.() || []).filter(
    (p) => p && p.connected !== false,
  );
  const signature = pads.map((p) => `${p.index}:${p.id}`).join("|");
  if (signature !== padSignature) {
    padSignature = signature;
    const prior = selectedPad;
    let remembered = "";
    try {
      remembered = localStorage.getItem("brick-controller-id") || "";
    } catch {}
    $("gamepad").replaceChildren();
    for (const pad of pads)
      $("gamepad").add(
        new Option(`${pad.index} · ${pad.id}`, String(pad.index)),
      );
    if (!pads.length) $("gamepad").add(new Option("No gamepad detected", ""));
    selectedPad = pads.some((p) => String(p.index) === prior)
      ? prior
      : String(
          pads.find((p) => p.id === remembered)?.index ?? pads[0]?.index ?? "",
        );
    $("gamepad").value = selectedPad;
    buttonsBefore = getPad()?.buttons.map((b) => b.pressed) || [];
    configureMappings();
    queueControl(true);
    refreshConnection();
  }
  const pad = getPad();
  if (!pad) {
    $("padPreview").textContent = "Waiting for controller input";
    return;
  }
  $("padPreview").textContent =
    `Axes ${pad.axes.map((v, i) => `${i}:${v.toFixed(2)}`).join(" ")}\nButtons ${
      pad.buttons
        .map((b, i) => (b.pressed ? i : null))
        .filter((i) => i !== null)
        .join(", ") || "none"
    }`;
  const down = pad.buttons.map((b) => b.pressed);
  if (port && !document.hidden && document.hasFocus()) {
    const up = Number($("uprightButton").value),
      downButton = Number($("invertedButton").value);
    if (
      up >= 0 &&
      down[up] &&
      !buttonsBefore[up] &&
      !(downButton >= 0 && down[downButton])
    )
      selectOrientation(0);
    if (
      downButton >= 0 &&
      down[downButton] &&
      !buttonsBefore[downButton] &&
      !(up >= 0 && down[up])
    )
      selectOrientation(1);
    for (const [id, action] of [
      ["captureButton", () => command("s")],
      ["exportButton", () => command("e")],
    ]) {
      const index = Number($(id).value);
      if (index >= 0 && down[index] && !buttonsBefore[index]) action();
    }
  }
  buttonsBefore = down;
}
function configureMappings() {
  const pad = getPad();
  let saved = {};
  try {
    const profile = pad
      ? localStorage.getItem("brick-mapping:" + pad.id)
      : null;
    saved = JSON.parse(
      profile || localStorage.getItem("brick-mapping") || "{}",
    );
    if (!saved || typeof saved !== "object") saved = {};
  } catch {}
  for (const id of [
    "axis",
    "deadman",
    "uprightButton",
    "invertedButton",
    "captureButton",
    "exportButton",
  ]) {
    const select = $(id),
      count = id === "axis" ? pad?.axes.length || 1 : pad?.buttons.length || 16;
    select.replaceChildren();
    if (id !== "axis")
      select.add(
        new Option(id === "deadman" ? "No hold button" : "Disabled", "-1"),
      );
    for (let i = 0; i < count; i++)
      select.add(
        new Option(`${id === "axis" ? "Axis" : "Button"} ${i}`, String(i)),
      );
    const defaults = {
      axis: 0,
      deadman: -1,
      uprightButton: 0,
      invertedButton: 1,
      captureButton: -1,
      exportButton: -1,
    };
    select.value = String(saved[id] ?? defaults[id]);
    if (select.selectedIndex < 0) select.selectedIndex = 0;
  }
  for (const id of ["gain", "deadzone", "invert"])
    if (saved[id] !== undefined) $(id).value = saved[id];
  $("gainValue").textContent = $("gain").value + "%";
}
function saveMapping() {
  if (
    $("uprightButton").value !== "-1" &&
    $("uprightButton").value === $("invertedButton").value
  ) {
    $("invertedButton").value = "-1";
    notice("Choose different buttons for Upright and Inverted.", true);
  }
  const ids = [
    "axis",
    "deadman",
    "uprightButton",
    "invertedButton",
    "captureButton",
    "exportButton",
    "gain",
    "deadzone",
    "invert",
  ];
  try {
    localStorage.setItem(
      getPad() ? "brick-mapping:" + getPad().id : "brick-mapping",
      JSON.stringify(Object.fromEntries(ids.map((id) => [id, $(id).value]))),
    );
    if (getPad()) localStorage.setItem("brick-controller-id", getPad().id);
  } catch {
    notice(
      "Browser storage is blocked; controller settings cannot be remembered.",
      true,
    );
  }
  queueControl(true);
  refreshConnection();
}
function selectOrientation(value) {
  if (!port) return;
  orientation = value;
  pendingOrientation = value;
  orientationAction++;
  queueControl();
  refreshConnection();
}

function metric(id, value, suffix = "") {
  $(id).replaceChildren(document.createTextNode(value));
  if (suffix) {
    const small = document.createElement("small");
    small.textContent = suffix;
    $(id).append(small);
  }
}
function updateDashboard() {
  const p = latest,
    usbFresh = port && p && performance.now() - latestAt < 1200;
  const fresh = usbFresh && p.link && p.have_telemetry && p.rx_age_ms < 1000;
  metric("rpm", fresh ? Math.round(p.rpm).toLocaleString() : "—", " RPM");
  metric(
    "target",
    fresh ? Math.round(p.target_rpm).toLocaleString() : "—",
    " RPM",
  );
  metric("rate", fresh ? Math.round(p.rssi_hz) : "—", " /s");
  metric(
    "correlation",
    fresh && p.correlation >= -1 ? p.correlation.toFixed(2) : "—",
  );
  $("rpmStatus").textContent = fresh
    ? RPM_STATUS[p.rpm_status] || `Status ${p.rpm_status}`
    : "Telemetry stale / unavailable";
  $("throttle").textContent = fresh
    ? `Throttle ${(p.throttle * 100).toFixed(1)}%`
    : "Throttle —";
  $("signal").textContent = fresh ? `Signal ${p.rssi_dbm} dBm` : "Signal —";
  $("rpmAge").textContent = fresh
    ? `Measurement age ${p.rpm_age_ms.toFixed(0)} ms`
    : "Measurement age —";
  const details = fresh
    ? {
        Robot:
          [
            "Network lost",
            "Radio controller lost",
            "Spin switch off",
            "Reserved",
            "Ready",
          ][p.connection_state] || p.connection_state,
        "Heading owner": p.companion_active ? "Companion" : "Radio",
        Orientation: p.inverted ? "Inverted" : "Upright",
        "Control age": `${p.control_age_ms} ms`,
        "RX queue drops": p.rx_dropped,
        "RSSI queue drops": p.rssi_dropped,
        "Sender TX failures": p.tx_failures,
        "Robot TX failures": p.robot_tx_failures,
        "Max send callback": `${(p.tx_max_us / 1000).toFixed(1)} ms`,
        "RF channel": p.channel,
      }
    : {
        Robot: "No fresh telemetry",
        "Companion lease":
          usbFresh && p.pc_active ? "Active on sender" : "Inactive",
      };
  $("linkDetails").replaceChildren();
  for (const [key, value] of Object.entries(details)) {
    const dt = document.createElement("dt"),
      dd = document.createElement("dd");
    dt.textContent = key;
    dd.textContent = value;
    $("linkDetails").append(dt, dd);
  }
  $("health").textContent = !fresh
    ? "Waiting for fresh robot telemetry"
    : !p.link || !p.controller_connected
      ? "Control link unavailable"
      : p.rssi_hz < 500
        ? "Low sample rate · RPM gate is 500 packets/s"
        : "Receiving robot telemetry";
  const end = performance.now() / 1000;
  drawPlot(
    $("liveRpm"),
    [
      { points: history.map((p) => [p.time, p.rpm]), color: colors.teal },
      {
        points: history.map((p) => [p.time, p.target_rpm]),
        color: colors.amber,
      },
    ],
    {
      x: [end - 60, end],
      y: [0, Math.max(3500, ...history.map((p) => p.rpm * 1.1))],
    },
  );
  drawPlot(
    $("liveRf"),
    [
      { points: history.map((p) => [p.time, p.rssi_hz]), color: colors.teal },
      {
        points: [
          [end - 60, 500],
          [end, 500],
        ],
        color: colors.amber,
      },
    ],
    {
      x: [end - 60, end],
      y: [0, Math.max(1100, ...history.map((p) => p.rssi_hz * 1.1))],
    },
  );
}
function drawPlot(canvas, series, opts = {}) {
  const rect = canvas.getBoundingClientRect();
  if (!rect.width) return;
  const ratio = devicePixelRatio || 1;
  canvas.width = Math.round(rect.width * ratio);
  canvas.height = Math.round(rect.height * ratio);
  const ctx = canvas.getContext("2d");
  ctx.scale(ratio, ratio);
  const w = rect.width,
    h = rect.height,
    left = 48,
    right = w - 12,
    top = 15,
    bottom = h - 28;
  const points = series
    .flatMap((s) => s.points)
    .filter(([x, y]) => Number.isFinite(x) && Number.isFinite(y));
  let xd = opts.x || [
    Math.min(...points.map((p) => p[0])),
    Math.max(...points.map((p) => p[0])),
  ];
  let yd = opts.y || [
    Math.min(...points.map((p) => p[1])),
    Math.max(...points.map((p) => p[1])),
  ];
  if (!Number.isFinite(xd[0])) xd = [0, 1];
  if (!Number.isFinite(yd[0])) yd = [0, 1];
  if (xd[0] === xd[1]) xd = [xd[0], xd[1] + 1];
  if (yd[0] === yd[1]) yd = [yd[0] - 1, yd[1] + 1];
  const X = (x) => left + ((x - xd[0]) / (xd[1] - xd[0])) * (right - left),
    Y = (y) => bottom - ((y - yd[0]) / (yd[1] - yd[0])) * (bottom - top);
  ctx.font = "10px system-ui";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const v = yd[0] + ((yd[1] - yd[0]) * i) / 4,
      y = Y(v);
    ctx.strokeStyle = "#2a3746";
    ctx.beginPath();
    ctx.moveTo(left, y);
    ctx.lineTo(right, y);
    ctx.stroke();
    ctx.fillStyle = "#91a3b6";
    ctx.textAlign = "right";
    ctx.fillText(
      Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(1),
      left - 8,
      y + 3,
    );
  }
  ctx.textAlign = "center";
  for (let i = 0; i <= 4; i++) {
    const x = xd[0] + ((xd[1] - xd[0]) * i) / 4;
    ctx.fillText((opts.absoluteX ? x : x - xd[0]).toFixed(1), X(x), h - 9);
  }
  ctx.save();
  ctx.beginPath();
  ctx.rect(left, top, right - left, bottom - top);
  ctx.clip();
  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    let started = false;
    for (const [x, y] of s.points) {
      if (!Number.isFinite(x) || !Number.isFinite(y)) {
        started = false;
        continue;
      }
      if (!started) {
        ctx.moveTo(X(x), Y(y));
        started = true;
      } else ctx.lineTo(X(x), Y(y));
    }
    ctx.stroke();
  }
  if (Number.isFinite(opts.cursor)) {
    ctx.strokeStyle = "#ccd9e6";
    ctx.setLineDash([3, 4]);
    ctx.beginPath();
    ctx.moveTo(X(opts.cursor), top);
    ctx.lineTo(X(opts.cursor), bottom);
    ctx.stroke();
  }
  ctx.restore();
  canvas.plotDomain = xd;
}
function refreshLogs() {
  $("logCount").textContent = captures.logs.length;
  $("logSelect").replaceChildren();
  captures.logs.forEach((log, i) =>
    $("logSelect").add(
      new Option(
        `${log.name} · ${log.complete ? "complete" : log.error ? "interrupted" : `${log.rows} rows`}`,
        String(i),
      ),
    ),
  );
  if (!captures.logs.length)
    $("logSelect").add(new Option("No captures yet", ""));
  if (selectedLog)
    $("logSelect").value = String(captures.logs.indexOf(selectedLog));
  const log = selectedLog;
  $("copyLog").disabled = $("saveLog").disabled = !log?.complete;
  $("logState").textContent = !log
    ? "Capture with s, stop the motors, then export with e."
    : log.error
      ? log.error
      : log.complete
        ? `${log.rows} records · complete CSV ready to copy/download`
        : `Receiving ${log.rows}${log.expected ? ` / ${log.expected}` : ""} records…`;
  if (log?.complete) {
    try {
      logRows = parseCsv(log.text);
      $("logPlots").classList.remove("hidden");
      const end = Math.max(...logRows.map((r) => r.time));
      $("cursor").max = end;
      $("cursor").value = end;
      renderLog();
    } catch (error) {
      notice(error.message, true);
      $("logPlots").classList.add("hidden");
    }
  } else {
    logRows = [];
    $("logPlots").classList.add("hidden");
  }
}
function renderLog() {
  if (!logRows.length) return;
  const t = Number($("cursor").value);
  $("cursorValue").textContent = t.toFixed(3) + " s";
  const states = logRows.filter((r) => r.kind === 2),
    radio = logRows.filter((r) => r.kind === 1);
  const domain = [0, Math.max(...logRows.map((r) => r.time))];
  drawPlot(
    $("logRpm"),
    [
      { points: states.map((r) => [r.time, r.rssi_rpm]), color: colors.teal },
      {
        points: states.map((r) => [r.time, r.target_rpm]),
        color: colors.amber,
      },
    ],
    { x: domain, cursor: t },
  );
  drawPlot(
    $("logRssi"),
    [{ points: radio.map((r) => [r.time, r.rssi_dbm]), color: colors.teal }],
    { x: domain, cursor: t },
  );
  drawPlot(
    $("logThrottle"),
    [{ points: states.map((r) => [r.time, r.throttle]), color: colors.amber }],
    { x: domain, y: [0, 1], cursor: t },
  );
  let index = 0;
  logRows.forEach((r, i) => {
    if (r.kind === 2 && r.time <= t) index = i;
  });
  const curve = correlationAt(logRows, index),
    row = logRows[index];
  drawPlot(
    $("correlationPlot"),
    [{ points: curve.map((v, i) => [i, v]), color: colors.teal }],
    { x: [8, 156], y: [-1, 1], absoluteX: true },
  );
  $("curveNote").textContent =
    `Lag · ms | RPM ${Math.round(row.rssi_rpm || 0)} | ${RPM_STATUS[row.rpm_status] || "offline curve"}`;
}
function download(text, name) {
  const a = document.createElement("a");
  a.href = URL.createObjectURL(
    new Blob([text], { type: "text/csv;charset=utf-8" }),
  );
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
async function copy(text) {
  try {
    await navigator.clipboard.writeText(text);
    notice("Copied to clipboard.");
  } catch {
    notice(
      "Clipboard unavailable. Use Download CSV, or select text in the serial terminal.",
      true,
    );
  }
}
$("connect").onclick = connectSerial;
$("disconnect").onclick = disconnectSerial;
$("arm").onclick = () => {
  armed = !armed;
  queueControl(!armed);
  refreshConnection();
};
$("upright").onclick = () => selectOrientation(0);
$("inverted").onclick = () => selectOrientation(1);
$("gamepad").onchange = () => {
  selectedPad = $("gamepad").value;
  buttonsBefore = getPad()?.buttons.map((b) => b.pressed) || [];
  configureMappings();
  try {
    if (getPad()) localStorage.setItem("brick-controller-id", getPad().id);
  } catch {}
  queueControl(true);
  refreshConnection();
};
for (const id of [
  "axis",
  "deadman",
  "uprightButton",
  "invertedButton",
  "captureButton",
  "exportButton",
  "gain",
  "deadzone",
  "invert",
])
  $(id).onchange = saveMapping;
$("gain").oninput = () => {
  $("gainValue").textContent = $("gain").value + "%";
};
$("deadzone").oninput = () => {
  $("deadzone").value = Math.max(
    0,
    Math.min(0.5, Number($("deadzone").value) || 0),
  );
};
for (const b of document.querySelectorAll("[data-command]"))
  b.onclick = () => command(b.dataset.command);
$("sendCommand").onclick = () => command($("command").value);
for (const b of document.querySelectorAll(".tab"))
  b.onclick = () => {
    document
      .querySelectorAll(".tab")
      .forEach((n) => n.classList.toggle("active", n === b));
    document
      .querySelectorAll(".view")
      .forEach((n) => n.classList.toggle("hidden", n.id !== b.dataset.view));
    if (b.dataset.view === "logs") renderLog();
  };
$("cursor").oninput = renderLog;
for (const id of ["logRpm", "logRssi", "logThrottle"])
  $(id).onclick = (e) => {
    const c = $(id),
      r = c.getBoundingClientRect();
    $("cursor").value =
      Math.max(0, Math.min(1, (e.clientX - r.left - 48) / (r.width - 60))) *
      Number($("cursor").max);
    renderLog();
  };
$("logSelect").onchange = () => {
  selectedLog = captures.logs[Number($("logSelect").value)];
  refreshLogs();
};
$("copyLog").onclick = () => selectedLog?.complete && copy(selectedLog.text);
$("saveLog").onclick = () =>
  selectedLog?.complete &&
  download(
    selectedLog.text,
    `brick-${new Date().toISOString().replaceAll(":", "-")}.csv`,
  );
$("copySerial").onclick = () => copy(terminalLines.join("\n"));
$("clearSerial").onclick = () => {
  terminalLines = [];
  $("terminal").textContent = "";
};
$("import").onclick = () => $("file").click();
$("file").onchange = async () => {
  const file = $("file").files[0];
  if (!file) return;
  try {
    if (file.size > 4 * 1024 * 1024) throw Error("CSV exceeds 4 MiB limit");
    const text = await file.text(),
      rows = parseCsv(text);
    selectedLog = { name: file.name, text, rows: rows.length, complete: true };
    captures.logs.push(selectedLog);
    if (captures.logs.length > 6) captures.logs.shift();
    refreshLogs();
  } catch (error) {
    notice(error.message, true);
  } finally {
    $("file").value = "";
  }
};
function release() {
  queueControl(true);
  refreshConnection();
}
function baselineButtons() {
  buttonsBefore = getPad()?.buttons.map((b) => b.pressed) || [];
}
window.addEventListener("blur", release);
window.addEventListener("focus", baselineButtons);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) release();
  else baselineButtons();
});
window.addEventListener("pagehide", release);
setInterval(() => {
  refreshGamepads();
  if (port && !document.hidden && document.hasFocus()) queueControl();
}, 20);
setInterval(() => {
  updateDashboard();
  const terminal = $("terminal");
  if (!$("serial").classList.contains("hidden")) {
    const follow =
      terminal.scrollHeight - terminal.scrollTop - terminal.clientHeight < 40;
    terminal.textContent = terminalLines.join("\n");
    if (follow) terminal.scrollTop = terminal.scrollHeight;
  }
}, 200);
window.addEventListener("resize", () => {
  updateDashboard();
  renderLog();
});
configureMappings();
refreshConnection();
updateDashboard();
if (!navigator.serial)
  notice(
    "Web Serial is unavailable in this browser. Open this page in desktop Chrome/Edge at localhost.",
    true,
  );
