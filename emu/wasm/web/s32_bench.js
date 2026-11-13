// SPDX-License-Identifier: Apache-2.0
/* What the bench page (index.html) and the cockpit page (cockpit.html) share: the scenario table and its runner, the
 * bench's default controls, the panel renderer, the firmware file loader and the default-firmware fetch. A plain
 * script (not a module): it defines globals, loaded before each page's own script and after s32_wasm.js. */

/* Scenarios: named bench states that a page can link to (?scenario=<id>, optionally &autostart=1) or a visitor picks
 * from a list. Each one starts from the same power-up (the default controls with the engine stopped, a power cycle with
 * the configuration kept, the home page, fuel trim 0 %) and then runs its steps in order:
 *   ["preset", "cruise" | "takeoff"]   the bench page's engine presets
 *   ["rpm", n]                          engine speed only
 *   ["set", id, value]                  one bench control by its id on the bench page (a slider, box or list)
 *   ["trim", pct]                       turn the knob until both ECUs report that fuel trim
 *   ["run", secs]                       run the bench for that long
 *   ["power-cycle"]                     turn both ECUs and the Controller off and on, configuration kept
 *   ["ecu-setting", n, "text"]          write ECU setting n (1..28) into both ECUs' configuration chips, as the
 *                                       documented layout holds it (model/s32_fram.ksy); the ECUs read it at their next
 *                                       power-up. The next scenario puts the old value back.
 * "blank": the configuration chips are erased before the power-up instead of kept. After the steps the bench pauses,
 * unless the link says autostart=1 or the visitor picked the scenario from the list.
 *
 * For the cockpit page each scenario also has:
 *   quadrant  where the cockpit page sets its throttle and prop levers when a step sets the engine ("preset", "rpm",
 *             or the RPM, MAP and throttle sliders): throttle and prop in % of lever travel, or rpm (the prop lever's
 *             governed RPM) and map (the throttle that gives this MAP at that RPM); stop ("correct" or "fine"), the
 *             propeller's low-pitch stop. The cockpit page's engine model (cockpit.html, ENGINE) then sets RPM and MAP
 *             from the levers and the sparks.
 *   shows     what the scenario shows, for the pilot guide (not shown on the pages)
 *   guide     a link to the pilot guide's page on it ("" for none yet)
 *   pilot     checklist lines for the pilot: say (the item), watch (what follows, a few words) and, for an item the
 *             pilot cannot do from the cockpit, act (the label of the button that does it) and do (the steps it runs).
 * Every "watch" line is what the cockpit page shows with the default firmware; scripts/bench_scenarios.py checks each
 * one. */
const CRUISE = { rpm: 2400, map: 24.0 };

// The state the emulator opens in, and that Reset returns to: the engine running at idle (prop forward), both
// ignitions on, ECU 1 selected, Fuel Pump 1. Not a preset of its own.
const DEFAULT_STATE = { id: "default", title: "Default", steps: [["rpm", 750], ["run", 2]],
                        quadrant: { throttle: 0, prop: 100 } };
const SCENARIOS = [
  { id: "first-start", title: "First start", blank: true,
    description: "Blank configuration chips, first power-up, engine stopped.",
    steps: [["run", 1]],
    shows: "First power-up of a new system; the ECUs fire from the next power-up. Then the first engine start.",
    guide: "",
    pilot: [
      { say: "Look at the tabs", watch: "ECU1 and ECU2 red." },
      { say: "Press MENU", watch: "Main Menu." },
      { say: "Press MONITOR", watch: "Gauges." },
      { say: "Cycle power", act: "Cycle power", do: [["power-cycle"]], watch: "ECU1 blue, ECU2 green." },
      { say: "Starter, hold 2 s", watch: "Engine runs. RPM ~750." },
      { say: "Throttle up", watch: "RPM rises." },
    ] },
  { id: "runup", title: "Run-up",
    description: "1800 RPM, both ECUs firing, ECU 1 selected. Try Ignition Enable, the fuel pumps and ECU Select.",
    steps: [["rpm", 1800], ["set", "sl-map", 20.0], ["set", "sl-tps", 30], ["run", 3]],
    quadrant: { throttle: 25, prop: 100 },
    shows: "Run-up at 1800 RPM, prop full forward. Both ECUs fire, ECU 1 fuels, Fuel Pump 1 runs.",
    guide: "",
    pilot: [
      { say: "Ignition 1 OFF", watch: "ECU1 red. RPM drops ~100." },
      { say: "Ignition 1 ON", watch: "ECU1 blue. RPM 1800." },
      { say: "Ignition 2 OFF", watch: "ECU2 red. RPM drops ~100." },
      { say: "Ignition 2 ON", watch: "ECU2 green. RPM 1800." },
      { say: "Fuel Pump 2", watch: "PMP2 yellow, PMP1 gray." },
      { say: "ECU Select 2", watch: "ECU2 blue, ECU1 green." },
    ] },
  { id: "cruise", title: "Cruise",
    description: "2400 RPM, 24.0 inHg, fuel trim 0 %.",
    steps: [["preset", "cruise"], ["run", 3]],
    quadrant: CRUISE,
    shows: "Cruise at 2400 RPM and 24 inHg, fuel trim 0 %.",
    guide: "",
    pilot: [
      { say: "Ignition 1 OFF", watch: "ECU1 red. RPM stays 2400." },
      { say: "Ignition 1 ON", watch: "ECU1 blue." },
      { say: "Knob 5 clicks left", watch: "Trim -5%. AFR ~11.7." },
      { say: "Knob 5 clicks right", watch: "Trim 0%. AFR ~11.1." },
      { say: "Press MONITOR", watch: "Page 2." },
      { say: "Press MONITOR", watch: "ECU2 numbers." },
    ] },
  { id: "takeoff", title: "Takeoff",
    description: "2700 RPM, 29.0 inHg, full throttle.",
    steps: [["preset", "takeoff"], ["run", 3]],
    quadrant: { throttle: 100, prop: 100 },
    shows: "Takeoff: throttle and prop full forward, 2700 RPM, 29 inHg.",
    guide: "",
    pilot: [
      { say: "RPM and MAP", watch: "RPM 2700, MAP 29." },
      { say: "Press MENU", watch: "Menu locked while running." },
      { say: "Press MONITOR", watch: "Gauges." },
      { say: "Press TUNE", watch: "Tuning Menu." },
      { say: "Press MONITOR", watch: "Gauges." },
      { say: "Throttle back to MAP 24", watch: "RPM ~2600." },
      { say: "Prop back to 2400 RPM", watch: "RPM 2400." },
    ] },
  { id: "ecu-silent", title: "ECU 1 silent",
    description: "Cruise with ECU 1 powered but its data link silent: red ECU1 tab, dashes for its values.",
    steps: [["preset", "cruise"], ["run", 1], ["set", "sw-e1-lnk", false], ["run", 4]],
    quadrant: CRUISE,
    shows: "ECU 1 runs, but the Controller hears nothing from it, as with a broken data cable.",
    guide: "",
    pilot: [
      { say: "Look at ECU1", watch: "Red. Dashes for its values." },
      { say: "ECU Select 2", watch: "ECU2 blue." },
      { say: "Press MONITOR twice", watch: "ECU2 numbers." },
    ] },
  { id: "map-fail", title: "MAP sensor failed",
    description: "Cruise with ECU 1's MAP sensor disconnected. Select ECU 2 to fuel from the other sensor.",
    steps: [["preset", "cruise"], ["run", 1], ["set", "sel-e1-map", "open"], ["run", 3]],
    quadrant: CRUISE,
    shows: "ECU 1's manifold pressure sensor has failed; ECU 1 fuels from a wrong reading until ECU 2 is selected.",
    guide: "",
    pilot: [
      { say: "Read MAP and AFR", watch: "MAP low. AFR 20.0." },
      { say: "ECU Select 2", watch: "ECU2 blue. AFR ~11." },
      { say: "Press MONITOR twice", watch: "ECU2 MAP 24." },
    ] },
  { id: "ecu-select", title: "ECU 2 fuels",
    description: "Cruise with ECU Select on ECU 2: ECU 2 fuels the engine (blue ECU2 tab).",
    steps: [["preset", "cruise"], ["run", 1], ["set", "sel-ecu-select", "2"], ["run", 3]],
    quadrant: CRUISE,
    shows: "Cruise with ECU Select on 2: ECU 2 fuels, ECU 1 still fires.",
    guide: "",
    pilot: [
      { say: "Look at the tabs", watch: "ECU2 blue, ECU1 green." },
      { say: "Ignition 2 OFF", watch: "ECU2 red. RPM stays 2400." },
      { say: "Ignition 2 ON", watch: "ECU2 blue." },
      { say: "ECU Select 1", watch: "ECU1 blue, ECU2 green." },
    ] },
  { id: "rpm-limit", title: "RPM limit",
    description: "RPM Limit set to 2750, engine at 2700. Move the RPM slider to 2750: the ignition cuts.",
    steps: [["ecu-setting", 7, "2750"], ["power-cycle"], ["preset", "cruise"], ["rpm", 2700], ["run", 3]],
    quadrant: { throttle: 25, prop: 100, stop: "fine" },
    shows: "RPM limit 2750, fine low-pitch stop. Throttle full, fast: RPM overshoots, the ECUs cut spark at 2750, then 2700.",
    guide: "",
    pilot: [
      { say: "Throttle full, fast", watch: "RPM ~2775, spark cut at 2750, then 2700." },
    ] },
  { id: "trim-lop", title: "Lean trim",
    description: "Cruise with the fuel trim turned to -28 %: the bench AFR reads about 15.5.",
    steps: [["preset", "cruise"], ["trim", -28], ["run", 3]],
    quadrant: CRUISE,
    shows: "Cruise with the fuel trim at -28 %: AFR about 15.5.",
    guide: "",
    pilot: [
      { say: "Read Fuel Trim and AFR", watch: "-28%. AFR ~15.5." },
      { say: "Knob right to 0%", watch: "AFR ~11." },
    ] },
];

// The Controller panel (320 x 240 RGBA from the module) on a canvas: WebGL when the browser has it, else Canvas2D.
// mode names the renderer in use.
function s32CreateLcd(canvas) {
  const glOpts = { alpha: false, depth: false, stencil: false, antialias: false, preserveDrawingBuffer: true,
                   powerPreference: "high-performance", desynchronized: true };
  const gl = canvas.getContext("webgl2", glOpts) || canvas.getContext("webgl", glOpts);
  if (gl) {
    const isWebGL2 = (typeof WebGL2RenderingContext !== "undefined") && (gl instanceof WebGL2RenderingContext);
    const vsSrc = `
      attribute vec2 a_pos;
      varying vec2 v_uv;
      void main() {
        v_uv = vec2((a_pos.x + 1.0) * 0.5, (1.0 - a_pos.y) * 0.5);
        gl_Position = vec4(a_pos, 0.0, 1.0);
      }
    `;
    const fsSrc = `
      precision mediump float;
      varying vec2 v_uv;
      uniform sampler2D u_tex;
      void main() {
        gl_FragColor = texture2D(u_tex, v_uv);
      }
    `;
    const compile = (type, src) => {
      const sh = gl.createShader(type);
      gl.shaderSource(sh, src);
      gl.compileShader(sh);
      return gl.getShaderParameter(sh, gl.COMPILE_STATUS) ? sh : null;
    };
    const vs = compile(gl.VERTEX_SHADER, vsSrc);
    const fs = compile(gl.FRAGMENT_SHADER, fsSrc);
    const prog = gl.createProgram();
    if (vs && fs && prog) {
      gl.attachShader(prog, vs);
      gl.attachShader(prog, fs);
      gl.linkProgram(prog);
      if (gl.getProgramParameter(prog, gl.LINK_STATUS)) {
        gl.useProgram(prog);
        const buf = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, buf);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]), gl.STATIC_DRAW);
        const loc = gl.getAttribLocation(prog, "a_pos");
        gl.enableVertexAttribArray(loc);
        gl.vertexAttribPointer(loc, 2, gl.FLOAT, false, 0, 0);

        const tex = gl.createTexture();
        gl.bindTexture(gl.TEXTURE_2D, tex);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
        gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
        gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, 320, 240, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
        gl.viewport(0, 0, canvas.width, canvas.height);
        return {
          mode: isWebGL2 ? "WebGL2 GPU" : "WebGL GPU",
          draw(M, rgbaPtr) {
            const pixels = new Uint8Array(M.HEAPU8.buffer, rgbaPtr, 320 * 240 * 4);
            gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, 320, 240, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
            gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
          },
        };
      }
    }
  }
  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(320, 240);
  return {
    mode: "Canvas2D",
    draw(M, rgbaPtr) {
      img.data.set(new Uint8ClampedArray(M.HEAPU8.buffer, rgbaPtr, 320 * 240 * 4));
      ctx.putImageData(img, 0, 0);
    },
  };
}

function s32Telemetry(M) {
  return JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
}

// A harness knob by name (emu/common/s32board.c), as the command line sets it from the environment.
function s32SetKnob(M, name, value) {
  return M.ccall("s32_wasm_knob", "number", ["string", "string"], [name, value]);
}

// Firmware files by name: a Controller image, an ECU image.
const s32IsCtName = (u) => (u.startsWith("CT") || u.includes("PHOENIX_CT") || u.endsWith("_CT_PROG.BIN")) &&
                           (u.endsWith(".BIN") || u.endsWith(".HEX"));
const s32IsEcuName = (u) => !s32IsCtName(u) && (u.endsWith("_BFLASH.BIN") || u.endsWith("_PFLASH.BIN") || u.endsWith(".FLAT") ||
                            ((u.startsWith("EC") || u.includes("PHOENIX_EC")) && (u.endsWith(".BIN") || u.endsWith(".HEX"))));

function s32WriteBytes(M, uint8, fn, extraArgs = []) {
  const ptr = M._malloc(uint8.length);
  M.HEAPU8.set(uint8, ptr);
  const rc = fn(ptr, uint8.length, ...extraArgs);
  M._free(ptr);
  return rc;
}

function s32KeyHint(name) {
  const m = /^EC(\d{3})(\d{3})\.bin$/i.exec(name.trim());
  if (!m) return -1;
  return (parseInt(m[1], 10) * 1000 + parseInt(m[2], 10)) & 0xFF;
}

// One firmware, configuration or card file into the module, by its name. True when it was loaded.
function s32LoadFile(M, name, buf) {
  const upper = name.toUpperCase();
  if (s32IsCtName(upper)) return s32WriteBytes(M, buf, M._s32_wasm_load_ct_firmware) > 0;
  if (upper.endsWith(".FLAT")) {                      // application only: the ECUs start it directly
    s32WriteBytes(M, buf, M._s32_wasm_load_ec_pflash);
    return true;
  }
  if (upper.endsWith("_BFLASH.BIN")) { s32WriteBytes(M, buf, M._s32_wasm_load_ec_bflash); return true; }
  if (upper.endsWith("_PFLASH.BIN")) { s32WriteBytes(M, buf, M._s32_wasm_load_ec_pflash); return true; }
  if ((upper.startsWith("EC") || upper.includes("PHOENIX_EC")) && (upper.endsWith(".BIN") || upper.endsWith(".HEX"))) {
    return s32WriteBytes(M, buf, M._s32_wasm_load_ec_firmware, [s32KeyHint(name)]) >= 0;
  }
  if (upper.endsWith(".FRAM") || upper.endsWith(".CFG")) {
    const fram = (target) => s32WriteBytes(M, buf, (p, n) => M._s32_wasm_load_fram(target, p, n));
    if (upper.includes("CT") || upper.includes("CONTROLLER")) fram(0);
    else if (upper.includes("ECU2") || upper.includes("E2")) fram(2);
    else if (upper.includes("ECU1") || upper.includes("E1")) fram(1);
    else { fram(1); fram(2); }
    return true;
  }
  if (upper.endsWith(".IMG") || upper.endsWith(".FAT")) { s32WriteBytes(M, buf, M._s32_wasm_load_sd); return true; }
  return false;
}

// Default firmware. The manifest names the files, their SHA-256 and, optionally, how the page labels them ("label",
// "note"): the emulator repository's `make bench-site` writes one for the stock v86 update files, which are FlyEFII's
// copyright and come from release assets (firmware/manifest.json, firmware/LICENSE-FIRMWARE). A site that serves the
// pages may write its own firmware/manifest.json next to them. With no manifest nothing is loaded (null): bring your
// own firmware. Each file is checked against its SHA-256 before anything is loaded; if any check fails, this throws and
// nothing is loaded.
async function s32FetchDefaultFirmware() {
  let manifest;
  try {
    const r = await fetch("firmware/manifest.json", { cache: "no-cache" });
    if (!r.ok) return null;
    manifest = await r.json();
  } catch (e) { return null; }
  const files = [];
  for (const f of manifest.files) {
    const r = await fetch("firmware/" + encodeURIComponent(f.name), { cache: "no-cache" });
    if (!r.ok) throw new Error(f.name + ": HTTP " + r.status);
    const buf = await r.arrayBuffer();
    if (!(window.crypto && crypto.subtle)) throw new Error("no SubtleCrypto here (use https or localhost) to check " + f.name);
    const digest = Array.from(new Uint8Array(await crypto.subtle.digest("SHA-256", buf)), (b) => b.toString(16).padStart(2, "0")).join("");
    if (digest !== f.sha256) throw new Error(f.name + ": SHA-256 does not match the manifest");
    files.push(new File([buf], f.name));
  }
  const ver = manifest.version ? ` ${manifest.version}` : "";
  const label = manifest.label || `default firmware${ver}` + (manifest.source_commit ? ` (build ${String(manifest.source_commit).slice(0, 12)})` : "");
  return { files, label, note: manifest.note || null };
}

/* The scenario runner. The page hands it:
 *   M()                  the module          booted()         true once the bench booted
 *   reboot()             power-cycle the bench with the page's controls (configuration kept)
 *   stepSim(quanta)      optional: run the bench (0.1 ms quanta), for a page that updates its own models as it goes
 *   draw()               draw the panel      refresh()        update the page's readouts from the telemetry
 *   resetControls(blank) the page's controls back to BENCH_DEFAULTS, engine stopped (blank: the configuration the
 *                        firmware writes on a blank chip, a 4-cylinder engine)
 *   actions              preset, rpm and set (see SCENARIOS)
 *   setSpeed(s)          0 pauses, 1 runs at 1x
 *   note(kind, sc, err)  tell the visitor: "unknown", "no-firmware", "setup", "paused", "running" or "error"
 *   started(sc), finished(sc)   optional
 * and gets back run(id or scenario, keepRunning), act(list) (steps from a pilot step's "do", at the current state) and busy().
 * window.S32_SCENARIO = { id, state } tells a test or an embedding page where it is: running, done, unknown,
 * no-firmware or error. */
function s32ScenarioRunner(host) {
  let busy = false;

  // Run the bench for secs of emulated time in short slices, drawing the panel between them.
  async function advance(secs) {
    const M = host.M();
    for (let left = Math.round(secs * 10000); left > 0;) {
      const n = Math.min(2500, left);
      if (host.stepSim) host.stepSim(n);
      else M._s32_wasm_step_quanta(n);
      left -= n;
      host.draw();
      await new Promise((r) => setTimeout(r, 0));
    }
    host.refresh();
  }

  async function setTrim(target) {
    const M = host.M();
    for (let tries = 0; tries < 4; tries++) {
      const trim = s32Telemetry(M).trim_pct;
      if (trim[0] === target && trim[1] === target) return;
      if (trim[0] !== target) M._s32_wasm_encoder_turn(target - trim[0]);
      await advance(Math.abs(target - trim[0]) * 0.135 + 1.5);   /* one detent per 0.135 s, then the status frames */
    }
    throw new Error(`the fuel trim did not reach ${target} %`);
  }

  async function powerUp() {
    const M = host.M();
    host.reboot();
    if (!host.booted()) throw new Error("the bench did not boot");
    for (let waited = 0; M._s32_wasm_have_ct() && M._s32_wasm_home_seen() < 0 && waited < 20; waited += 0.5) await advance(0.5);
    await advance(1.0);
  }

  // ECU settings in the configuration chips (model/s32_fram.ksy): 16-byte NUL-padded text slots from 0x1480, a 16-bit
  // byte sum of 0x1480..0x187F at 0x1890. undo holds what the last scenario's "ecu-setting" steps replaced.
  let undo = [];
  function writeSlot(k, n, bytes) {
    const M = host.M(), p = M._s32_wasm_get_fram_ptr(k);
    M.HEAPU8.set(bytes, p + 0x1480 + (n - 1) * 16);
    let sum = 0;
    for (let a = 0x1480; a < 0x1880; a++) sum += M.HEAPU8[p + a];
    M.HEAPU8[p + 0x1890] = sum & 0xff;
    M.HEAPU8[p + 0x1891] = (sum >> 8) & 0xff;
  }
  function ecuSetting(n, text) {
    const M = host.M();
    const bytes = Array.from({ length: 16 }, (_, i) => (i < text.length ? text.charCodeAt(i) : 0));
    for (const k of [1, 2]) {
      const at = M._s32_wasm_get_fram_ptr(k) + 0x1480 + (n - 1) * 16;
      if (!undo.some(([kk, m]) => kk === k && m === n)) undo.push([k, n, M.HEAPU8.slice(at, at + 16)]);
      writeSlot(k, n, bytes);
    }
  }

  async function resetBench(blank) {
    if (!blank) for (const [k, n, bytes] of undo.reverse()) writeSlot(k, n, bytes);
    undo = [];
    host.resetControls(blank);
    if (blank) for (const target of [0, 1, 2]) host.M()._s32_wasm_clear_fram(target);
    await powerUp();
    if (!blank) await setTrim(0);
  }

  const actions = () => ({ ...host.actions, trim: setTrim, run: advance, "power-cycle": powerUp,
                           "ecu-setting": ecuSetting });

  async function runSteps(list) {
    const a = actions();
    for (const [action, ...args] of list) await a[action](...args);
  }

  async function run(id, keepRunning) {
    const sc = typeof id === "object" ? id : SCENARIOS.find((x) => x.id === id);
    if (typeof id === "object") id = sc.id;
    if (!sc) {
      host.note("unknown", { id });
      window.S32_SCENARIO = { id, state: "unknown" };
      return;
    }
    if (!host.M() || !host.booted()) {
      host.note("no-firmware", sc);
      window.S32_SCENARIO = { id, state: "no-firmware" };
      return;
    }
    if (busy) return;
    busy = true;
    window.S32_SCENARIO = { id, state: "running" };
    if (host.started) host.started(sc);
    host.note("setup", sc);
    host.setSpeed(0);
    try {
      await resetBench(!!sc.blank);
      await runSteps(sc.steps);
      if (host.finished) host.finished(sc);
      host.setSpeed(keepRunning ? 1 : 0);
      host.note(keepRunning ? "running" : "paused", sc);
      window.S32_SCENARIO = { id, state: "done" };
    } catch (e) {
      host.setSpeed(1);
      host.note("error", sc, e);
      window.S32_SCENARIO = { id, state: "error", error: e.message };
    } finally {
      busy = false;
    }
  }

  async function act(list) {
    if (busy || !host.M() || !host.booted()) return false;
    busy = true;
    try {
      await runSteps(list);
    } finally {
      busy = false;
    }
    return true;
  }

  return { run, act, busy: () => busy };
}

/* ---- The bench: every control and reading (the cockpit page's Advanced section) ----
 * The page's controls are the bench's state: each one, through its handler below, sets the emulator's input. A plain
 * script: these are globals, used by the page's markup (inline handlers) and by the page's own script. */

let M = null;
let booted = false;
let simSpeed = 0;
let lastFrameWall = performance.now();
let measuredRate = 0;
let activeMapIsFuel = 1;
let afrManualOverride = false;

// Replay state
let replayRows = [];
let replayIdx = 0;
let replayPlaying = false;
let lastReplayStepSim = 0;

function setLoadedSummary(text) {
  const el = document.getElementById("loaded-summary");
  if (el) el.textContent = text;
}

// What runs: the Controller and ECU images by origin (the site's default firmware, or files the user dropped).
const FW = { ct: null, ecu: null, ecuDefault: false, defaultNote: null };

function showFirmware() {
  const fwv = document.getElementById("fw-ver");
  if (!fwv) return;
  const ct = FW.ct || "none (drop your own)";
  const ecu = FW.ecu || "none";
  fwv.textContent = `ECU: ${ecu} | Controller: ${ct}`;
  fwv.style.color = FW.ecu ? "var(--green)" : "var(--muted)";
  const nff = document.getElementById("not-for-flight");
  if (nff) nff.style.display = FW.ecuDefault ? "" : "none";
  const note = document.getElementById("fw-default-note");
  if (note) {
    note.textContent = FW.ecuDefault && FW.defaultNote ? FW.defaultNote : FW.ecuDefault
      ? "Running this site's default firmware. Not for flight. Drop your own files to replace it."
      : (FW.ecu || FW.ct ? "Running the files you supplied." : "No firmware is loaded yet.");
  }
}

async function handleFiles(fileList, label, isDefault = false) {
  if (!M) return;
  let loadedAny = false;
  const loadedNames = [];
  const upperNames = Array.from(fileList, (f) => f.name.toUpperCase());
  const dropsCt = upperNames.some(s32IsCtName), dropsEcu = upperNames.some(s32IsEcuName);
  if (dropsCt) { M._s32_wasm_clear_ct(); FW.ct = null; }          // a dropped image replaces, never mixes
  if (dropsEcu) { M._s32_wasm_clear_ec(); FW.ecu = null; FW.ecuDefault = false; }
  for (const file of fileList) {
    const buf = new Uint8Array(await file.arrayBuffer());
    const name = file.name;
    const upper = name.toUpperCase();
    if (upper.endsWith(".CSV")) {
      const text = new TextDecoder().decode(buf);
      parseDatalogCsv(text, name);
      continue;
    }
    if (s32LoadFile(M, name, buf)) {
      loadedNames.push(name);
      loadedAny = true;
    }
  }
  if (loadedAny) {
    setLoadedSummary(`Loaded: ${loadedNames.join(", ")}`);
    const who = label || "user-supplied";
    if (dropsCt) FW.ct = who;
    if (dropsEcu) { FW.ecu = who; FW.ecuDefault = isDefault; }
    showFirmware();
    rebootSystem();
    renderMapTable();
  }
}

const NO_FIRMWARE_MSG = "No firmware loaded. Drop your own Controller + ECU .bin/.hex (and optional .fram) files to start.";
const ECU_ONLY_MSG = "Running: ECUs only (no Controller image loaded; drop one to see the panel)";

function rebootSystem() {
  if (!M) return;
  const cyl = parseInt(document.getElementById("sel-cyl").value, 10) || 6;
  const mag1 = parseFloat(document.getElementById("in-mag1").value) || 80;
  const mag2 = parseFloat(document.getElementById("in-mag2").value) || 77;
  const rc = M._s32_wasm_boot(cyl, mag1, mag2);
  booted = rc === 0;
  if (rc === 0) {
    updateEngine();
    updateSensors();
    updateSwitches(0);
    updateSwitches(1);
    updateTpsFault();
    updateMapFault();
    updatePumps();
    document.getElementById("status-dot").className = "dot live";
    document.getElementById("status-text").textContent = M._s32_wasm_have_ct() ? "Running" : ECU_ONLY_MSG;
  } else {
    document.getElementById("status-dot").className = "dot warn";
    document.getElementById("status-text").textContent = NO_FIRMWARE_MSG;
  }
}

function setSimSpeed(s) {
  simSpeed = s;
  for (let i = 0; i <= 3; i++) {
    document.getElementById("spd-" + i).classList.toggle("active", i === s);
  }
}

function setRpmPreset(rpm) {
  document.getElementById("sl-rpm").value = String(rpm);
  updateEngine();
}

function setCruisePreset() {
  document.getElementById("sl-rpm").value = "2400";
  document.getElementById("sl-map").value = "24.0";
  document.getElementById("sl-tps").value = "55";
  updateEngine();
  updateSensors();
}

function setTakeoffPreset() {
  document.getElementById("sl-rpm").value = "2700";
  document.getElementById("sl-map").value = "29.0";
  document.getElementById("sl-tps").value = "100";
  updateEngine();
  updateSensors();
}

function injectFailoverPreset() {
  document.getElementById("sw-e1-pwr").checked = false;
  document.getElementById("sw-e2-pwr").checked = true;
  document.getElementById("sw-e2-inj").checked = true;
  document.getElementById("sel-e2-tps").value = "float";
  document.getElementById("sl-rpm").value = "1800";
  updateSwitches(0);
  updateSwitches(1);
  updateTpsFault();
  updateEngine();
}

function clearFaultsPreset() {
  document.getElementById("sw-e1-pwr").checked = true;
  document.getElementById("sw-e2-pwr").checked = true;
  document.getElementById("sw-e1-lnk").checked = true;
  document.getElementById("sw-e2-lnk").checked = true;
  document.getElementById("sel-e2-tps").value = "none";
  document.getElementById("sel-e1-map").value = "none";
  document.getElementById("sl-noise").value = "0";
  updateSwitches(0);
  updateSwitches(1);
  updateTpsFault();
  updateMapFault();
  updateSensors();
}

function updateTpsFault() {
  if (!M) return;
  const mode = document.getElementById("sel-e2-tps").value;
  if (mode === "float") {
    M._s32_wasm_set_tps_fault(1, 512, 30);
  } else if (mode === "gnd") {
    M._s32_wasm_set_tps_fault(1, 0, 0);
  } else if (mode === "5v") {
    M._s32_wasm_set_tps_fault(1, 1023, 0);
  } else {
    M._s32_wasm_set_tps_fault(1, -1, 0);
    updateSensors();
  }
}

function telemetry() {
  return s32Telemetry(M);
}

function setKnob(name, value) {
  return s32SetKnob(M, name, value);
}

// ECU 1's MAP input: its sensor, or a wire fault from now on.
function updateMapFault() {
  if (!M || !booted) return;
  const mode = document.getElementById("sel-e1-map").value;
  M.ccall("s32_wasm_set_an_fault", "number", ["number", "number", "string"], [0, 5, mode === "none" ? "" : mode]);
}

// The fuel pump switches as the Controller's pump inputs see them, moved now.
function updatePumps() {
  if (!M || !booted) return;
  const t = (telemetry().time_s + 0.001).toFixed(4);
  const pos = (id) => (document.getElementById(id).checked ? "on" : "off");
  setKnob("S32_SWITCH", `${t}:pump1=${pos("sw-pump1")},${t}:pump2=${pos("sw-pump2")}`);
}

// The ECU Select switch grounds the Fuel Injectors Enable wire of the ECU that is not selected.
function setEcuSelect(v) {
  document.getElementById("sw-e1-inj").checked = v === "1" || v === "both";
  document.getElementById("sw-e2-inj").checked = v === "2" || v === "both";
  updateSwitches(0);
  updateSwitches(1);
}

function showEcuSelect() {
  const e1 = document.getElementById("sw-e1-inj").checked, e2 = document.getElementById("sw-e2-inj").checked;
  document.getElementById("sel-ecu-select").value = e1 && e2 ? "both" : e1 ? "1" : e2 ? "2" : "none";
}

function updateEngine() {
  if (!M) return;
  const rpm = parseFloat(document.getElementById("sl-rpm").value) || 0;
  const cyl = parseInt(document.getElementById("sel-cyl").value, 10) || 6;
  const mag1 = parseFloat(document.getElementById("in-mag1").value) || 80;
  const mag2 = parseFloat(document.getElementById("in-mag2").value) || 77;
  document.getElementById("val-rpm").textContent = `${rpm} RPM`;
  M._s32_wasm_set_cylinders(cyl, mag1, mag2);
  M._s32_wasm_set_rpm(rpm);
  highlightActiveMapCell();
}

function onManualAfrSlide() {
  afrManualOverride = true;
  document.getElementById("btn-afr-auto").style.display = "inline-block";
  updateSensors();
}

function resetAfrAuto() {
  afrManualOverride = false;
  document.getElementById("btn-afr-auto").style.display = "none";
  updateSensors();
}

function updateSensors() {
  if (!M) return;
  const map = parseFloat(document.getElementById("sl-map").value);
  const tps = parseFloat(document.getElementById("sl-tps").value);
  const noise = parseInt(document.getElementById("sl-noise").value, 10);
  const iat = parseFloat(document.getElementById("sl-iat").value);
  const et = parseFloat(document.getElementById("sl-et").value);
  const batt1 = parseFloat(document.getElementById("sl-batt1").value);
  const batt2 = parseFloat(document.getElementById("sl-batt2").value);
  const afrSlide = parseFloat(document.getElementById("sl-afr").value);

  document.getElementById("val-map").textContent = `${map.toFixed(1)} inHg`;
  document.getElementById("val-tps").textContent = `${tps.toFixed(0)} %`;
  document.getElementById("val-noise").innerHTML = `&plusmn;${noise} counts`;
  document.getElementById("val-iat").innerHTML = `${iat.toFixed(0)} &deg;F`;
  document.getElementById("val-et").innerHTML = `${et.toFixed(0)} &deg;F`;
  document.getElementById("val-batt1").textContent = `${batt1.toFixed(1)} V`;
  document.getElementById("val-batt2").textContent = `${batt2.toFixed(1)} V`;
  if (afrManualOverride) {
    document.getElementById("val-afr").textContent = `${afrSlide.toFixed(1)} (Manual)`;
  }

  M._s32_wasm_set_sensors(map, tps, iat, et, batt1, batt2, noise, afrManualOverride ? afrSlide : -1.0);
  highlightActiveMapCell();
}

function updateSwitches(k) {
  showEcuSelect();
  if (!M) return;
  const pfx = k === 0 ? "sw-e1-" : "sw-e2-";
  const pwr = document.getElementById(pfx + "pwr").checked ? 1 : 0;
  const pld = document.getElementById(pfx + "pld").checked ? 1 : 0;
  const inj = document.getElementById(pfx + "inj").checked ? 1 : 0;
  const ret = document.getElementById(pfx + "ret").checked ? 1 : 0;
  M._s32_wasm_set_switches(k, pld, inj, ret, pwr);
  M._s32_wasm_set_link(k, document.getElementById(pfx + "lnk").checked ? 1 : 0);
}

/* Map Inspector / Editor */
let mapAxes = { rpms: [], maps: [] };

function selectMapTab(isFuel) {
  activeMapIsFuel = isFuel ? 1 : 0;
  document.getElementById("map-btn-fuel").classList.toggle("active", activeMapIsFuel === 1);
  document.getElementById("map-btn-ign").classList.toggle("active", activeMapIsFuel === 0);
  renderMapTable();
}

function renderMapTable() {
  if (!M) return;
  /* ECU1's table as its FRAM chip holds it, decoded by the emulator through the documented FRAM layout. */
  const t = JSON.parse(M.UTF8ToString(M._s32_wasm_get_map_json(0, activeMapIsFuel)));
  if (!t.nrows || !t.ncols) return;
  const nRows = t.nrows;
  const nCols = t.ncols;
  const rpms = t.rpm;
  const maps = t.map_inhg;
  mapAxes = { rpms, maps };

  let html = `<table class="map-tbl"><thead><tr><th>RPM \\ MAP (inHg)</th>`;
  for (let c = 0; c < nCols; c++) html += `<th>${maps[c].toFixed(0)}</th>`;
  html += `</tr></thead><tbody>`;
  for (let r = 0; r < nRows; r++) {
    html += `<tr><th>${rpms[r]}</th>`;
    for (let c = 0; c < nCols; c++) {
      const val = t.cells[r][c];
      html += `<td id="mcell-${r}-${c}"><input class="cell-in" type="number" min="0" max="255" value="${val}" onchange="onEditMapCell(${r},${c},this.value)" /></td>`;
    }
    html += `</tr>`;
  }
  html += `</tbody></table>`;
  document.getElementById("map-table-container").innerHTML = html;
  highlightActiveMapCell();
  /* Each ECU's write state for the viewed table, including a write that was not acknowledged. */
  const say = (e) => {
    const tally = e.sent ? `${e.sent} sent, ${e.acked} acknowledged` : "no writes yet";
    if (e.pending) return `a write is on the wire (${tally})`;
    if (e.last_result === 1) return `last write acknowledged (${tally})`;
    if (e.last_result === -1) return `last write refused by the ECU (${tally})`;
    if (e.last_result === -2) return `a write was not acknowledged (${tally})`;
    return tally;
  };
  const ws = [0, 1].map((k) => JSON.parse(M.UTF8ToString(M._s32_wasm_get_map_json(k, activeMapIsFuel))));
  document.getElementById("map-write-status").textContent = `Link writes this boot -- ECU1: ${say(ws[0])}; ECU2: ${say(ws[1])}`;
}

function onEditMapCell(r, c, valStr) {
  if (!M) return;
  const v = parseInt(valStr, 10);
  if (isNaN(v)) return;
  /* Before boot: edits the FRAM images. Running: sent to both ECUs as link table writes; the view re-reads the FRAM
   * chip once the ECUs have stored it. */
  if (M._s32_wasm_set_map_cell(3, activeMapIsFuel, r, c, v) === 1) setTimeout(renderMapTable, 1500);
}

function highlightActiveMapCell() {
  const { rpms, maps } = mapAxes;
  if (!rpms.length || !maps.length) return;
  const curRpm = parseFloat(document.getElementById("sl-rpm").value) || 0;
  const curMap = parseFloat(document.getElementById("sl-map").value) || 0;
  let bestR = 0, bestC = 0;
  for (let r = 0; r < rpms.length; r++) {
    if (Math.abs(rpms[r] - curRpm) <= Math.abs(rpms[bestR] - curRpm)) bestR = r;
  }
  for (let c = 0; c < maps.length; c++) {
    if (Math.abs(maps[c] - curMap) <= Math.abs(maps[bestC] - curMap)) bestC = c;
  }
  document.querySelectorAll("table.map-tbl td.active-cell").forEach(el => el.classList.remove("active-cell"));
  const cell = document.getElementById(`mcell-${bestR}-${bestC}`);
  if (cell) cell.classList.add("active-cell");
}

/* Datalog CSV Replay */
function parseDatalogCsv(text, name) {
  const lines = text.split(/\r?\n/);
  const rows = [];
  for (const line of lines) {
    const p = line.trim().split(",");
    if (p.length < 24 || p[0] === "Date" || !/^\d/.test(p[2])) continue;
    const rpm1  = parseFloat(p[2]);
    const map1  = parseFloat(p[4]);
    const iat1  = parseFloat(p[6]);
    const tps   = parseFloat(p[8]);
    const et    = parseFloat(p[9]);
    const afr   = parseFloat(p[10]);
    const batt1 = parseFloat(p[11]);
    const batt2 = parseFloat(p[12]);
    const ff    = parseFloat(p[21]);
    const trim  = parseFloat(p[22]);
    const duty  = parseFloat(p[23]);
    const ign   = parseFloat(p[24]);
    if (!isNaN(rpm1) && !isNaN(map1)) {
      rows.push({
        time: p[1],
        rpm1,
        map1,
        iat1,
        tps,
        et,
        afr: !isNaN(afr) ? afr : -1,
        batt1: !isNaN(batt1) ? batt1 : 14.1,
        batt2: !isNaN(batt2) ? batt2 : (!isNaN(batt1) ? batt1 : 14.1),
        ff,
        trim,
        duty,
        ign
      });
    }
  }
  if (!rows.length) return;
  replayRows = rows;
  replayIdx = 0;
  const sl = document.getElementById("sl-rpl");
  sl.max = String(rows.length - 1);
  sl.value = "0";
  sl.disabled = false;
  applyReplayRow(0);
}

/* BEGIN generated by scripts/gen_sample_datalog.py: synthetic values, do not hand-edit */
const DATALOG_COLUMNS = ["Date", "Time", "RPM1", "RPM2", "MAP1", "MAP2", "IAT1", "IAT2", "TPS", "ENGTEMP", "AFR", "BATT1", "BATT2", "PUMP1", "PUMP2", "HP", "TORQUE", "INJ EN1", "INJ EN2", "PLEAD1", "PLEAD2", "FUEL FLOW", "FUEL TRIM", "INJ DUTY", "IGNITION", "T1S1", "T2S2"];
const SAMPLE_DATALOG_ROWS = [
  "06/15/2024,10:12:20,2700,2703,29.1,29.0,71,74,100,182,12.4,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,24.0,0,32.4,26,Y/Y,Y/Y",
  "06/15/2024,10:12:21,2704,2706,29.1,29.0,71,74,100,183,12.4,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,24.0,0,32.4,26,Y/Y,Y/Y",
  "06/15/2024,10:12:22,2706,2708,29.1,28.9,71,74,100,184,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,24.0,0,32.4,26,Y/Y,Y/Y",
  "06/15/2024,10:12:23,2705,2708,29.0,28.9,71,74,100,185,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.9,0,32.3,26,Y/Y,Y/Y",
  "06/15/2024,10:12:24,2702,2704,29.0,28.9,71,74,100,186,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.9,0,32.3,27,Y/Y,Y/Y",
  "06/15/2024,10:12:25,2698,2700,29.0,28.9,71,74,100,186,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.8,0,32.2,27,Y/Y,Y/Y",
  "06/15/2024,10:12:26,2695,2698,29.0,28.9,72,75,100,187,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.8,0,32.1,27,Y/Y,Y/Y",
  "06/15/2024,10:12:27,2694,2696,28.9,28.8,72,75,100,188,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.8,0,32.1,27,Y/Y,Y/Y",
  "06/15/2024,10:12:28,2696,2698,28.9,28.8,72,75,100,189,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.8,0,32.1,27,Y/Y,Y/Y",
  "06/15/2024,10:12:29,2700,2703,28.9,28.8,72,75,100,190,12.4,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,23.8,0,32.1,27,Y/Y,Y/Y",
  "06/15/2024,10:12:30,2704,2706,28.9,28.8,72,75,100,191,12.4,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,23.8,0,32.1,27,Y/Y,Y/Y",
  "06/15/2024,10:12:31,2706,2708,28.8,28.7,72,75,100,192,12.4,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,23.8,0,32.1,27,Y/Y,Y/Y",
  "06/15/2024,10:12:32,2705,2708,28.8,28.7,73,76,100,193,12.4,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,23.8,0,32.1,27,Y/Y,Y/Y",
  "06/15/2024,10:12:33,2702,2704,28.8,28.7,73,76,100,194,12.4,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,23.7,0,32.0,27,Y/Y,Y/Y",
  "06/15/2024,10:12:34,2698,2700,28.8,28.6,73,76,100,195,12.4,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,23.7,0,31.9,27,Y/Y,Y/Y",
  "06/15/2024,10:12:35,2695,2698,28.7,28.6,73,76,100,196,12.4,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,23.6,0,31.9,27,Y/Y,Y/Y",
  "06/15/2024,10:12:36,2694,2696,28.7,28.6,73,76,100,196,12.4,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,23.6,0,31.8,27,Y/Y,Y/Y",
  "06/15/2024,10:12:37,2696,2698,28.7,28.6,73,76,100,197,12.4,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,23.6,0,31.8,27,Y/Y,Y/Y",
  "06/15/2024,10:12:38,2700,2703,28.7,28.6,74,77,100,198,12.4,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,23.6,0,31.9,27,Y/Y,Y/Y",
  "06/15/2024,10:12:39,2704,2706,28.6,28.5,74,77,100,199,12.4,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,23.6,0,31.9,27,Y/Y,Y/Y",
  "06/15/2024,10:12:40,2706,2708,28.6,28.5,74,77,100,200,12.4,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,23.6,0,31.9,27,Y/Y,Y/Y",
  "06/15/2024,10:12:41,2705,2708,28.6,28.5,74,77,100,201,12.4,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,23.6,0,31.8,27,Y/Y,Y/Y",
  "06/15/2024,10:12:42,2702,2704,28.6,28.4,74,77,100,202,12.4,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,23.5,0,31.8,27,Y/Y,Y/Y",
  "06/15/2024,10:12:43,2698,2700,28.5,28.4,74,77,100,203,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.5,0,31.7,27,Y/Y,Y/Y",
  "06/15/2024,10:12:44,2695,2698,28.5,28.4,75,78,100,204,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.4,0,31.6,27,Y/Y,Y/Y",
  "06/15/2024,10:12:45,2694,2696,28.5,28.4,75,78,100,204,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.4,0,31.6,27,Y/Y,Y/Y",
  "06/15/2024,10:12:46,2696,2698,28.5,28.4,75,78,100,205,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.4,0,31.6,27,Y/Y,Y/Y",
  "06/15/2024,10:12:47,2700,2703,28.4,28.3,75,78,100,206,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.4,0,31.6,27,Y/Y,Y/Y",
  "06/15/2024,10:12:48,2704,2706,28.4,28.3,75,78,100,207,12.4,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,23.4,0,31.6,27,Y/Y,Y/Y",
  "06/15/2024,10:12:49,2656,2658,27.5,27.4,75,78,92,208,12.7,14.1,14.0,ON,ON,0,0,ON,OFF,ON,ON,21.6,0,29.2,29,Y/Y,Y/Y",
  "06/15/2024,10:12:50,2605,2608,26.7,26.6,76,79,84,209,12.9,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,19.9,0,26.9,30,Y/Y,Y/Y",
  "06/15/2024,10:12:51,2552,2554,25.8,25.7,76,79,76,210,13.1,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,18.3,0,24.6,31,Y/Y,Y/Y",
  "06/15/2024,10:12:52,2498,2500,24.9,24.8,76,79,68,211,13.3,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,16.7,0,22.5,33,Y/Y,Y/Y",
  "06/15/2024,10:12:53,2495,2498,24.9,24.8,76,79,68,212,13.3,14.0,13.9,ON,ON,0,0,ON,OFF,ON,ON,16.7,0,22.5,33,Y/Y,Y/Y",
  "06/15/2024,10:12:54,2494,2496,24.9,24.8,76,79,68,213,13.3,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,16.6,0,22.5,33,Y/Y,Y/Y",
  "06/15/2024,10:12:55,2496,2498,24.8,24.7,76,79,68,214,13.3,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,16.6,0,22.5,33,Y/Y,Y/Y",
  "06/15/2024,10:12:56,2500,2503,24.8,24.7,77,80,68,214,13.3,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,16.6,0,22.5,33,Y/Y,Y/Y",
  "06/15/2024,10:12:57,2504,2506,24.8,24.7,77,80,68,215,13.3,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,16.7,0,22.5,33,Y/Y,Y/Y",
  "06/15/2024,10:12:58,2506,2508,24.8,24.7,77,80,68,216,13.3,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,16.6,0,22.5,33,Y/Y,Y/Y",
  "06/15/2024,10:12:59,2505,2508,24.7,24.6,77,80,68,217,13.3,13.9,13.8,ON,ON,0,0,ON,OFF,ON,ON,16.6,0,22.4,33,Y/Y,Y/Y",
];
/* END generated by scripts/gen_sample_datalog.py */
function loadSampleTakeoffReplay() {
  const sample = [DATALOG_COLUMNS.join(","), ...SAMPLE_DATALOG_ROWS].join("\n");
  parseDatalogCsv(sample, "Demo_Climb.CSV");
  replayPlaying = true;
}

function scrubReplay(val) {
  const idx = parseInt(val, 10) || 0;
  applyReplayRow(idx);
}

function toggleReplayPlay() {
  if (!replayRows.length) {
    loadSampleTakeoffReplay();
    return;
  }
  replayPlaying = !replayPlaying;
}

function applyReplayRow(idx) {
  if (!replayRows.length || idx < 0 || idx >= replayRows.length) return;
  replayIdx = idx;
  const r = replayRows[idx];
  document.getElementById("sl-rpl").value = String(idx);
  document.getElementById("sl-rpm").value = String(r.rpm1);
  document.getElementById("sl-map").value = String(r.map1);
  document.getElementById("sl-iat").value = String(r.iat1);
  document.getElementById("sl-et").value = String(r.et);
  document.getElementById("sl-tps").value = String(r.tps);
  document.getElementById("sl-batt1").value = String(r.batt1);
  document.getElementById("sl-batt2").value = String(r.batt2);
  updateEngine();
  updateSensors();
  document.getElementById("rpl-status").textContent = `Row ${idx + 1}/${replayRows.length} (${r.time}): ${r.rpm1} RPM, ${r.map1.toFixed(1)} inHg`;
  document.getElementById("rpl-logged").textContent = `Logged: ${r.ff.toFixed(1)} GPH (${r.duty.toFixed(1)}% duty, IGN ${r.ign}°, Trim ${r.trim >= 0 ? "+" : ""}${r.trim}%)`;
}

// A configuration chip (0 = Controller, 1 = ECU 1, 2 = ECU 2) as a file to save: 128 KB, as the chip holds it.
function downloadFram(target, name) {
  if (!M) return;
  const p = M._s32_wasm_get_fram_ptr(target);
  if (!p) return;
  const url = URL.createObjectURL(new Blob([M.HEAPU8.slice(p, p + 131072)], { type: "application/octet-stream" }));
  const a = Object.assign(document.createElement("a"), { href: url, download: name });
  document.body.append(a);
  a.click();
  a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

// The file drop and the other listeners the markup cannot declare. Called once the page's markup exists.
function benchInit() {
  const dz = document.getElementById("dropzone");
  dz.addEventListener("dragover", (e) => { e.preventDefault(); dz.classList.add("dragover"); });
  dz.addEventListener("dragleave", () => dz.classList.remove("dragover"));
  dz.addEventListener("drop", (e) => {
    e.preventDefault();
    dz.classList.remove("dragover");
    if (e.dataTransfer.files.length) handleFiles(e.dataTransfer.files);
  });
}

function renderTelemetry(t) {
  document.getElementById("sim-time").textContent = `${t.time_s.toFixed(2)}s`;
  document.getElementById("sim-rate").textContent = `${measuredRate.toFixed(2)}×`;
  document.getElementById("sim-trim").textContent = `${t.trim_pct[0] >= 0 ? "+" : ""}${t.trim_pct[0]}%`;
  document.getElementById("sim-gph").textContent = `${t.fuel_gph.toFixed(1)} GPH`;
  if (typeof t.afr === "number") {
    document.getElementById("sim-afr").textContent = t.afr.toFixed(1);
    if (!afrManualOverride) {
      document.getElementById("sl-afr").value = t.afr.toFixed(1);
      document.getElementById("val-afr").textContent = `${t.afr.toFixed(1)} (Auto)`;
    }
  }

  let html = "";
  for (let k = 0; k < 2; k++) {
    const e = t.ecu[k];
    const st = !e.powered ? `<span style="color:var(--red)">OFFLINE</span>`
             : e.err ? `<span style="color:var(--red)">ERR ${e.err}</span>`
             : `<span style="color:var(--green)">ONLINE</span>`;
    html += `
      <div class="telem-ecu">
        <h3><span>ECU ${k + 1}</span>${st}</h3>
        <div style="display:flex;justify-content:space-between;font-family:var(--mono);font-size:11px;color:var(--muted);">
          <span>CT&rarr;ECU: <strong style="color:var(--text)">${e.ct_to_ecu}</strong></span>
          <span>ECU&rarr;CT: <strong style="color:var(--text)">${e.ecu_to_ct}</strong></span>
          <span>Duty / Trim: <strong style="color:var(--accent)">${t.duty_pct[k].toFixed(1)}% / ${t.trim_pct[k] >= 0 ? "+" : ""}${t.trim_pct[k]}%</strong></span>
        </div>
        <table class="telem-tbl">
          <thead><tr><th>Ignition Coil</th><th>Sparks</th><th>Dwell</th><th>Advance</th></tr></thead>
          <tbody>
            ${e.coils.map((c, i) => `
              <tr>
                <td>Coil ${i + 1} (Cyl ${i*2+1}/${i*2+2})</td>
                <td>${c.sparks}</td>
                <td>${c.dwell_ms.toFixed(2)} ms</td>
                <td>${c.adv_deg.toFixed(1)}&deg; BTDC</td>
              </tr>
            `).join("")}
          </tbody>
        </table>
        <table class="telem-tbl" style="margin-top:6px;">
          <thead><tr><th>Injector</th><th>Pulses</th><th>Pulse Width</th><th>Duty (@ ${t.rpm} RPM)</th></tr></thead>
          <tbody>
            ${e.inj.slice(0, t.ncyl).map((inj, i) => {
              const revMs = t.rpm > 0 ? (60000 / t.rpm) : 0;
              const duty = revMs > 0 ? (inj.width_ms / revMs * 100) : 0;
              return `
                <tr>
                  <td>Cyl ${i + 1}</td>
                  <td>${inj.pulses}</td>
                  <td>${inj.width_ms.toFixed(3)} ms</td>
                  <td>${duty.toFixed(1)}%</td>
                </tr>
              `;
            }).join("")}
          </tbody>
        </table>
      </div>
    `;
  }
  document.getElementById("telem-container").innerHTML = html;

  const con1 = M.UTF8ToString(M._s32_wasm_get_ecu_console(0));
  if (con1) document.getElementById("ecu-console").textContent = con1.trim();
}

// The controls a scenario starts from, at the values the page loads with (engine configuration and replay excluded).
const SCENARIO_CONTROLS = ["sl-rpm", "sl-map", "sl-tps", "sl-afr", "sl-iat", "sl-et", "sl-batt1", "sl-batt2", "sl-noise",
  "sw-e1-pwr", "sw-e1-lnk", "sw-e1-pld", "sw-e1-inj", "sw-e1-ret", "sel-e1-map",
  "sw-e2-pwr", "sw-e2-lnk", "sw-e2-pld", "sw-e2-inj", "sw-e2-ret", "sel-e2-tps", "sw-pump1", "sw-pump2"];

// Every control back to the value the page loads with, the engine stopped (blank: the 4-cylinder engine the firmware
// writes on a blank configuration chip).
function benchResetControls(blank) {
  replayPlaying = false;
  if (afrManualOverride) resetAfrAuto();
  for (const id of SCENARIO_CONTROLS) {
    const el = document.getElementById(id);
    if (el.type === "checkbox") el.checked = el.defaultChecked;
    else if (el.tagName === "SELECT") el.value = Array.from(el.options).find((o) => o.defaultSelected)?.value ?? el.options[0].value;
    else el.value = el.defaultValue;
  }
  document.getElementById("sl-rpm").value = "0";
  if (blank) document.getElementById("sel-cyl").value = "4";
}

// One control by its id (a slider, box or list), then its own handler: a scenario's "set" step.
function benchSet(id, value) {
  const el = document.getElementById(id);
  if (el.type === "checkbox") el.checked = value;
  else el.value = String(value);
  el.dispatchEvent(new Event(el.type === "range" ? "input" : "change"));
}

// The default firmware (s32FetchDefaultFirmware): the files firmware/manifest.json names, each checked against its
// SHA-256 first. Dropping your own files replaces these. With no configuration files dropped the ECUs' configuration
// chips are blank: on its first power-up the firmware writes its default settings into them (as on a new ECU) and
// starts firing from the next. Run that first power-up (2 s) and power-cycle once, so the bench starts configured. The
// firmware's default configuration is a 4-cylinder engine: match the crank wheel to it (6 is the page's default for
// dropped IO-540 configurations). True when the bench booted.
async function benchLoadDefaultFirmware() {
  const fw = await s32FetchDefaultFirmware();
  if (!fw) return false;
  FW.defaultNote = fw.note;
  await handleFiles(fw.files, fw.label, true);
  document.getElementById("sel-cyl").value = "4";
  if (M._s32_wasm_step_quanta(20000) === 0) rebootSystem();
  return booted;
}
