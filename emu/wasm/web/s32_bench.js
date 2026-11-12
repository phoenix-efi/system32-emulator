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
const SCENARIOS = [
  { id: "first-start", title: "First start", blank: true,
    description: "Blank configuration chips, first power-up, engine stopped.",
    steps: [["run", 1]],
    shows: "First power-up of a new system: the ECUs store their default settings and fire from the next power-up.",
    guide: "",
    pilot: [
      { say: "Look at the tabs", watch: "ECU1 and ECU2 red." },
      { say: "Press MENU", watch: "Main Menu." },
      { say: "Press MONITOR", watch: "Gauges." },
      { say: "Cycle power", act: "Cycle power", do: [["power-cycle"]], watch: "ECU1 blue, ECU2 green." },
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
      { say: "Throttle back to MAP 24", watch: "RPM ~2550." },
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
    shows: "RPM limit 2750, fine low-pitch stop. Throttle full, fast: RPM pulses at 2750 until the governor holds 2700.",
    guide: "",
    pilot: [
      { say: "Throttle full, fast", watch: "RPM 2750, pulsing, then 2700." },
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

/* The bench page's controls that a scenario starts from, by id, at the values that page loads with: sliders, the
 * ECU switch boxes and fault lists, the panel switches. The cockpit page drives the same controls with these values;
 * scripts/bench_scenarios.py checks them against index.html. */
const BENCH_DEFAULTS = {
  "sl-rpm": 2400, "sl-map": 24.0, "sl-tps": 55, "sl-afr": 14.2, "sl-iat": 72, "sl-et": 185, "sl-batt1": 14.1,
  "sl-batt2": 14.1, "sl-noise": 0,
  "sw-e1-pwr": true, "sw-e1-lnk": true, "sw-e1-pld": true, "sw-e1-inj": true, "sw-e1-ret": false, "sel-e1-map": "none",
  "sw-e2-pwr": true, "sw-e2-lnk": true, "sw-e2-pld": true, "sw-e2-inj": false, "sw-e2-ret": false, "sel-e2-tps": "none",
  "sw-pump1": true, "sw-pump2": false,
};

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
 * and gets back run(id, keepRunning), act(list) (steps from a pilot step's "do", at the current state) and busy().
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
    const sc = SCENARIOS.find((x) => x.id === id);
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
