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
 * "blank": the configuration chips are erased before the power-up instead of kept. After the steps the bench pauses,
 * unless the link says autostart=1 or the visitor picked the scenario from the list.
 *
 * For the cockpit page each scenario also has:
 *   shows   what the scenario shows, in two lines at most
 *   guide   a link to the pilot guide's page on it ("" for none yet)
 *   pilot   the steps a pilot follows, in order: say (what to do), watch (what to look for) and, for a step the pilot
 *           cannot do from the cockpit, act (the label of the button that does it) and do (the steps it runs).
 * Every "watch" line is what this bench shows with the default firmware; scripts/bench_scenarios.py checks each one. */
const SCENARIOS = [
  { id: "first-start", title: "First start", blank: true,
    description: "Blank configuration chips, first power-up, engine stopped.",
    steps: [["run", 1]],
    shows: "The first power-up of a new system. The ECUs store their default settings and do not fire until the next power-up.",
    guide: "",
    pilot: [
      { say: "Look at the tabs along the top of the screen.",
        watch: "ECU1 and ECU2 are red, and both Ignition lights are off." },
      { say: "Press Up.",
        watch: "The Main Menu opens. With the engine stopped, it lists its settings." },
      { say: "Press Back.",
        watch: "The gauges come back." },
      { say: "Cycle the power to both ECUs.", act: "Cycle power", do: [["power-cycle"]],
        watch: "ECU1 turns blue and ECU2 green. Blue marks the ECU that fuels the engine." },
    ] },
  { id: "runup", title: "Run-up",
    description: "1800 RPM, both ECUs firing, ECU 1 selected. Try Ignition Enable, the fuel pumps and ECU Select.",
    steps: [["rpm", 1800], ["set", "sl-map", 20.0], ["set", "sl-tps", 30], ["run", 3]],
    shows: "The engine at 1800 RPM for the run-up. Both ECUs fire, ECU 1 fuels, Fuel Pump 1 is on.",
    guide: "",
    pilot: [
      { say: "Turn Ignition 1 off.",
        watch: "The ECU1 tab turns red and Ignition light 1 goes out. ECU 2 keeps firing." },
      { say: "Turn Ignition 1 back on.",
        watch: "ECU1 turns blue again and its light comes back on." },
      { say: "Turn Ignition 2 off.",
        watch: "The ECU2 tab turns red. ECU 1 keeps firing." },
      { say: "Turn Ignition 2 back on.",
        watch: "ECU2 turns green again." },
      { say: "Turn Fuel Pump 2 on.",
        watch: "The PMP2 tab turns yellow." },
      { say: "Set ECU Select to 2.",
        watch: "ECU2 turns blue and ECU1 green: ECU 2 now fuels the engine." },
    ] },
  { id: "cruise", title: "Cruise",
    description: "2400 RPM, 24.0 inHg, fuel trim 0 %.",
    steps: [["preset", "cruise"], ["run", 3]],
    shows: "Cruise at 2400 RPM and 24 inHg with the fuel trim at 0 %.",
    guide: "",
    pilot: [
      { say: "Turn the knob five clicks to the left.",
        watch: "Fuel Trim reads -5 % and the AFR rises from about 11.1 to 11.7: a leaner mixture." },
      { say: "Turn the knob five clicks to the right.",
        watch: "Fuel Trim is back at 0 % and the AFR at about 11.1." },
      { say: "Press Back.",
        watch: "The next page shows Eng Temp, Throttle, Int Air Temp, Inj Duty, Battery and Crank Trigger." },
      { say: "Press Back again.",
        watch: "The box moves to the ECU2 tab: these are ECU 2's numbers. Its Fuel Flow shows dashes because ECU 2 does not fuel." },
    ] },
  { id: "takeoff", title: "Takeoff",
    description: "2700 RPM, 29.0 inHg, full throttle.",
    steps: [["preset", "takeoff"], ["run", 3]],
    shows: "Full throttle for takeoff: 2700 RPM and 29 inHg.",
    guide: "",
    pilot: [
      { say: "Read RPM and MAP on the screen.",
        watch: "RPM reads 2700. MAP climbs to about 29." },
      { say: "Press Up.",
        watch: "The Main Menu says its functions are not available while the engine is running." },
      { say: "Press Back.",
        watch: "The gauges come back." },
      { say: "Press Down.",
        watch: "The Tuning Menu opens, engine running: RPM Fuel Tuning, MAP Fuel Tuning, Injector Trim." },
      { say: "Press Back.",
        watch: "The gauges come back." },
      { say: "Pull the throttle back to Cruise.",
        watch: "RPM falls to 2400 and MAP to about 24." },
    ] },
  { id: "ecu-silent", title: "ECU 1 silent",
    description: "Cruise with ECU 1 powered but its data link silent: red ECU1 tab, dashes for its values.",
    steps: [["preset", "cruise"], ["run", 1], ["set", "sw-e1-lnk", false], ["run", 4]],
    shows: "ECU 1 has power and runs, but the Controller hears nothing from it, as with a broken data cable.",
    guide: "",
    pilot: [
      { say: "Look at the ECU1 tab.",
        watch: "It is red, and RPM, MAP, Fuel Flow and IGN show dashes. Both Ignition lights stay on: ECU 1 still fires." },
      { say: "Set ECU Select to 2.",
        watch: "ECU2 turns blue: ECU 2 now fuels. ECU1 stays red." },
      { say: "Press Back twice.",
        watch: "The box moves to the ECU2 tab and shows ECU 2's RPM, MAP and fuel flow." },
    ] },
  { id: "map-fail", title: "MAP sensor failed",
    description: "Cruise with ECU 1's MAP sensor disconnected. Select ECU 2 to fuel from the other sensor.",
    steps: [["preset", "cruise"], ["run", 1], ["set", "sel-e1-map", "open"], ["run", 3]],
    shows: "ECU 1's manifold pressure sensor has failed. ECU 1 fuels from a wrong reading until you select ECU 2.",
    guide: "",
    pilot: [
      { say: "Read MAP and AFR on the screen.",
        watch: "MAP reads far below the engine's 24 inHg, and AFR reads 20.0, the top of its scale: very lean." },
      { say: "Set ECU Select to 2.",
        watch: "ECU2 turns blue and the AFR comes back to about 11: ECU 2 fuels from its own sensor." },
      { say: "Press Back twice.",
        watch: "ECU 2's page shows MAP at about 24." },
    ] },
  { id: "ecu-select", title: "Fuel from ECU 2",
    description: "Cruise with ECU Select on ECU 2: ECU 2 fuels the engine (blue ECU2 tab).",
    steps: [["preset", "cruise"], ["run", 1], ["set", "sel-ecu-select", "2"], ["run", 3]],
    shows: "Cruise with ECU Select on 2. ECU 2 fuels the engine; ECU 1 still fires.",
    guide: "",
    pilot: [
      { say: "Look at the tabs along the top of the screen.",
        watch: "ECU2 is blue: it fuels the engine. ECU1 is green: it runs and fires." },
      { say: "Turn Ignition 2 off.",
        watch: "ECU2 turns red and Ignition light 2 goes out. ECU 1 keeps firing." },
      { say: "Turn Ignition 2 back on.",
        watch: "ECU2 turns blue again." },
      { say: "Set ECU Select to 1.",
        watch: "ECU1 turns blue and ECU2 green: ECU 1 fuels again." },
    ] },
  { id: "rpm-limit", title: "Near the RPM limit",
    description: "2950 RPM. With this configuration the ignition cuts at 3000 RPM: move the RPM slider there.",
    steps: [["preset", "cruise"], ["rpm", 2950], ["run", 3]],
    shows: "The engine at 2950 RPM, just under the 3000 RPM limit of this configuration.",
    guide: "",
    pilot: [
      { say: "Look at the Ignition lights.",
        watch: "Both are on: both ECUs fire at 2950 RPM." },
      { say: "Raise the engine to 3000 RPM.", act: "Go to 3000 RPM", do: [["rpm", 3000]],
        watch: "Both Ignition lights go out: the ECUs stop firing at the limit. On this bench the engine speed is held, so the screen still reads 3000." },
      { say: "Pull the throttle back to Cruise.",
        watch: "RPM falls to 2400 and both Ignition lights come back on." },
    ] },
  { id: "trim-lop", title: "Leaned with trim",
    description: "Cruise with the fuel trim turned to -28 %: the bench AFR reads about 15.5.",
    steps: [["preset", "cruise"], ["trim", -28], ["run", 3]],
    shows: "Cruise with the fuel trim turned down to -28 %. The mixture is leaned until the AFR reads about 15.5.",
    guide: "",
    pilot: [
      { say: "Read Fuel Trim and AFR on the screen.",
        watch: "Fuel Trim reads -28 % and AFR about 15.5." },
      { say: "Turn the knob to the right until Fuel Trim reads 0 %.",
        watch: "The AFR falls back to about 11." },
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
      M._s32_wasm_step_quanta(n);
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

  async function resetBench(blank) {
    host.resetControls(blank);
    if (blank) for (const target of [0, 1, 2]) host.M()._s32_wasm_clear_fram(target);
    await powerUp();
    if (!blank) await setTrim(0);
  }

  const actions = () => ({ ...host.actions, trim: setTrim, run: advance, "power-cycle": powerUp });

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
