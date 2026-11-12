// Headless Node/V8 smoke test for the 3-MCU WebAssembly bundle (make -C emu/wasm test).
//
// No firmware ships with this repository. For the firmware checks point S32_FIRMWARE_DIR at a directory holding:
//   *_prog.bin      Cockpit Controller program flash
//   *_bflash.bin    ECU boot flash
//   *_pflash.bin    ECU program flash
//   default_ecu.fram (optional; ECU FRAM image used for both ECUs, otherwise blank FRAM)
//
// Without S32_FIRMWARE_DIR only the firmware-independent checks run; every firmware-dependent
// check is reported as SKIP, never as ok.
//
// The numeric expectations (29.0 deg advance, ~8.07 ms pulse width at 2400 RPM / 24.0 inHg) are the
// values measured on the stock v86 controller + ECU images with default_ecu.fram. They describe that
// firmware's behaviour in this emulator; they are not a specification. The AFR is the bench's physics
// (emu/common/bench_engine.c: injector open time minus dead time x 60 lb/hr, speed-density air for a
// 4 x 90 cu in engine at VE 0.868): the vendor-default map runs rich on that engine, about 11.2.

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const wasmJsPath = path.resolve(__dirname, "../build/s32_wasm.js");

const { default: createS32Module } = await import(wasmJsPath);
const M = await createS32Module();

function writeBytes(buf, fn, extra = []) {
  const ptr = M._malloc(buf.length);
  M.HEAPU8.set(buf, ptr);
  const rc = fn(ptr, buf.length, ...extra);
  M._free(ptr);
  return rc;
}

let fails = 0;
let passes = 0;
let skips = 0;
function check(name, cond, detail = "") {
  if (cond) {
    console.log(`  ok   wasm ${name}${detail ? " (" + detail + ")" : ""}`);
    passes++;
  } else {
    console.error(`  FAIL wasm ${name}${detail ? " (" + detail + ")" : ""}`);
    fails++;
  }
}
function skip(name, why) {
  console.log(`  SKIP wasm ${name} (${why})`);
  skips++;
}
function finish() {
  console.log(`\nwasm smoke: ${passes} ok, ${fails} FAIL, ${skips} SKIP`);
  if (skips) console.log("  (SKIPs are firmware-dependent checks: to run them, run make firmware (or set S32_FIRMWARE_DIR=.firmware-cache/v86))");
  process.exit(fails ? 1 : 0);
}

// ---- Firmware-independent checks ------------------------------------------------------------
const requiredExports = [
  "_s32_wasm_load_ct_firmware", "_s32_wasm_load_ec_bflash", "_s32_wasm_load_ec_pflash",
  "_s32_wasm_load_fram", "_s32_wasm_boot", "_s32_wasm_step_quanta", "_s32_wasm_render_rgba",
  "_s32_wasm_get_telemetry_json", "_s32_wasm_get_ecu_console", "_s32_wasm_encoder_turn",
];
const missing = requiredExports.filter((f) => typeof M[f] !== "function");
check("module exports the emulator API", missing.length === 0, missing.length ? `missing ${missing.join(",")}` : `${requiredExports.length} functions`);
check("boot refuses to start with no ECU firmware loaded", M._s32_wasm_boot(4, 80, 80) === -1);
check("Controller loader rejects a too-short image", writeBytes(new Uint8Array(4), M._s32_wasm_load_ct_firmware) === -1);
// The harness knobs (emu/common/s32board.c) by name, as s32pair reads them from its environment.
const knobNamesPtr = M._s32_wasm_knob_names();
if (!knobNamesPtr) throw new Error(M.UTF8ToString(M._s32_wasm_knob_error()));
const knobNames = JSON.parse(M.UTF8ToString(knobNamesPtr));
const knob = (n, v) => M.ccall("s32_wasm_knob", "number", ["string", "string"], [n, v]);
check("harness knobs reachable by name", knobNames.includes("S32_AN_FAULT") && knobNames.includes("S32_ECU_SELECT") &&
      knobNames.length >= 40, `${knobNames.length} knobs`);
check("a bad knob value is refused with a reason", knob("S32_CYL", "5") === -2 &&
      M.UTF8ToString(M._s32_wasm_knob_error()).includes("S32_CYL"));

// Pre-boot knob storage: the module keeps the caller's strings whole and says why when it refuses. (A fresh module:
// pending knobs would otherwise ride into the firmware checks' boot.)
{
  const K = await createS32Module();
  const kknob = (n, v) => K.ccall("s32_wasm_knob", "number", ["string", "string"], [n, v]);
  for (let i = 0; i < 64; i++) kknob("S32_RPM", "1200");
  const rc65 = kknob("S32_RPM", "1300");
  check("the 65th pre-boot knob is refused with a reason", rc65 === -3 &&
        K.UTF8ToString(K._s32_wasm_knob_error()).includes("too many"),
        `rc ${rc65}: "${K.UTF8ToString(K._s32_wasm_knob_error())}"`);
}

// ---- Firmware-dependent checks --------------------------------------------------------------
const FW_CHECKS = [
  "Controller program flash load",
  "ECU boot + program flash load",
  "3-MCU boot",
  "Controller display non-blank at t=1.0s (boot screen)",
  "Controller display changes after boot screen by t=4.0s",
  "Serial link frames in both directions, both ECUs",
  "ECU1+ECU2 spark advance 29.0 deg BTDC (2400 RPM @ 24.0 inHg)",
  "ECU1+ECU2 injector pulse width ~8.07 ms (2400 RPM @ 24.0 inHg)",
  "Wideband AFR 11.2 +-0.3 (bench physics, default map) at 2400 RPM / 24.0 inHg",
  "ECU UART1 console output on boot",
  "4.0 s simulated in < 2.0 s wall",
  "Rotary knob +5 raises fuel trim (wider pulse, lower AFR)",
  "Fuel trim read off the link (status frames) equals on both ECUs after the knob",
  "Map edit while running goes over the link: both ECUs ACK, store it in FRAM and run it",
  "ECU1 power loss: Controller shows ECU1 gray; the knob still reaches ECU2",
  "ECU1 link dead but powered: Controller shows ECU1 red",
  "Harness knob from the page: ECU1 P-lead to STOP stops ECU1's sparks, ECU2 keeps firing",
  "A knob that passes the RTC-less validation board but fails on the real board fails the boot, naming the knob",
  "A >4096-byte pre-boot knob value is kept whole and applied at boot (the native front end has no length limit either)",
  "Boot again after the Controller has run (the page's power cycle after the first power-up) does not trap",
  "ECU Select on ECU 2 by the page's switches (ECU 1's enable wire grounded, ECU 2's open): ECU 2 alone fuels",
  "ECU 1 MAP input disconnected from the page: ECU 1 reports a low MAP, ECU 2 its sensor; cleared, both read it again",
];

const fwDir = process.env.S32_FIRMWARE_DIR;
if (!fwDir) {
  for (const n of FW_CHECKS) skip(n, "S32_FIRMWARE_DIR not set; no firmware is shipped");
  finish();
}
if (!fs.existsSync(fwDir) || !fs.statSync(fwDir).isDirectory()) {
  check(`S32_FIRMWARE_DIR=${fwDir} is a directory`, false);
  finish();
}
const files = fs.readdirSync(fwDir);
const pick = (suffix) => files.filter((f) => f.endsWith(suffix)).sort()[0];
const ctName = pick("_prog.bin");
const bflName = pick("_bflash.bin");
const pflName = pick("_pflash.bin");
if (!ctName || !bflName || !pflName) {
  check(`S32_FIRMWARE_DIR=${fwDir} holds *_prog.bin, *_bflash.bin and *_pflash.bin`, false,
        `found prog=${ctName ?? "-"} bflash=${bflName ?? "-"} pflash=${pflName ?? "-"}`);
  finish();
}
console.log(`  firmware: ${fwDir} (${ctName}, ${bflName}, ${pflName})`);
const ctProg = fs.readFileSync(path.join(fwDir, ctName));
const ecBfl = fs.readFileSync(path.join(fwDir, bflName));
const ecPfl = fs.readFileSync(path.join(fwDir, pflName));
const framPath = path.join(fwDir, "default_ecu.fram");
const defFram = fs.existsSync(framPath) ? fs.readFileSync(framPath) : null;
console.log(`  ECU FRAM: ${defFram ? framPath : "none (blank FRAM; numeric expectations assume default_ecu.fram)"}`);

const ctRc = writeBytes(ctProg, M._s32_wasm_load_ct_firmware);
const bflRc = writeBytes(ecBfl, M._s32_wasm_load_ec_bflash);
const pflRc = writeBytes(ecPfl, M._s32_wasm_load_ec_pflash);
check(FW_CHECKS[0], ctRc > 0, `rc=${ctRc}`);
check(FW_CHECKS[1], bflRc === 0 && pflRc === 0, `bflash rc=${bflRc}, pflash rc=${pflRc}`);

M._s32_wasm_clear_fram(0);
if (defFram) {
  writeBytes(defFram, (p, n) => M._s32_wasm_load_fram(1, p, n));
  writeBytes(defFram, (p, n) => M._s32_wasm_load_fram(2, p, n));
}

const bootRc = M._s32_wasm_boot(4, 80, 80);
check(FW_CHECKS[2], bootRc === 0, `rc=${bootRc}`);

M._s32_wasm_set_rpm(2400);
M._s32_wasm_set_sensors(24.0, 55.0, 72.0, 185.0, 14.1, 14.1, 0, -1.0);

function countLitPixels() {
  const rgbaPtr = M._s32_wasm_render_rgba();
  const rgba = new Uint8Array(M.HEAPU8.buffer, rgbaPtr, 320 * 240 * 4);
  let lit = 0;
  for (let i = 0; i < rgba.length; i += 4) {
    if (rgba[i] !== 0 || rgba[i + 1] !== 0 || rgba[i + 2] !== 0) lit++;
  }
  return lit;
}

const t0 = performance.now();
M._s32_wasm_step_quanta(10000); // 1.0 s simulated
const splashLit = countLitPixels();
check(FW_CHECKS[3], splashLit > 0, `${splashLit}/76800 lit pixels`);

M._s32_wasm_step_quanta(30000); // 4.0 s simulated
const wallMs = performance.now() - t0;
const gaugeLit = countLitPixels();
check(FW_CHECKS[4], gaugeLit > 0 && gaugeLit !== splashLit, `${gaugeLit}/76800 lit pixels (was ${splashLit})`);

const t = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
const e1 = t.ecu[0];
const e2 = t.ecu[1];

check(FW_CHECKS[5], e1.ct_to_ecu >= 12 && e1.ecu_to_ct >= 15 && e2.ct_to_ecu >= 12 && e2.ecu_to_ct >= 15,
      `E1 rx/tx=${e1.ct_to_ecu}/${e1.ecu_to_ct}, E2 rx/tx=${e2.ct_to_ecu}/${e2.ecu_to_ct}`);
check(FW_CHECKS[6],
      Math.abs(e1.coils[0].adv_deg - 29.0) <= 0.3 && Math.abs(e2.coils[0].adv_deg - 29.0) <= 0.3,
      `E1=${e1.coils[0].adv_deg.toFixed(1)} deg, E2=${e2.coils[0].adv_deg.toFixed(1)} deg`);
check(FW_CHECKS[7],
      Math.abs(e1.inj[0].width_ms - 8.07) <= 0.15 && Math.abs(e2.inj[0].width_ms - 8.07) <= 0.15,
      `E1=${e1.inj[0].width_ms.toFixed(3)} ms, E2=${e2.inj[0].width_ms.toFixed(3)} ms`);
check(FW_CHECKS[8], Math.abs(t.afr - 11.2) <= 0.3, `AFR=${t.afr.toFixed(2)}`);

const con0 = M.UTF8ToString(M._s32_wasm_get_ecu_console(0)).trim();
check(FW_CHECKS[9], con0.length > 0, `len=${con0.length}`);

check(FW_CHECKS[10], wallMs < 2000, `4.0 s sim in ${wallMs.toFixed(0)} ms (${(4000 / wallMs).toFixed(2)}x real time)`);

const pwBefore = e1.inj[0].width_ms;
M._s32_wasm_encoder_turn(5);
M._s32_wasm_step_quanta(10000);
const tPlus = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
check(FW_CHECKS[11],
      tPlus.ecu[0].inj[0].width_ms > pwBefore + 0.2 && tPlus.afr < t.afr - 0.2,
      `PW ${pwBefore.toFixed(2)} -> ${tPlus.ecu[0].inj[0].width_ms.toFixed(2)} ms, AFR ${t.afr.toFixed(1)} -> ${tPlus.afr.toFixed(1)}`);

M._s32_wasm_step_quanta(15000);   // status frames every ~0.5 s; the Controller mirrors ECU1's trim to ECU2 after one
const tTrim = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
check(FW_CHECKS[12], tTrim.trim_pct[0] > 0 && tTrim.trim_pct[0] === tTrim.trim_pct[1],
      `trim ECU1/ECU2 = ${tTrim.trim_pct[0]}/${tTrim.trim_pct[1]} %`);

// Map edit while running: fuel cell (row 8, col 5) +30 on both ECUs, sent as link table writes.
const mapCell = (k) => JSON.parse(M.UTF8ToString(M._s32_wasm_get_map_json(k, 1)));
const before = mapCell(0).cells[8][5];
const pwPre = tTrim.ecu[0].inj[0].width_ms;
const rcEdit = M._s32_wasm_set_map_cell(3, 1, 8, 5, before + 30);
M._s32_wasm_step_quanta(20000);
const m1 = mapCell(0), m2 = mapCell(1);
const tEdit = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
check(FW_CHECKS[13],
      rcEdit === 1 && m1.last_result === 1 && m2.last_result === 1 && m1.cells[8][5] === before + 30 &&
      m2.cells[8][5] === before + 30 && m1.valid && m2.valid &&
      m1.sent === 1 && m1.acked === 1 && m2.sent === 1 && m2.acked === 1 &&
      tEdit.ecu[0].inj[0].width_ms > pwPre + 0.3 && tEdit.ecu[1].inj[0].width_ms > pwPre + 0.3,
      `cell ${before} -> ${m1.cells[8][5]}/${m2.cells[8][5]}, ACK ${m1.last_result}/${m2.last_result}, ` +
      `sent/acked ${m1.sent}/${m1.acked} ${m2.sent}/${m2.acked}, ` +
      `PW ${pwPre.toFixed(2)} -> ${tEdit.ecu[0].inj[0].width_ms.toFixed(2)}/${tEdit.ecu[1].inj[0].width_ms.toFixed(2)} ms`);

// ECU tab colour on the Controller's home page (label box OBSERVED on the stock firmware).
function ecu1TabColour() {
  const p = M._s32_wasm_render_rgba();
  const px = new Uint8Array(M.HEAPU8.buffer, p, 320 * 240 * 4);
  const n = {};
  for (let y = 14; y <= 25; y++) for (let x = 29; x <= 68; x++) {
    const i = (y * 320 + x) * 4, r = px[i], g = px[i + 1], b = px[i + 2];
    if ((r | g | b) === 0 || Math.min(r, g, b) > 240) continue;
    const c = r > 200 && g < 80 && b < 80 ? "red" : b > 200 && r < 80 && g < 80 ? "blue"
            : g > 150 && r < 80 && b < 80 ? "green"
            : Math.abs(r - g) < 24 && Math.abs(g - b) < 24 && r > 90 && r < 210 ? "gray" : "other";
    n[c] = (n[c] || 0) + 1;
  }
  return Object.keys(n).sort((a, b) => n[b] - n[a])[0] || "none";
}
function rebootRunning() {
  M._s32_wasm_clear_fram(0);
  if (defFram) {
    writeBytes(defFram, (p, n) => M._s32_wasm_load_fram(1, p, n));
    writeBytes(defFram, (p, n) => M._s32_wasm_load_fram(2, p, n));
  }
  M._s32_wasm_boot(4, 80, 80);
  M._s32_wasm_set_rpm(2400);
  M._s32_wasm_set_sensors(24.0, 55.0, 72.0, 185.0, 14.1, 14.1, 0, -1.0);
  M._s32_wasm_step_quanta(50000);
}

rebootRunning();
const healthyTab = ecu1TabColour();
M._s32_wasm_set_switches(0, 1, 1, 0, 0);          // ECU1 loses power
M._s32_wasm_step_quanta(30000);
const trimBeforeKnob = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json())).trim_pct[1];
M._s32_wasm_encoder_turn(2);
M._s32_wasm_step_quanta(30000);
const tLoss = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
const lossTab = ecu1TabColour();
check(FW_CHECKS[14], lossTab === "gray" && tLoss.trim_pct[1] > trimBeforeKnob,
      `ECU1 tab ${healthyTab} -> ${lossTab}; ECU2 trim ${trimBeforeKnob} -> ${tLoss.trim_pct[1]} %`);

rebootRunning();
M._s32_wasm_set_link(0, 0);                        // ECU1 powered, link silent
M._s32_wasm_step_quanta(30000);
const linkTab = ecu1TabColour();
check(FW_CHECKS[15], linkTab === "red", `ECU1 tab ${linkTab}`);

rebootRunning();
const tA = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
const rcK = knob("S32_SWITCH", `${(tA.time_s + 0.05).toFixed(3)}:1:plead=stop`);
M._s32_wasm_step_quanta(20000);
const tB = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
M._s32_wasm_step_quanta(20000);
const tC = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
check(FW_CHECKS[16], rcK === 0 && tC.ecu[0].coils[0].sparks === tB.ecu[0].coils[0].sparks &&
      tC.ecu[1].coils[0].sparks > tB.ecu[1].coils[0].sparks,
      `ECU1 sparks ${tB.ecu[0].coils[0].sparks} -> ${tC.ecu[0].coils[0].sparks}, ECU2 ${tB.ecu[1].coils[0].sparks} -> ${tC.ecu[1].coils[0].sparks}`);

// The page's Fuel Injectors Enable switches after a boot: ECU 2's wire must open when asked (after a boot the select
// switch's default position, ECU 1, has it grounded).
rebootRunning();
M._s32_wasm_set_switches(0, 1, 0, 0, 1);
M._s32_wasm_set_switches(1, 1, 1, 0, 1);
M._s32_wasm_step_quanta(30000);
const tSel = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json()));
check(FW_CHECKS[20], tSel.fuel_gph > 5.0 && tSel.afr < 13.0, `fuel ${tSel.fuel_gph.toFixed(2)} GPH, AFR ${tSel.afr.toFixed(2)}`);

rebootRunning();
const anFault = (mode) => M.ccall("s32_wasm_set_an_fault", "number", ["number", "number", "string"], [0, 5, mode]);
const rcOpen = anFault("open");
M._s32_wasm_step_quanta(30000);
const mapOpen = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json())).link.map((l) => l.map_inhg);
const rcClear = anFault("");
M._s32_wasm_step_quanta(30000);
const mapBack = JSON.parse(M.UTF8ToString(M._s32_wasm_get_telemetry_json())).link.map((l) => l.map_inhg);
check(FW_CHECKS[21], rcOpen === 0 && rcClear === 0 && mapOpen[0] < 10 && Math.abs(mapOpen[1] - 24.0) < 0.2 &&
      Math.abs(mapBack[0] - 24.0) < 0.2 && Math.abs(mapBack[1] - 24.0) < 0.2,
      `MAP ECU1/ECU2 ${mapOpen.join("/")} inHg disconnected, ${mapBack.join("/")} inHg cleared`);

// ---- pre-boot knob storage, on firmware that can boot ------------------------------------------------------------
// A knob the scratch validation board cannot judge (it has no RTC) but the real board refuses must fail the boot,
// naming the knob -- not be applied or dropped silently.
{
  const K = await createS32Module();
  const loadK = (buf, fn) => { const p = K._malloc(buf.length); K.HEAPU8.set(buf, p); const rc = fn(p, buf.length); K._free(p); return rc; };
  loadK(ctProg, K._s32_wasm_load_ct_firmware);
  loadK(ecBfl, K._s32_wasm_load_ec_bflash);
  loadK(ecPfl, K._s32_wasm_load_ec_pflash);
  const kknob = (n, v) => K.ccall("s32_wasm_knob", "number", ["string", "string"], [n, v]);
  const rcKnob = kknob("S32_RTC", "bogus");
  const rcBoot = K._s32_wasm_boot_plain();
  const why = K.UTF8ToString(K._s32_wasm_knob_error());
  check(FW_CHECKS[17], rcKnob === 0 && rcBoot === -4 && why.includes("S32_RTC"),
        `knob rc ${rcKnob}, boot rc ${rcBoot}: "${why}"`);
}
// A knob value longer than any fixed slot is kept whole and applied at boot: 683 pairs setting the MAP pin raw to
// 900, then a final pair setting 500 -- the run must end with 500's MAP, exactly the run with "5=500" alone.
{
  const longAn = "5=900,".repeat(683) + "5=500";        // 4103 bytes (the old fixed slot cut it at 4095, mid-pair)
  const run = async (val) => {
    const K = await createS32Module();
    const loadK = (buf, fn) => { const p = K._malloc(buf.length); K.HEAPU8.set(buf, p); const rc = fn(p, buf.length); K._free(p); return rc; };
    loadK(ctProg, K._s32_wasm_load_ct_firmware);
    loadK(ecBfl, K._s32_wasm_load_ec_bflash);
    loadK(ecPfl, K._s32_wasm_load_ec_pflash);
    if (defFram) {
      loadK(defFram, (p, n) => K._s32_wasm_load_fram(1, p, n));
      loadK(defFram, (p, n) => K._s32_wasm_load_fram(2, p, n));
    }
    const rcK = K.ccall("s32_wasm_knob", "number", ["string", "string"], ["S32_AN", val]);
    const rcB = K._s32_wasm_boot_plain();
    K._s32_wasm_step_quanta(60000);                     // 6 s: link status frames flow
    const tj = JSON.parse(K.UTF8ToString(K._s32_wasm_get_telemetry_json()));
    return { rcK, rcB, map: tj.link.map((l) => l.map_inhg) };
  };
  const a = await run(longAn), b = await run("5=500");
  check(FW_CHECKS[18], longAn.length > 4096 && a.rcK === 0 && a.rcB === 0 && b.rcK === 0 && b.rcB === 0 &&
        a.map[0] === b.map[0] && a.map[1] === b.map[1],
        `${longAn.length} bytes: MAP ${a.map[0]}/${a.map[1]} inHg vs the "5=500" run's ${b.map[0]}/${b.map[1]}`);
}

// The page's default-firmware path boots, runs the first power-up for 2 s and boots again: the second boot must run
// (the 230 KB panel buffer is static, and the module is linked with a 4 MB stack (Makefile), as the Node front end is).
{
  const R = await createS32Module();
  const loadR = (buf, fn) => { const p = R._malloc(buf.length); R.HEAPU8.set(buf, p); const rc = fn(p, buf.length); R._free(p); return rc; };
  loadR(ctProg, R._s32_wasm_load_ct_firmware);
  loadR(ecBfl, R._s32_wasm_load_ec_bflash);
  loadR(ecPfl, R._s32_wasm_load_ec_pflash);
  let detail;
  try {
    const b1 = R._s32_wasm_boot(4, 80, 80), s1 = R._s32_wasm_step_quanta(20000);
    const b2 = R._s32_wasm_boot(4, 80, 80), s2 = R._s32_wasm_step_quanta(5000);
    detail = { ok: b1 === 0 && b2 === 0 && s1 === 0 && s2 === 0, text: `boot ${b1}, step ${s1}, boot ${b2}, step ${s2}` };
  } catch (e) {
    detail = { ok: false, text: `trapped: ${e.message}` };
  }
  check(FW_CHECKS[19], detail.ok, detail.text);
}

finish();
