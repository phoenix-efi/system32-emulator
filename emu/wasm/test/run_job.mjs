// Run one emulator job on the browser bench's WebAssembly module (build/s32_wasm.js) in Node: the conformance gate
// (scripts/conformance.py) and the speed benchmark (scripts/bench.py) use it to drive the module the browser page
// loads, through its exported C API, with the harness exactly as emu/pair's s32pair starts it (s32_wasm_boot_plain).
//
//   node test/run_job.mjs job.json
//
// job.json: {"ct": file, "bfl": file, "pfl": file, "fram": {"ct": file|null, "e1": file|null, "e2": file|null},
//            "ct_sd": file|null, "knobs": [["S32_RPM", "2400"], ...], "secs": 10, "trace": file|null}
// Prints one JSON line: {"rc", "secs", "wall_s", "x_real_time", "trace_sha256", "trace_bytes"}.
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));
const job = JSON.parse(fs.readFileSync(process.argv[2], "utf8"));
const { default: createS32Module } = await import(path.resolve(here, "../build/s32_wasm.js"));
const M = await createS32Module();

function load(file, fn, ...extra) {
  const buf = fs.readFileSync(file);
  const ptr = M._malloc(buf.length);
  M.HEAPU8.set(buf, ptr);
  const rc = fn(ptr, buf.length, ...extra);
  M._free(ptr);
  return rc;
}
function fail(msg) {
  console.log(JSON.stringify({ rc: 2, error: msg }));
  process.exit(2);
}

if (load(job.ct, M._s32_wasm_load_ct_firmware) < 0) fail("Controller image refused");
if (job.bfl && load(job.bfl, M._s32_wasm_load_ec_bflash) < 0) fail("ECU boot flash refused");   // none: application start
if (load(job.pfl, M._s32_wasm_load_ec_pflash) < 0) fail("ECU program flash refused");
const fram = job.fram || {};
[["ct", 0], ["e1", 1], ["e2", 2]].forEach(([k, target]) => {
  if (fram[k]) { if (load(fram[k], (p, n) => M._s32_wasm_load_fram(target, p, n)) < 0) fail(`FRAM ${k} refused`); }
  else M._s32_wasm_clear_fram(target);
});
if (job.ct_sd && load(job.ct_sd, M._s32_wasm_load_sd) < 0) fail("SD image refused");
// s32pair applies the knobs in the harness's own table order (s32board_knobs_from_env); so does this.
const orderPtr = M._s32_wasm_knob_names();
if (!orderPtr) fail(`knob names: ${M.UTF8ToString(M._s32_wasm_knob_error())}`);
const order = JSON.parse(M.UTF8ToString(orderPtr));
const knobs = [...(job.knobs || [])].sort((a, b) => order.indexOf(a[0]) - order.indexOf(b[0]));
// Knob strings go on the module's heap, not through ccall's "string" arguments: ccall copies those onto the 64 KB
// WebAssembly stack, and a longer value overruns it (memory corruption; OBSERVED: a 70 KB S32_AN crashed the boot).
function heapString(s) {
  const b = new TextEncoder().encode(s);
  const p = M._malloc(b.length + 1);
  M.HEAPU8.set(b, p);
  M.HEAPU8[p + b.length] = 0;
  return p;
}
for (const [name, value] of knobs) {
  if (!order.includes(name)) fail(`not a harness knob: ${name}`);
  const pn = heapString(name), pv = heapString(value);
  const rc = M._s32_wasm_knob(pn, pv);
  M._free(pn); M._free(pv);
  if (rc) fail(`knob ${name}=${value}: ${M.UTF8ToString(M._s32_wasm_knob_error())}`);
}
M._s32_wasm_trace_enable(job.trace ? 1 : 0);
const brc = M._s32_wasm_boot_plain();
if (brc) fail(`boot failed (${brc})`);

const out = job.trace ? fs.openSync(job.trace, "w") : null;
let traceBytes = 0;
function drain() {
  if (!out) return;
  const n = M._s32_wasm_trace_len();
  if (!n) return;
  const p = M._s32_wasm_trace_ptr();
  fs.writeSync(out, M.HEAPU8.subarray(p, p + n));
  traceBytes += n;
  M._s32_wasm_trace_clear();
}

const quanta = Math.trunc(job.secs * 1e6 / 100);   // 100-us quanta, as s32pair: (long)(secs * 1e6 / QUANTUM_US)
const t0 = process.hrtime.bigint();
for (let done = 0; done < quanta;) {
  const n = Math.min(1000, quanta - done);
  if (M._s32_wasm_step_quanta(n)) fail("step failed");
  done += n;
  drain();
}
const wall = Number(process.hrtime.bigint() - t0) / 1e9;
let sha = null;
if (out) {
  sha = M.UTF8ToString(M._s32_wasm_trace_finish());
  drain();
  fs.closeSync(out);
}
console.log(JSON.stringify({ rc: 0, secs: job.secs, wall_s: wall, x_real_time: job.secs / wall, trace_sha256: sha,
                             trace_bytes: traceBytes }));
