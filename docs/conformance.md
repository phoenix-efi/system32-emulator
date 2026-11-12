# Emulator conformance and speed

The emulator is built several ways: natively on macOS and Linux, as WebAssembly for Node, and as the browser bench's
module (`emu/wasm`) that runs in Node and in a browser. A result from any of them has to mean the same thing, so the
same stimulus must give the same output on every build, on every machine, and on every run. This page describes the
trace that defines "the same output", the gate that checks it, and how fast each build runs.

## The trace (`S32_TRACE=<file>`)

One C implementation (`emu/common/s32trace.c`) used by every front end, so every build writes the same bytes. It
records only what a bench with a logic analyser on every pin and a tap on every wire could see:

| Line | Meaning |
|---|---|
| `E1 <cycle> pin RA 0000 0001` | an output-pin change on an MCU (CT, E1, E2), driven pins only (LAT where TRIS = 0): injector drivers, tach, fuel flow, LEDs, test points, DIGOUT, chip selects, panel lines |
| `E2 <off cycle> coil OC1 on <on cycle>` | an ECU output-compare coil pulse |
| `CT <cycle> tx U3 7E` | a byte an MCU's UART sent (link pairs and consoles, every UART) |
| `q <quantum> deliver CT>E1 <n> <hex>` | the bytes delivered over a cat5 pair in one 100-us quantum, after the harness (faults applied) |
| `q <quantum> panel <vblanks> <hash> bl <per mille> lit G1CON1 .. G1CON2 .. G1CON3 ..` | the picture on the panel (hash of the 320 x 240 pixels), the backlight PWM duty, lit or dark and why, and the graphics controller's display registers; written when any of it changes |
| `end fram`, `end sd`, `end pflash`, `end cycles` | SHA-256 of each FRAM chip, SD card and ECU program flash at the end of the run, and each MCU's cycle count |

Time stamps are integers: the MCU's own cycle count (Controller Fcy, ECU SYSCLK, crystal error applied) or the
quantum number. Within a quantum the Controller's lines come first, then ECU1's, then ECU2's, then the quantum's
deliveries and panel line, so the trace does not depend on how the MCUs were scheduled. No firmware RAM, no emulator
internals (stopped-core codes, idle-skip and warp counters and the like stay in the diagnostics, not in the trace).

The panel picture comes from `emu/common/s32panel.c`, which is also what the native emulator's screen dumps
(`S32_PPM`, `S32_SNAPDIR`) and the browser canvas use. The panel is dark while the Controller is unpowered, the
graphics module (G1CON1.G1EN) or its display pins (G1CON3.DPPINOE) are off, or G1CON2.DPTEST selects a test pattern;
the stock firmware selects the black screen while it builds a new screen. The backlight duty is reported, not applied
to the pixels. RD0 ("panel enable") is a pin in the trace but does not gate the picture: the stock firmware keeps RD0
low in every run while the panel shows content (OBSERVED), so RD0 is not this panel's enable.

## The gate (`make conformance`, `scripts/conformance.py`)

Stimuli: the emulator's own cases (`tests/conformance/cases.json`) and the benchmark scenarios
(`tests/bench/scenarios.json`), in the case format of `scripts/s32case.py`. Home-page times `{H+x}` in the knobs are
left for the emulator to place at the home page it detects; a run length `{H+x}` is fixed at 5 + x s because the
browser bench's module takes a number (`--home 5` restores the older fixed home time). Each case's inputs are prepared
once and every build runs on a fresh copy of them. A case that takes another case's output gets the reference run's
output. These cases exercise the hardware model; none of them judges what the firmware does.

### Case providers (`--provider FILE.py`)

A firmware project with its own stimuli runs them through this gate without copying it: `--provider FILE.py` loads a
module that defines

* `load_cases(tier, only, home)` -> a list of `{"id", "group", "case", "deps"}` (`case` holds a `stimulus` with
  `secs`; `deps` are ids of cases whose reference outputs this one reads),
* `prepare(group, case, fw, dir, done)` -> `(env, secs, ...)`: the case's inputs written into `dir` and the
  environment of its emulator run (`done` maps an earlier case's name to its reference run directory),
* optionally `needs_boot_code(group, case)` -> a reason string when the case cannot run on an application-only ECU.

`scripts/s32case.py` is the default provider and the reference implementation. The snapshot (`snapshot`,
`snapshot-self`, `snapshot-wasm`) and remote builds work the same for every provider.

Builds (the first is the reference; every other run must give a byte-identical trace and the same exit code):

| Build | What runs |
|---|---|
| `native` | `emu/pair/build/s32pair_shim`, one thread per MCU (the reference) |
| `native-again` | the same binary again |
| `native-serial` | `S32_SERIAL=1`: the three MCUs one after another on one thread |
| `native-scribble` | fresh heap memory filled with a pattern by the C library (catches reads of uninitialised heap) |
| `wasm-node` | `emu/pair/s32pair.c` compiled to WebAssembly (`make -C emu/wasm node`), in Node |
| `wasm-bench` | the browser bench's module through its exported API, in Node (`emu/wasm/test/run_job.mjs`) |
| `browser` | the same module in headless Chromium (`scripts/browser_job.py`); compared by the trace's SHA-256 |

`--remote user@host` copies the tree to a second machine, builds it there with that machine's compiler, runs its
builds alongside, and compares them with the local reference: the cross-platform check. On a mismatch the gate prints
the first differing trace line with the two lines before it.

```bash
make conformance-synthetic                                     # no firmware: generated images, every local build
S32_FIRMWARE_DIR=<dir> make conformance                        # + the emulator's cases and scenarios on <dir>
make check                                                     # the same on the default firmware (make firmware)
python3 scripts/conformance.py --firmware-dir <dir> --builds native,native-again,wasm-node,wasm-bench \
    --remote user@host --remote-env 'PATH=...'                 # and a second machine
```

Without firmware the firmware cases SKIP. The synthetic run (generated Controller and ECU images that drive their
ports; `conformance-synthetic`) is part of `make check`, WebAssembly builds included; `make conformance-browser` adds
headless Chromium. Both run in the pinned environment (`nix develop`: Emscripten 4.0.10, Node 20.19.1).

What the browser bench's module cannot express is SKIPped with the reason: ECU SD cards, per-ECU program-flash images,
replay and the live sensor feed.

## What `make check` compares

On the default firmware: the 9 cases of `tests/conformance/cases.json` and the 3 benchmark scenarios, each on
`native` (the reference), `native-again`, `native-serial`, `wasm-node`, `wasm-bench` and `snapshot`; on generated
images (no firmware): `native`, `native-again`, `native-serial`, `native-scribble`, `wasm-node`, `wasm-bench`,
`snapshot`, `snapshot-self` and `snapshot-wasm`. Every comparison must be identical. `--remote user@host` adds
another machine's builds (the cross-platform check: macOS arm64 against Linux x86-64, say).

What the emulator guarantees for the gate, each with its test:

* **Text logs in a fixed order.** `S32_LINKLOG`, `S32_STATUSLOG`, `S32_SPARKLOG` and `S32_INJLOG` lines are buffered
  per MCU thread and merged between quanta in emulated-time order (ties: Controller, ECU1, ECU2), so they never tear or
  reorder. Test: `emu/pair/test/log_order.sh` (two threaded runs and one serial run, byte-identical logs).
* **Reproducible SD card images.** `scripts/sdcard_image.py` writes a fixed volume ID and fixed root-directory time
  stamps, so a case that builds a card gives the same trace every time it is prepared.
* **One panel renderer.** The native screen dumps and the browser canvas both use `emu/common/s32panel.c` (black while
  G1CON2.DPTEST selects the test pattern, as the panel shows it). Test: `emu/pair/test/test_trace.c`
  (`panel_lit_and_dark`).
* **No fused multiply-add.** Every emulator build uses `-ffp-contract=off`, so arm64, x86-64 and WebAssembly round the
  harness arithmetic (crank edge times, engine angle, clock targets, the wideband sensor lag) the same way.
* **Browser bench start-up.** The crystal-error knob (`S32_XTAL_PPM`) is applied before the ECU clocks are set. Knob
  values set before boot are kept whole, however long; the 65th pre-boot knob is refused with a reason, and a knob
  that fails when it is applied at boot fails the boot with the knob named.

The bench page's own start (`s32_wasm_boot`) sets its sensor panel defaults
(24 inHg, 55 % TPS, 72 F, 185 F, 14.1 V), sizes the engine at 90 cu in per cylinder, and drives RPM from the page,
so `S32_RPM` set as a knob does nothing there. The conformance runs use `s32_wasm_boot_plain`, which starts the harness
exactly as `s32pair` does.

Limits:

* The `browser` build is compared by the trace's SHA-256 only; a mismatch there is examined in Node.
* The bench module cannot load ECU SD cards or per-ECU program-flash images.
* A case's run length `{H+x}` is fixed at 5 + x s (its knob times stay home-relative); a provider's own runner may
  pass `{H+x}` run lengths to the emulator, which the gate does not exercise.
* The live sensor feed and datalog replay are not in the stimulus set.

## Snapshots: boot once (`S32_SNAPSHOT_OUT`, `S32_SNAPSHOT_IN`, `S32_STIMULUS`)

Most cases spend their first seconds doing the same thing: the Controller and both ECUs boot from reset, the engine
starts, the home page comes up, and only then does the case's own stimulus begin. The emulator can save the whole
system at a quantum boundary and start another run from that state, so the common part runs once.

**What a snapshot holds** (`emu/common/s32snap.h`; a gzip file of tagged chunks): the Controller (PIC24 core, RAM,
program flash as erased and written, every SFR, timers, SPI, UARTs with their queues, the graphics controller), each
ECU (SoC SFRs, timers, SPI, UARTs, pin queue, output-compare state, flash chip, watchdog, the MIPS core's registers,
RAM, flash copies, decoded-block cache and code hooks), the three FRAM chips, the RTC, each SD card (its controller state
and the blocks written since it was loaded), each ECU's program-flash image, the harness (power, engine and its angle
history, sensors, noise generator state, switches and contact bounce, link faults and the link mux, the event queue
and the position in it, every fault's progress, the injector meter), the front end's link analysers, status decoder,
trim tracker, injector and spark statistics, and the bytes already written to the trace and the logs. Pointers are
not stored: a restored run sets itself up the normal way and copies the state over it. A snapshot holds the firmware's
RAM because it must; nothing reads values out of it, and the checks still observe only physical interfaces.

**Which runs may share a snapshot.** `S32_STIMULUS=<file>` makes the emulator report what a run's start-up and knobs
set up, without running: a hash of the static part (every MCU, chip and card as initialised, every board setting not
tied to a time, the front end's options, the emulator build) and every timed item (each event, each analogue, pin,
crank and link fault, the engine start, the statistics window) with its time. A snapshot taken at t can start a run
whose static part is the same and whose timed items before t + 10 ms are the same, in the same order; from then on its
own items replace the snapshot's run's. `S32_SNAPSHOT_IN` checks this itself and refuses (exit 5, with the reason) a
run that does not match. Replay, the live sensor feed, I/O coverage and ECU console capture cannot snapshot; a restored run
cannot use `S32_SNAPDIR` (the screens before t would be missing).

**The emulator build** in the static part is the front end's build stamp (`BUILD_STAMP`, `emu/pair/s32pair.c`): the
content identity of the emulator's sources and headers, the generators of its build-time headers and the register
models they read, its Makefiles, the compiler's version and the flags (`emu/common/build_id.sh`), never the time of the
build. Two builds of the same tree carry the same stamp and load the same bytes (measured: two clean builds of
one tree in two directories give the same loaded-code identity on macOS arm64 and on Linux x86-64), so the stamp
identifies the exact build (in a snapshot and in the stimulus signature, `S32_STIMULUS`) across clean builds, checkouts
and machines. A change to those
inputs gives another stamp, so a snapshot is not restored into different code. The stamp hashes the pair's own
compiler and flags: a library rebuilt by its own make with other flags, or a hand-edited file under `build/`, keeps
the stamp (measured). make compares file times to the second on macOS,
so a new id is written a second after the recipe starts and the binaries always relink. A binary built without the
Makefiles falls back to its compile time.

Home-page times: an item at `{H+x}` (`x >= 0`; pre-home `{H-x}` times are refused) is reported as `[kind, sub, x, hash, 1]`. The snapshot records the home time the
run had reached (or -1). When a restore is checked, a `{H+x}` item counts as before t + 10 ms when the snapshot's home
time plus x is (home page not reached yet: when t + x is, the earliest it can act). A restored run places its own
home-page times from the snapshot's home time. `S32_SNAPSHOT_OUT` accepts
`<file>@{H+x}`: the snapshot is taken x after the home page the run detects (a point that has already passed when the
home page is detected is taken at the end of that quantum).

**A case runner** (a provider's, for one) asks the emulator for every case's stimulus, groups the cases with the same static
part, and in each group lets the case whose snapshots save the most emulation run from power-on writing one snapshot
per start point; every other case that saves at least 0.5 s starts from its snapshot. Where two cases first differ at a
home-page item the point is asked for as `{H+x}`, so cases that are the same up to the home page share that boot; the
runner orders home items among fixed-time ones with a guessed home time of 4 s, and the emulator's own check catches a
wrong guess. Cases that take another case's output and the screen-calibration runs start from power-on. A run that cannot use its snapshot
is repeated from power-on, so the snapshot machinery never decides a result. `--no-snapshot` runs every case from
power-on.

**The gate.** `scripts/conformance.py` has three snapshot builds, compared with the reference like every other build:

| Build | What runs |
|---|---|
| `snapshot` | the case runner's grouping: the leaders (writing snapshots) and every case started from a leader's snapshot |
| `snapshot-self` | each case writes snapshots at 1/4, 1/2 and 3/4 of its length (a case with home-page times also at `{H}` and `{H+1}`) and is restored from each with its own knobs |
| `snapshot-wasm` | `snapshot-self` with the WebAssembly build (`s32pair` compiled to WebAssembly, in Node) |

The trace compared is the whole file: the restored run writes the snapshot run's trace bytes first and continues the
trace's running SHA-256, so a restored trace equals a straight run's from the header to the final FRAM, SD, flash and
cycle-count lines. `emu/pair/test/snapshot.sh` (generated images, part of `make -C emu/pair test`) also compares the
logs and the end-of-run report, restores a run started from a snapshot, and checks that mismatched starts are refused.
The generated-image runs are in `make conformance-synthetic`.

A snapshot of a running system is about 0.9 MB.

Limits:

* A snapshot is restorable only by the build that wrote it (the chunk "build" and a layout fingerprint are checked):
  a native snapshot does not load in WebAssembly or in another build of the emulator.
* The browser bench's module (`emu/wasm/src/s32_wasm.c`) has no snapshot API; WebAssembly snapshots are those of
  `s32pair` compiled for Node.
* The restored run's stderr starts with its own start-up lines and a "snapshot: restored" line instead of the
  snapshot's run's lines up to t; the end-of-run report (stdout) and every file are the same.
* The emulator finds the home page inside the run (`{H}`, `@{H+x}`), so home-page cases can start from a snapshot;
  a case with a phase-anchored time (`~`) starts from power-on.

## Speed (`make bench`, `scripts/bench.py`)

Simulated seconds per wall-clock second, the emulator's own figure (its loop only, no process start, firmware load
or WebAssembly compile), median of 3 runs, one emulator at a time, on the three scenarios of
`tests/bench/scenarios.json` with the default firmware. `native` uses three cores (one thread per MCU); every other
build uses one.

**Apple M4 Pro** (14 logical CPUs, macOS 15), clang 16.0.6 from the pinned environment, `-std=gnu11 -O2 -g
-ffp-contract=off`; Emscripten 4.0.10 `-O3 -flto -ffp-contract=off`; Node 20.19.1. Load average 4 to 5 during the
run.

| scenario | native | native, one thread | WebAssembly (Node) | bench module (Node) |
|---|---|---|---|---|
| boot | 11.6x | 6.4x | 4.8x | 5.1x |
| cruise 2400 RPM, 30 s | 12.8x | 6.3x | 4.7x | 5.0x |
| crank fault | 12.0x | 6.0x | 4.5x | 4.8x |

One thread per MCU runs about twice as fast as one thread for all three. On one core the WebAssembly builds run at
about 75 % of native speed. Most of the emulated time is idle loops, which the models skip
([emulator-fidelity.md](emulator-fidelity.md), Known limits); `S32_NOIDLESKIP=1` executes every instruction and runs
three to four times slower.
