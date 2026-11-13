# System32 emulator

An emulator of the EFII System32 engine-control system: the Cockpit Controller (PIC24FJ256DA210, its 320 x 240
panel, bezel buttons and knob, SD card, FRAM, clock) and both ECUs (PIC32MX on a pure-C11 MIPS32 core, crank and cam
sensors, coils, injectors, analogue inputs, FRAM, SD), wired together over the serial link, as on the aircraft, and
driven by an engine and harness model. It runs the unmodified firmware images: natively on macOS and Linux, and as
WebAssembly in Node and in the browser. It is a desk copy of the system for getting to know it: the screens, menus,
link, sensors and outputs behave as the firmware drives them.

Every build gives the same output, byte for byte, for the same stimulus: native = WebAssembly = browser, checked by
the conformance gate (`docs/conformance.md`). What is modelled, at what fidelity, and the known limits are in
`docs/emulator-fidelity.md` and `docs/io-ledger.md`.

Phoenix EFI (replacement firmware for the same hardware) uses this emulator.

Not for flight. Results from the emulator describe the model under the conditions of a run; they do not establish
airworthiness or suitability for any aircraft or engine. Not affiliated with Protek Performance / FlyEFII (`NOTICE`).

## Build and test

Everything runs in one pinned environment (Nix flake: pinned compiler, Python, uv, Node and Emscripten revisions):

```bash
nix develop                     # pinned tools (nix/tool-versions.txt); Python packages from uv.lock
make emu                        # native front end: emu/pair/build/s32pair_shim
make test                       # emulator unit tests (C) and tooling tests (Python)
make wasm wasm-test             # WebAssembly: the browser bench module and the Node front end, smoke test
make conformance-synthetic      # native = WebAssembly = snapshots, on generated images (no firmware)
make firmware                   # fetch and verify the default firmware (below)
make conformance                # native = WebAssembly on the firmware: tests/conformance/cases.json + scenarios
make check                      # all of the above plus generated sources, cppcheck
make check-nofw                 # the same without any firmware
```

`nix build` builds the native emulator and runs its unit tests in the Nix sandbox.

## Run

```bash
FW=.firmware-cache/v86
S32_ECU1_FRAM=$FW/default_ecu.fram S32_ECU2_FRAM=$FW/default_ecu.fram S32_RPM=2400 S32_START=2 S32_PPM=screen.ppm \
  emu/pair/build/s32pair_shim $FW/CT086001_prog.bin $FW/v86_ec086001_bflash.bin $FW/v86_ec086001_pflash.bin 10
emu/pair/build/s32pair_shim --knobs           # every harness knob: engine, sensors, faults, switches, bezel, power
make serve-bench                              # the browser emulator with the default firmware, http://127.0.0.1:8787/
```

The front-end options (logs, screens, snapshots, traces, datalog replay) are in the header of
`emu/pair/s32pair.c`; what the bench observes is in `docs/simulator.md`; the trace and the conformance gate in
`docs/conformance.md`; the hardware I/O the model covers in `docs/io-ledger.md` (`model/io_ledger.json`).

## In the browser

`make serve-bench` serves the emulator page (`emu/wasm/web/cockpit.html`) with the default firmware. It opens
running, at idle, with the Controller drawn as the panel instrument (screen, MENU, TUNE, MONITOR and the knob), the
panel switches (Ignition 1 and 2, fuel pump, ECU Select, starter), a throttle and a prop lever.

* **Presets** load a starting state (switches, levers, faults, configuration): first start, run-up, cruise, takeoff,
  ECU 1 silent, MAP sensor failed, ECU 2 fuels, RPM limit, lean trim. Each has optional checklist steps.
  **Reset** returns to the default state.
* **Links** open the page in a preset: `cockpit.html?scenario=runup`. `&embed=1` shows only the Controller and the
  switches, for embedding in another page.
* **Advanced**, below, holds every bench control and reading: sensors, faults, per-ECU switches, coil, injector and
  link readouts, the live fuel and ignition maps, file drop and export, datalog replay and the ECU console.

The engine behind the levers is a simple bench model (governor, manifold pressure, spark); its numbers are in
`docs/simulator.md`. `make bench-scenarios` opens every preset in headless Chromium and checks its state.

## Firmware

**No firmware is in this repository, and none is covered by its licence.**

### Bring your own

A firmware directory for the native front end holds `*_prog.bin` (Controller program memory), `*_bflash.bin` +
`*_pflash.bin` (ECU boot and program flash) or an application-only `*_app.flat`, and `default_ecu.fram`. Point the
gates at it with `S32_FIRMWARE_DIR=<dir>`. The browser bench takes files dropped on the page: Controller and ECU
update files (`CTvvvbbb.bin`, `ECvvvbbb.bin`, decoded in the page), flash images, `.fram` configuration chips, an SD
card image, a `DATALOG.CSV`. Nothing leaves the browser.

### The default firmware (stock v86, FlyEFII's copyright)

By default the emulator loads the stock EFII System32 v86 build 001 update files, `CT086001.bin` and `EC086001.bin`,
unmodified. They are FlyEFII's copyright, **not Apache-2.0** (`firmware/LICENSE-FIRMWARE`), never committed, and
published separately as release assets. `firmware/manifest.json` names each file with its size and SHA-256, and the
four images derived from them with theirs.

```bash
make firmware                                   # download the release assets into .firmware-cache/ (git-ignored)
S32_V86_SRC=<dir> make firmware                 # or copy them from a local directory instead of downloading
```

`scripts/fetch_firmware.py` refuses any file whose size or SHA-256 differs from the manifest, then derives the
emulator images (the ECU's default configuration chip is written by the emulator itself, running the firmware's first
power-up on a blank chip) and checks each of those against the manifest too.

## Layout

| Path | What |
|---|---|
| `emu/pic24`, `emu/controller` | PIC24 core; the Cockpit Controller (graphics, panel, peripherals, flash) |
| `emu/ecu` | the ECU (PIC32MX peripherals) on the C11 MIPS32 shim (`emu/ecu/shim`) |
| `emu/common` | harness and engine model, link decoder, FRAM layout, trace, panel, snapshots |
| `emu/pair` | the native front end (`s32pair`): Controller + two ECUs, 100-us quanta |
| `emu/wasm` | the WebAssembly builds and the browser page (`emu/wasm/web/cockpit.html`, `s32_bench.js`) |
| `model/` | SFR and pin models generated from Microchip's public DFPs; the I/O ledger |
| `scripts/` | conformance, bench, case preparation, SD card images, generators, firmware fetch |
| `tests/` | conformance cases, benchmark scenarios, tooling tests |
| `firmware/` | the default firmware's manifest and licence notice (no images) |

The emulator's build stamp (`emu/common/build_id.sh`) hashes paths relative to the repository root, so a project that
embeds this tree at the same layout gets the same stamp for the same sources.

## Licence

Apache-2.0 (`LICENSE`, `NOTICE`) for everything in this repository. The firmware images are not part of it
(`firmware/LICENSE-FIRMWARE`).
