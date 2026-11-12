# Interactive 3-MCU WebAssembly Simulator

The **System32 Emulator** runs all three microcontrollers (`PIC24FJ256DA210` Cockpit Controller + `ECU 1` and `ECU 2`
`PIC32MX795F512L`) in lockstep inside your browser.

- **It starts with the stock firmware.** With nothing dropped, the Controller and both ECUs run the stock EFII System32
  v86 build 001 update files, unmodified (`firmware/manifest.json`). They are FlyEFII's copyright, are not in this
  repository and are not covered by its Apache-2.0 licence (`firmware/LICENSE-FIRMWARE`). It is not for flight. The
  page shows which firmware is loaded and where it came from.
- **Your own firmware replaces the default.** Drop your own lawfully obtained `CTvvvbbb.bin` and `ECvvvbbb.bin` (or
  `.fram` / `DATALOG.CSV`) into the emulator: another stock version, or a build of your own. All decoding and emulation
  run in your browser; no file is uploaded.

## The default firmware

`make bench-site` builds the page into `build/bench-site` (git-ignored) with the two update files that
`scripts/fetch_firmware.py` downloads from this repository's release assets (or copies from `S32_V86_SRC`). Each file is
checked against the size and SHA-256 in `firmware/manifest.json` before anything is derived from it, and the page checks
every file against the manifest's SHA-256 again before it loads it; if any check fails, nothing is loaded.

A project that serves this page with its own default firmware writes its own `firmware/manifest.json` next to the page
(`files`, and optionally `label` and `note`); with no manifest the page loads nothing and you bring your own.

When the configuration chips start blank, the firmware writes its default settings into them on its first power-up (as
on a new ECU; the default configuration is a 4-cylinder engine) and fires from the next power-up, so the page runs that
first power-up and power-cycles once before it hands the bench to you.

### Application start

An application-only image has no boot flash and nothing in program flash below the application window. A full image
(boot flash and program flash) starts from the reset vector and enters the application later, at the start of the
application window + 0x1000. With no boot region the emulator starts the application directly at that entry point, at
the time and in the state in which a full image reaches it on a power-up with no update to install: the
coprocessor-0 and peripheral registers, the pins and the configuration words, with a second, small table for a card in
the slot (no update file on it). The CPU models, unlike the front ends and the harness, see every instruction fetch;
the hand-off point (the first fetch in the application window), its time and the state there were measured that way
on the stock images and are declared as a table in `emu/ecu/src/ecu.c` ("application start"); every power-up and
reset of the ECU goes through it. A full image still starts from the reset vector as before.

What changes compared with the full image (observed):

* the hand-off happens when a full image reaches it on a power-up with no card, 204.7 us after power-on
  (`ECU_APP_BOOT_CYCLES`); `S32_ECU_BOOT_CYCLES=0` starts the application at once instead. Until then the ECU's pins
  stay as at reset, where a full image already drives some of them;
* program flash below the application window reads erased;
* with a card in the ECU slot (no update file on it) a full image reaches the hand-off 8.10 ms after power-on, with
  the card interface on. Application start waits that long and hands over in that state; with a full image the time
  depends on what the card holds (647,850 to 648,558 cycles on the cards tried), so that ECU may start a few
  microseconds earlier or later than with a full image. The card is not read before the application reads it;
* nothing can install an ECU update file: installing one from the card needs the boot region. A case that needs it
  reports SKIP with the reason "needs the ECU boot region".

## What the simulator observes

The simulator is a bench, not a debugger. It runs any firmware that keeps the documented interfaces, and it learns
about the system only the way a bench with real hardware could:

| Observes | How | Used for |
|---|---|---|
| Output pins | coil and injector driver edges | spark advance, dwell, injector pulse widths, fuel delivered |
| Analogue inputs it drives | MAP, IAT, ET, TPS, battery, wideband O2 voltage | the engine and sensor panel |
| Link bytes | frames on the Controller to ECU wires, decoded by `emu/common/link_decode.c` | live RPM, duty, trim, MAP, ET, IAT, advance per ECU; the replay trim follower |
| FRAM chips | the documented layout `model/s32_fram.ksy` (`emu/common/fram_format.c`) | the map view, and map edits before boot |
| LCD panel | the graphics controller's display buffer as the panel shows it (`emu/common/s32panel.c`, the same renderer as the native emulator's screen dumps): dark while the Controller is unpowered, the graphics module or its display pins are off, or the controller shows its test pattern (the stock firmware selects the black screen while it builds a new screen) | the Controller screen |

It never reads or writes a firmware's RAM; no front-end code does.

### Program flash and watchdog

Each ECU's program flash is a chip that keeps what its boot code erased and programmed across power cycles and resets
(an update interrupted by a power cut leaves exactly what was programmed). `S32_ECU1_PFLASH` / `S32_ECU2_PFLASH` load a
different image per ECU and `S32_ECU1_PFLASH_OUT` / `S32_ECU2_PFLASH_OUT` read it back at exit, as a device programmer
would. The Controller's program flash is a chip too: what the application erases and programs (the firmware-update
request rewrites one flash page) survives power cycles and resets; `S32_CT_PFLASH` loads another image (or `blank`) and
`S32_CT_PFLASH_OUT` reads it back at exit, 3 bytes per 24-bit instruction word. Each ECU core has its watchdog timer
(DEVCFG1.FWDTEN or WDTCON.ON, period 1 ms x 2^WDTPS, cleared by WDTCLR); when it expires the harness resets that core
with RCON.WDTO set, also while the core is stopped on an exception. The stock images leave it off.

### Wideband AFR (bench physics)

The AFR gauge is computed the way the engine makes it, from the injector pins (`emu/common/bench_engine.c`):

* **Fuel.** Each injector output channel drives one injector. Both ECUs' drivers are wired in parallel to it, so the
  injector is open while either driver is on (the union of the two pulses). Each opening delivers fuel for its open
  time minus the injector dead time, at the injector's static flow (60 lb/hr). The dead time follows the injector's
  supply, the aircraft bus that feeds both ECU breakers: 0.80 ms at 13.8 V, longer as the bus falls (about 1.06 ms at
  11.5 V), no opening below 6 V. The curve has the usual shape of a high-impedance injector's published data (INFERRED;
  no data sheet for the installed injector); `S32_INJ_DEAD=<ms>` fixes it instead. An ECU whose injector-enable
  wire is grounded does not reach the injectors: the installation's ECU Select switch grounds one of the two wires
  (ECU 1 selected by default), so normally only one ECU fuels.
* **Air.** Speed-density for a four-stroke engine: VE x displacement x RPM / 2 x MAP / (R x T_IAT), with 90 cu in per
  cylinder (360 cu in on four cylinders, 540 on six) and VE 0.868.
* **Sensor.** A PLX SM-AFR wideband module: 0-5 V linear output, 10.0 AFR at 0 V to 20.0 AFR at 5 V (gasoline), with
  a 0.1 s response, into the Controller's AFR input through its 4:1 input divider. The Controller's own O2 display
  scale and offset turn that into the number on its screen.
* **Calibration, done once.** VE 0.868 makes a six-cylinder configuration's maps read AFR 13.3 at 2400 RPM /
  24.0 inHg / IAT 59 F / trim 0 with one ECU fuelling. No other point is fitted. With both ECUs fuelling, the parallel
  injector pulses overlap to a 3.4 % longer opening and the gauge reads 12.85. The default maps on a 4-cylinder
  engine read about 11.2.

### Map editor

Before you press Boot, edits go into the ECU FRAM images, as you would prepare a configuration chip. While the system
runs, an edit is sent the way the Controller writes maps: a table-write frame on the link to each ECU, which checks
it, stores it in its FRAM and acknowledges it. The view shows ECU 1's FRAM chip. The bench paces table frames at half
the line rate. The line under the table shows each ECU's write state (sent / acknowledged / refused / not
acknowledged); the native front end ends every run with a count of any table write that crossed the cable but was not
acknowledged.

### ECU faults

* **ECU Power off** cuts the ECU and drops the Controller's power-sense input for it, so the Controller shows that ECU
  gray ("not seen"), as on the aircraft when its breaker is pulled.
* **Link Data Wires off** keeps the ECU powered and running but silences its link both ways (a broken data pair, a
  failed transceiver, a hung ECU). The Controller shows that ECU red, the colour it also uses for a grounded
  P-lead.
* **ECU 1 MAP Sensor** disconnects ECU 1's MAP input or shorts it to ground or 5 V; ECU 2 keeps its own sensor.

### Panel switches

**Fuel Pump 1** and **Fuel Pump 2** drive the Controller's pump inputs (the PMP1 / PMP2 tabs). **ECU Select** grounds
the Fuel Injectors Enable wire of the ECU that is not selected, so only the selected ECU fuels; the page starts with
ECU 1 selected, as the installation does. The two Fuel Injectors Enable boxes are the same two wires, set one at a time.

## Scenarios and links

The Scenarios list on the bench page sets the bench up in a named state. A page can link straight to one:

    index.html?scenario=<id>              set up, then pause (press 1x to run)
    index.html?scenario=<id>&autostart=1  set up and keep running

The scenario is set up once the default firmware has loaded and its first power-up and power cycle have run. Each one
starts from the same point: the page's default controls with the engine stopped, a power cycle with the configuration
kept, the Controller's home page, fuel trim 0 %. An unknown id shows a short notice and the bench runs normally.
Picking a scenario from the list puts its id in the address bar, so the link can be copied from there.

| id | State |
|---|---|
| `first-start` | configuration chips erased, first power-up, engine stopped: both ECU tabs red, no sparks |
| `runup` | 1800 RPM, 20 inHg, both ECUs firing, ECU 1 selected, pump 1 on |
| `cruise` | 2400 RPM, 24.0 inHg, fuel trim 0 % |
| `takeoff` | 2700 RPM, 29.0 inHg, full throttle |
| `ecu-silent` | cruise, ECU 1 powered but its link silent: ECU1 tab red, its values shown as dashes |
| `map-fail` | cruise, ECU 1's MAP sensor disconnected; selecting ECU 2 brings the AFR back |
| `ecu-select` | cruise on ECU 2: ECU 2 fuels, ECU2 tab blue |
| `rpm-limit` | 2950 RPM; with this configuration the ignition cuts at 3000 RPM |
| `trim-lop` | cruise with the fuel trim at -28 %: the bench AFR reads about 15.5 |

The states in the right-hand column are what the stock v86 firmware shows on this bench. The scenarios are one table in
the page (`SCENARIOS` in `emu/wasm/web/index.html`); `make bench-scenarios` opens each link in headless Chromium and
checks that it sets up without a JavaScript error and reaches that state (`scripts/bench_scenarios.py`;
`SCENARIO_ARGS="--shots DIR"` saves screenshots).

### The harness and its knobs

Everything outside the three MCUs -- the engine and its crank / sync sensors, every sensor and its wiring, the cockpit
switches, the power supplies, the cat5 data pairs, the consoles, the FRAM, RTC and SD chips -- is one model shared by the
browser bench and the command-line co-simulator (`emu/common/s32board.c`). Its knobs have the same names in both: the
command line reads them from the environment (`s32pair --knobs` lists them), the page passes them through
`s32_wasm_knob(name, value)`. For example `S32_AN_FAULT=1:5=gnd@30` shorts ECU 1's MAP input to ground at t = 30 s,
`S32_SWITCH=40:select=2` moves the ECU Select switch, `S32_SUPPLY=2:5.0@50,2:13.8@50.3` sags ECU 2's supply below the
regulator's drop-out for 0.3 s. The [I/O ledger](io-ledger.md) lists every signal with its model, faults, knob, test and
evidence.
