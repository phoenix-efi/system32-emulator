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
| `rpm-limit` | ECU setting 7 (the RPM limit, 3000 by default) set to 2750 in both ECUs, 2700 RPM; at 2750 the ignition cuts |
| `trim-lop` | cruise with the fuel trim at -28 %: the bench AFR reads about 15.5 |

The states in the right-hand column are what the stock v86 firmware shows on this bench. `rpm-limit` writes its
setting into both ECUs' configuration chips the way the documented layout holds it (`model/s32_fram.ksy`: a 16-byte
text slot per setting, then the checksum) and power-cycles, so the ECUs read it as they read any configuration; with
2750 there both ECUs stop firing at 2750 RPM (observed). The next scenario puts the old value back.

The scenarios are one table that both pages load (`SCENARIOS` in `emu/wasm/web/s32_bench.js`); `make bench-scenarios`
opens each link on both pages in headless Chromium and checks that it sets up without a JavaScript error and reaches
that state (`scripts/bench_scenarios.py`; `SCENARIO_ARGS="--shots DIR"` saves screenshots, `--pages cockpit` runs one
page).

### The harness and its knobs

Everything outside the three MCUs -- the engine and its crank / sync sensors, every sensor and its wiring, the cockpit
switches, the power supplies, the cat5 data pairs, the consoles, the FRAM, RTC and SD chips -- is one model shared by the
browser bench and the command-line co-simulator (`emu/common/s32board.c`). Its knobs have the same names in both: the
command line reads them from the environment (`s32pair --knobs` lists them), the page passes them through
`s32_wasm_knob(name, value)`. For example `S32_AN_FAULT=1:5=gnd@30` shorts ECU 1's MAP input to ground at t = 30 s,
`S32_SWITCH=40:select=2` moves the ECU Select switch, `S32_SUPPLY=2:5.0@50,2:13.8@50.3` sags ECU 2's supply below the
regulator's drop-out for 0.3 s. The [I/O ledger](io-ledger.md) lists every signal with its model, faults, knob, test and
evidence.

## Cockpit view

`cockpit.html`, next to the bench page, shows the same emulator to a pilot: the Cockpit Controller drawn as the
instrument (its screen, the knob and the MENU, TUNE and MONITOR buttons, all of which work, including a press and
hold, and a TUNE + MONITOR button for the two together), the panel switches (Ignition 1 and 2, a Fuel Pump switch
that runs pump 1 or pump 2, ECU Select 1 / 2), a throttle (black knob) and a prop lever (blue knob), and the engine's
RPM and MAP. Above them, the scenario's checklist: one item at a time, with what follows it in a few words, Next and
Restart. Below, a folded Bench section with the propeller's low-pitch stop setting and a link to the bench page at the
same scenario, and a one-line footer (firmware, not affiliated). The page follows the system's light or dark setting
and fits a 390 px phone screen.

    cockpit.html?scenario=<id>                     set up, then wait for Start
    cockpit.html?scenario=<id>&autostart=1         set up and keep running
    cockpit.html?scenario=<id>&autostart=1&embed=1 only the Controller and the switches, on a transparent
                                                   background, for an iframe in a guide page

The ids are the bench page's (table above). With no id the page starts at the run-up. The checklist is in the same
table (`pilot`: `say`, `watch`, and for an item the pilot cannot do from the cockpit an action button), with the
levers' positions (`quadrant`); `shows` and `guide` are for the pilot guide and are not shown on the page.
`make bench-scenarios` goes through every item on the page with its own controls and checks what each "watch" line
says, from the panel's pixels, the ECUs' link data, the bench's spark counts and the engine model.

### The engine model

On the cockpit page the levers and the sparks set the engine; the bench page keeps its own RPM and MAP sliders. The
model is a simple bench model for an engine with a constant-speed propeller, made up for this page: its numbers are
not aircraft data and are not fitted to any engine. It is one table in the page (`ENGINE`) and drives the bench's
existing RPM, MAP and throttle-sensor inputs; the emulated System32 and the C engine model are unchanged.

* **Throttle.** Sets the RPM the engine reaches with the blades on the low-pitch stop (both ECUs firing): 750 at idle,
  1800 at 25 %, 2520 at 50 %, 2620 at 65 %, 2700 at full, in straight lines between.
* **Prop lever.** Sets the governed RPM, 2000 (back) to 2700 (forward), in a straight line. When the engine could turn
  faster than that, the governor coarsens the blades until it holds the governed RPM. The blade angle follows the
  angle that would hold it as a first-order lag with a time constant of 0.8 s, so a fast throttle push overshoots
  first; while no ECU fires, the blades stay where they are. Below the governed RPM the blades sit on the stop and the
  throttle sets the RPM.
* **MAP.** With the throttle closed: 29.9 inHg stopped, 12.0 at 750 RPM, 10.0 at 2700 RPM. The throttle opens it
  toward 29.0 inHg (full throttle): 45 % of the way at 25 % throttle, 70 % at 50 %, 80 % at 65 %. So MAP rises with
  the throttle and falls a little as RPM rises.
* **Spark.** An ECU counts as firing while its first coil fires at least half as often as the RPM calls for (once per
  revolution), over the last 0.3 s. With one ECU firing the engine makes 85 % of its power, which turns it at the cube
  root of that (about 95 %): at the run-up, about 100 RPM less; above the governed RPM the governor makes up the
  difference. With none it winds down and stops below 200 RPM; it stays stopped until a scenario starts it again.
* **Speed.** RPM moves toward its target with a time constant of 0.4 s up, 0.6 s down, 1.5 s with no spark. When a
  scenario starts the engine, RPM is held for 1.0 s while the ECUs pick up the crank signal.
* **Low-pitch stop** (Bench section). Correct: 2700 RPM at full throttle on the stop. Too fine: 3150 RPM (the
  throttle's RPM above idle scales with it).
* **Not modeled.** A fixed-pitch propeller, the starter, mixture, fuel, temperatures, airspeed and load: none of them
  change the RPM.

On the cockpit page a scenario that sets the engine sets the levers instead: run-up = prop forward, throttle 25 %
(1800 RPM); takeoff = both forward (2700 RPM, 29.0 inHg); cruise and the scenarios built on it = prop at 2400 RPM,
throttle at 24.0 inHg (about 55 %); first-start leaves the engine stopped. `rpm-limit` has the RPM limit at 2750 (see
above), a low-pitch stop set too fine and the run-up levers (about 2040 RPM on that stop). Throttle full, fast: RPM
runs past 2700 before the governor catches up, the ECUs cut spark at 2750, RPM pulses there for about a second, and
the governor then holds 2700 (within 10 RPM from about 3.5 s on).

The Controller's face is drawn from a photo of the real unit, as one table in the page (`FACE`: the screen, the knob,
each button and its label, the card slot and the indicator; the last two are drawing only). MENU, TUNE and MONITOR
drive the same Controller inputs as the bench page's buttons of those names, and the knob's push the same as its PUSH.
The firmware's soft labels (Up, Down, Back, Push to Select) appear on the screen only: in a list MENU moves up, TUNE
down and MONITOR goes back.
