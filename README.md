# Tanmatsu graceloader template app project

This template project shows how to build an app for Tanmatsu using [Graceloader](https://github.com/nullislandspace/tanmatsu-graceloader)

## Null Island Space Program

*To nowhere, and beyond!* — a ship, an island at 0°N 0°E, and not much else.
This repository is the template turned into that game: SynthEngine3D as a
submodule, the flight, the menus, saves, settings, the livestream, and the
device test kit wired to a scripted flight path.

The island is one small fixed mesh submitted in full every frame. There is no
chunk streaming here, whatever SynthMiner does.

The design and the step-by-step status of the game live in
[`claudeplans/project.md`](claudeplans/project.md): the body table, the rocket
blueprint model, the flight physics, the map and maneuver planner, and the
plan with its progress table. It is updated as part of the work, not after it.

### Where the files live

| Path | What |
|---|---|
| `/sd/apps/at.cavac.nisp/` | the install: `app.so`, `metadata.json`, `nullislandlogo_ball.png`, `music/` |
| `/sd/nisp/saves/` | the three save slots |
| `/sd/nisp/settings.txt` | settings and key bindings |
| `/sd/nisp/test/` | test-kit screenshots (`SCREENSHOT_DIR`) |

Player data is deliberately outside the install directory — the launcher owns
that and may empty it on an update — so copying `/sd/nisp` is the whole backup
(`main/common/paths.h`).

### The logo

`nullislandlogo_ball.png` is data, not code. `make install` uploads it beside
`app.so` and `metadata/metadata.json` lists it as an asset, so the artwork can
be redrawn without rebuilding anything. `main/ui/logo.c` decodes it once at
boot, scales it down to the size the title draws it at (`LOGO_DRAW_H`), and
throws the 3.5 MB original away: every frame after that is a blit, not a
resample. What it keeps is packed for the blit -- the framebuffer's own RGB565
halfwords laid out column by column, each column trimmed to the rows the
artwork actually touches, with its fully opaque middle recorded as a run.
Drawing it is then 341 contiguous sweeps of the framebuffer: stores for the
middle, a blend for the soft edge, nothing at all for the transparent corners.
Alpha survives the packing because the ball is round and the framebuffer has no
alpha to keep a soft edge with; on a display that is not RGB565 / `PAX_O_ROT_CW`
the layer stays ARGB8888 and `logo_draw()` falls back to `pax_draw_image()`. If
the file is missing the title falls back to the wordmark alone.

The title's artwork is timed as its own profiler phase (`logo`), because
blitting 341x340 pixels is an order of magnitude more work than the few hundred
pixels of text the rest of the overlay is.

`se_splash()` — the SynthEngine3D logo, the same way SynthMiner shows it — is
the first thing `on_init()` does, before the scene, the bindings, the saves or
the title exist.

### Music and licenses

The menu plays a recording rather than a generator: `brittle_rille.mp3`,
looped, out of `music/menu/` beside `app.so`. The engine's MP3 source plays
every `.mp3` in the directory it is handed and cannot be pointed at one file,
which is why the menu's track has a directory of its own while the rest of the
shipped music sits in `music/` until something wants it (`main/audio/music.c`).
A card with no music falls back to the procedural generator rather than to
silence.

All three tracks are Kevin MacLeod (incompetech.com) under Creative Commons
Attribution 4.0. Each `.mp3` has a `.txt` beside it carrying the license, and
that file is uploaded with the track; Settings → Audio shows the attribution
line for whatever is playing, and **Next track** moves to the next one in the
playlist.

### Menu structure

```
title bar   a strip along the bottom: Play / Settings / Quit
  Play      the three slots
    a world   Play / Rename / Delete
    empty     New world: name, seed, Random seed, Create
  Settings  Controls / Graphics / Audio / Display / Livestream
pause       Resume / Save / Settings / Save and quit / Quit
```

The title bar is a bar rather than a panel, so it does not cover the logo and
the wordmark; `>` marks the chosen item and the arrow keys move along it. Every
other screen is a panel drawn by the engine's list menu. The livestream is a
setting rather than a title-bar item: it takes the USB-C port away from the
debug console, so it belongs where a person has to open the panel to find it.

The menu decides, `main/main.c` acts: anything that changes which world is
open, whether the ship is flying, or whether the USB-C port is a video link
comes back from `menu_update()` as a `MENU_CMD_*`. Settings and bindings are
the menu's own business and never reach `main.c`.

### Controls

All of them remappable under Settings → Controls (`main/game/input.c`):
`WASD` thrust, `Space`/`Left Shift` up and down, `Right Shift` boost, cursor
keys look, `Esc` pause, `F2` save, `Backspace` position/diagnostics, `0`
screenshot.

### Testing it on a badge

```sh
make push                                          # app.so only -- the fast path
make install                                       # app.so + the logo + the music
make cycle TEST="perf scene=space secs=20"         # frame rate, phase split, primitive counts
make cycle TEST="shots scene=title ms=0,2500"      # render exact instants, save PNGs + hashes
make testrefs   TEST="shots scene=space secs=20"   # store those hashes
make testcompare TEST="shots scene=space secs=20"  # regression test against them
```

`install` sends every file every time — there is no stamp for "already sent",
because a cache of what this checkout last uploaded is a cache about the card it
uploaded to. `push` sends `app.so` and nothing else, which is what iterating on
code needs; run `install` when the artwork or the music actually changed.

`scene=` takes `title`, `space`, or any menu name (`worlds`, `settings`,
`controls`, `pause`, ...). A flight steered by a stick cannot be a pure
function of the clock, so the kit flies a scripted path instead (`bench_pose`)
— which makes the same run a regression test of the renderer, the HUD and the
menus. See "Automated device tests" below.

## 3D: SynthEngine3D

[SynthEngine3D](https://github.com/nullislandspace/synthengine3D) is the 3D engine for
graceloader apps: a software rasteriser (z-buffer and raycast), PPA compositing, meshes,
textures, lighting, audio and UI helpers, with its own `se_run()` main loop.

The engine is **not** shipped with the template, so apps that do not want it are not
carrying it around. What the template does ship is the build wiring, which sits idle until
an app adds the engine. In a new app that wants 3D:

```sh
make engine     # git submodule add -b main git@github.com:nullislandspace/synthengine3D.git synthengine3D
git add .gitmodules synthengine3D && git commit -m "Add SynthEngine3D"
```

`CMakeLists.txt` picks it up by itself: when `synthengine3D/CMakeLists.txt` exists it builds
the engine, propagates its include directory (so app sources can `#include "synthengine3d.h"`)
and folds its objects into `app.so`. When it does not, the link line is exactly the plain
one, which is why every app can keep this template as `upstream` whether it uses 3D or not.

Two things follow from it being a submodule:

* clone such an app with `git clone --recursive`, or run `git submodule update --init` in it;
* `git submodule update --remote synthengine3D` moves it to the newest engine, and an app
  that wants a fixed version pins it (`ENGINE_REF=V2.0 make engine`, or check out the tag
  inside `synthengine3D/` and commit the new pointer).

Engine settings (list caps and the like) are compile definitions that must reach the
`synthengine3d` target, so set them with `add_compile_definitions()` **before**
`add_subdirectory(synthengine3D)` — see the engine's `docs/configuration.md`.

## Automated device tests

`main/testkit/` is a ready-made test loop for an app on real hardware. The host
sends a command over the debug console, the app runs it inside its own frame
loop, reports machine-readable records, and returns to the launcher by itself —
so `make cycle` builds, installs, runs, tests and comes back with a verdict
without anyone touching the badge.

```sh
make cycle       TEST="perf  scene=title secs=20"       # frame rate, phase split, primitive counts
make cycle       TEST="shots scene=title ms=0,1500,4000" # render exact instants, save PNGs + hashes
make testrefs    TEST="shots scene=title ms=0,1500,4000" # store those hashes as the references
make testcompare TEST="shots scene=title ms=0,1500,4000" # compare against them: a regression test
make recover                                             # after a crash or a hang
```

Results land in `results/<UTC>-<test>-<scene>/` as `console.log` + `result.json`;
references live in `tests/refs/manifest.json`. Exit codes: 0 ok, 1 link, 2 crash,
3 the test reported bad, 4 an image mismatch, 5 usage.

### What it is

| File | What |
|---|---|
| `testkit/debugcon.*` | The console listener: `PING`, `RUN <test> k=v`, `EXIT`, `BADGELINK`, read through the USB-serial/JTAG **driver** (graceloader 2.4.0+ exports it). Emits a `READY` record every 2 s while idle, so the host can find the app without sending anything. |
| `testkit/report.*` | The record format: `@@SR-<KIND>@@ <json> @@<crc32>@@`, one line, CRC'd so a line another task interleaved into it is dropped rather than believed. |
| `testkit/devtest.*` | The two tests (`perf`, `shots`) and the runner that drives them. |
| `testkit/profile.*` | Per-phase frame timing (`prof_begin`/`prof_end`), reported in each `PERF` record. The phase names are yours. |
| `testkit/screenshot.*` | Framebuffer → PNG on the SD card, with a deflate *stored* stream so it needs 64 KB of PSRAM rather than a ~130 KB compressor in scarce internal RAM. |
| `testkit/showtime.*` | The clock everything hangs off: real time, or fixed steps of 1/fps, or set outright. |
| `tools/testrun.py` | The host side: connect, identify, refuse a stale build, run, collect, compare, write results. |
| `tools/recover.py` | Get a wedged badge back. |

### Wiring it into an app

1. Add `main/testkit/*.c` to `APP_SOURCES`, and `main` to `APP_INCLUDES` (it is
   there already). Set the app's paths and name while you are there:

   ```cmake
   add_compile_definitions(SCREENSHOT_DIR="/sd/myapp")
   # REPORT_PREFIX="SR" by default; change it only if two apps' logs mix,
   # and pass the same to testrun.py with --prefix.
   ```

2. Tell the kit how to address your content — one struct, five functions:

   ```c
   static devtest_content_t const CONTENT = {
       .select    = level_select,     // play this one from its start; false if unknown
       .duration  = level_duration,   // seconds, <= 0 for endless
       .started   = level_started,    // show time at which it began
       .name      = level_name,       // what is selected now
       .shot_name = level_section,    // sub-section for per-shot stats, or ""
   };
   static devtest_config_t const TEST = {
       .app = "tld.username.myapp", .shot_dir = "/sd/myapp/test", .content = &CONTENT,
   };
   ```

3. Call it from the frame loop:

   ```c
   on_init:    devtest_start(&TEST);
   on_update:  showtime_frame(); devtest_update(); /* then advance your own content */
   on_render:  /* draw the frame */ devtest_after_render(fb, rast_us);
   per second: devtest_period(fps, frame_ms);
   ```

An app without SynthEngine3D compiles the kit with `TESTKIT_NO_ENGINE`: it then
reports timings and heap, and the primitive counts read zero.

### The one precondition

The `shots` test renders *chosen instants*: it sets the clock instead of running
it. That is only meaningful if what you draw is a **pure function of
`showtime_now()`** — same t, same picture. Get that right and a stored hash is a
real regression test, a host-side checker can replay the same instant (see the
engine's `docs/testing.md`), and a video export can render far slower than real
time without changing a frame. An app that accumulates per-frame `dt`, or draws
from unseeded randomness, can still use `perf`, but its shot hashes will wobble
and the references mean nothing.

## License

This software is under the [MIT license](https://opensource.org/license/mit). The MIT license allows others to build upon your work without restrictions while also making sure you retain your attribution.

(C) 2026 Rene Schickbauer
