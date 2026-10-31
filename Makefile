PORT ?= /dev/ttyACM0
# BadgeLink transport. Empty (the default) means raw USB through libusb, the
# only thing the P4 exposes in BadgeLink mode (16d0:0f9a, vendor interface
# ff:00:00, no CDC-ACM, so there is no /dev/ttyACM* for it). host:port
# selects a TCP proxy, anything else a serial device.
BADGELINKPORT ?=

SHELL := /usr/bin/env bash

# App installation settings
# The slug the launcher knows this app by, and where it lives on the card.
# /sd/apps/ rather than /int/apps/: the game's own data is /sd/nisp, and a
# player who wants to back everything up copies one card, not two
# partitions (main/common/paths.h).
APP_SLUG_NAME ?= at.cavac.nisp
APP_INSTALL_BASE_PATH ?= /sd/apps/
APP_INSTALL_PATH = $(APP_INSTALL_BASE_PATH)$(APP_SLUG_NAME)

# The shipped music (music/). Everything in that directory goes onto the
# card, EXCEPT the tracks named in the *_STEMS lists, which go into the
# directory of the context that plays them: se_mp3 plays every .mp3 in the
# directory it is given and has no way to be pointed at one file, so a
# track that belongs to one place lives in a directory of its own
# (main/audio/music.h). The .txt beside each track is its license and
# travels with it. metadata.json lists the same files by hand -- add a
# track to both, or the app repository install comes up without it.
MUSIC_ALL := $(notdir $(wildcard music/*))
MUSIC_MENU_STEMS        := brittle_rille
MUSIC_ENGINEERING_STEMS := sneaky_adventure
MUSIC_SPACE_STEMS       := impact_lento
MUSIC_MENU := $(foreach f,$(MUSIC_ALL),$(if $(filter $(MUSIC_MENU_STEMS),$(basename $(f))),$(f)))
MUSIC_ENGINEERING := $(foreach f,$(MUSIC_ALL),$(if $(filter $(MUSIC_ENGINEERING_STEMS),$(basename $(f))),$(f)))
MUSIC_SPACE := $(foreach f,$(MUSIC_ALL),$(if $(filter $(MUSIC_SPACE_STEMS),$(basename $(f))),$(f)))
MUSIC_POOL := $(filter-out $(MUSIC_MENU) $(MUSIC_ENGINEERING) $(MUSIC_SPACE),$(MUSIC_ALL))

# ESP-IDF tools path (needed for the RISC-V cross-compiler)
IDF_PATH ?= $(shell cat .IDF_PATH 2>/dev/null || echo `pwd`/esp-idf)
IDF_TOOLS_PATH ?= $(shell cat .IDF_TOOLS_PATH 2>/dev/null || echo `pwd`/esp-idf-tools)
IDF_VERSION ?= v6.0.2
IDF_GITHUB_ASSETS ?= dl.espressif.com/github_assets

# The ESP-IDF environment script. Only mode_badgelink needs it (for pyserial
# out of the IDF python env -- the app build itself just needs the toolchain
# via IDF_TOOLS_PATH). Honours an IDF_SOURCE already exported by the shell.
IDF_SOURCE ?= $(shell cat .IDF_PATH 2>/dev/null && echo '$(IDF_PATH)/export.sh' || test -d `pwd`/esp-idf && echo '$(IDF_PATH)/export.sh' || echo '$(HOME)/.espressif/tools/activate_idf_$(IDF_VERSION).sh')

export IDF_TOOLS_PATH
export IDF_GITHUB_ASSETS

BUILD ?= build

MAKEFLAGS += --silent

####

.PHONY: all
all: build

.PHONY: build
build: check
	@echo "=== Building app.so ==="
	mkdir -p $(BUILD)
	cd $(BUILD) && cmake .. && make
	@echo "=== Build complete: $(BUILD)/app.so ==="

# ---------------------------------------------------------------------
# Host checks: no badge, seconds to run, and `build` depends on them so a
# broken invariant stops the build that broke it (claudeplans/project.md,
# Part H).
#
# The orbital maths and the rocket arithmetic are the two parts of this
# game that can be wrong without anything looking wrong -- a sign error in
# the static margin or a mean motion off by 2*pi produces a rocket that
# misses the orbit the map says it reached. Those are checked here, on this
# PC, against numbers a player can look up, before app.so is even linked.
#
# The engine's compile-time settings come out of CMakeLists.txt rather than
# being written down twice: a host check running against different caps than
# the app builds with is worse than no check.
# ---------------------------------------------------------------------
HOSTCC      ?= cc
ENGINE_DEFS := $(shell sed -n 's/^add_compile_definitions(\(SE_[A-Z_]*=[0-9]*\))/-D\1/p' CMakeLists.txt)
HOSTCFLAGS  := -O1 -Wall -Wextra -Werror=implicit-function-declaration -Werror=format \
               -DSM_HOST -Imain -Itools -Isynthengine3D/include \
               -Isynthengine3D/src/internal $(ENGINE_DEFS)

# The pure set: everything Part H compiles against, and nothing else.
PURE_SRCS := $(wildcard main/sim/*.c) $(wildcard main/rocket/*.c) $(wildcard main/flight/*.c) \
             main/common/rng.c main/common/quat.c

.PHONY: check
check: hostpurity nispcheck

# The pure set really is pure: no engine, no BSP, no FreeRTOS.
.PHONY: hostpurity
hostpurity:
	./tools/hostpurity.sh

# Orbital mechanics, patched conics, the atmosphere, the rocket equation,
# heat. Every number the game is built on.
.PHONY: nispcheck
nispcheck:
	mkdir -p $(BUILD)/host
	$(HOSTCC) $(HOSTCFLAGS) tools/nispcheck.c $(PURE_SRCS) -lm -o $(BUILD)/host/nispcheck
	$(BUILD)/host/nispcheck

# SynthEngine3D, the 3D engine: not part of the template, added per app as a
# git submodule (CMakeLists.txt builds it when synthengine3D/ is there, and is
# untouched otherwise). Run this once in a new app that wants 3D; afterwards a
# fresh clone needs `git clone --recursive` or `git submodule update --init`.
ENGINE_URL ?= git@github.com:nullislandspace/synthengine3D.git
ENGINE_REF ?= main

.PHONY: engine
engine:
	if test -d synthengine3D; then \
	  echo "synthengine3D/ is already there -- 'git submodule update --remote synthengine3D' updates it"; exit 1; \
	fi
	git submodule add -b $(ENGINE_REF) $(ENGINE_URL) synthengine3D
	git submodule update --init --recursive synthengine3D
	@echo "=== SynthEngine3D added. Commit .gitmodules and synthengine3D, then #include \"synthengine3d.h\" ==="

# Test automation (main/testkit/devtest.h, tools/testrun.py). Opt-in:
# the app has to compile main/testkit/*.c and call devtest_start(). See
# README, "Automated device tests".
#
#   make testrun TEST="perf scene=title secs=20"      the app must be running
#   make cycle   TEST="shots scene=title ms=0,2500"   build, install, run, test
#   make testrefs    TEST="shots ..."                 cycle, then store the shot hashes as references
#   make testcompare TEST="shots ..."                 cycle, then compare against them
#   make recover                                      after a crash or a hang
#
# The app runs the test and returns to the launcher by itself, so the
# whole cycle is hands-free. Shot images stay on the SD card; the hashes
# come back over the console, which is what a regression check compares.
# TESTFLAGS=--fetch downloads the images too (slow).
TEST ?=
TESTFLAGS ?=

.PHONY: testrun cycle testrefs testcompare recover
testrun:
	source "$(IDF_SOURCE)" >/dev/null && \
	python3 -u tools/testrun.py --port "$(PORT)" --badgelink-conn "$(BADGELINK_CONN)" $(TESTFLAGS) -- $(TEST)

cycle: build install run
	$(MAKE) testrun

testrefs:
	$(MAKE) cycle TESTFLAGS="--capture-refs"

testcompare:
	$(MAKE) cycle TESTFLAGS="--compare"

recover:
	source "$(IDF_SOURCE)" >/dev/null && python3 tools/recover.py --port "$(PORT)"

# Badgelink
.PHONY: badgelink
badgelink:
	rm -rf badgelink
	#git clone https://github.com/badgeteam/esp32-component-badgelink.git badgelink
	git clone https:///github.com/nullislandspace/esp32-component-badgelink.git badgelink
	cd badgelink/tools; ./install.sh

# Determine badgelink connection argument: --tcp for host:port, --port for serial
# devices, nothing at all for raw USB (badgelink.py then uses libusb)
BADGELINK_CONN := $(if $(BADGELINKPORT),$(if $(findstring :,$(BADGELINKPORT)),--tcp $(BADGELINKPORT),--port $(BADGELINKPORT)),)

.PHONY: install
# INSTALL PUTS EVERY FILE ON THE DEVICE, EVERY TIME: the app, the artwork,
# the music and its licenses. There is no stamp for "already sent" -- a
# cache keyed on what this checkout last uploaded is a cache about the card
# it was uploaded to, and when it is wrong the app comes up with no artwork
# and no music for a reason nothing on screen can explain. `make push` is
# the fast path for iterating on code.

# $(1) = local path, $(2) = remote path
define UPLOAD
	@echo "  upload $(1)"
	@( cd badgelink/tools && ./badgelink.sh $(BADGELINK_CONN) fs upload $(2) ../../$(1) ) || exit 1
endef

# The fast path: app.so and nothing else. A code change does not need the
# 9 MB of music re-sent, and `install` is for when the assets actually
# changed.
.PHONY: push
push: build
	@echo "=== Pushing app.so ==="
	cd badgelink/tools; ./badgelink.sh $(BADGELINK_CONN) fs upload $(APP_INSTALL_PATH)/app.so ../../$(BUILD)/app.so
	@echo "=== Pushed (assets untouched -- 'make install' for those) ==="

install: build
	@echo "=== Installing to device ==="
	cd badgelink/tools; ./badgelink.sh $(BADGELINK_CONN) fs mkdir $(APP_INSTALL_PATH) || true
	$(call UPLOAD,metadata/metadata.json,$(APP_INSTALL_PATH)/metadata.json)
	$(call UPLOAD,metadata/icon16.png,$(APP_INSTALL_PATH)/icon16.png)
	$(call UPLOAD,metadata/icon32.png,$(APP_INSTALL_PATH)/icon32.png)
	$(call UPLOAD,metadata/icon64.png,$(APP_INSTALL_PATH)/icon64.png)
	# The title screen's logo (ui/logo.h). Data, not a blob in the binary:
	# it goes onto the card beside app.so, where it can be replaced without
	# rebuilding anything. Listed in metadata.json's assets, so an install
	# from the app repository brings it along too.
	$(call UPLOAD,nullislandlogo_ball.png,$(APP_INSTALL_PATH)/nullislandlogo_ball.png)
	$(call UPLOAD,$(BUILD)/app.so,$(APP_INSTALL_PATH)/app.so)
	@echo "Music: $(words $(MUSIC_ALL)) file(s) -- $(words $(MUSIC_MENU)) in music/menu, $(words $(MUSIC_ENGINEERING)) in music/engineering, $(words $(MUSIC_SPACE)) in music/space, $(words $(MUSIC_POOL)) in music"
	cd badgelink/tools; ./badgelink.sh $(BADGELINK_CONN) fs mkdir $(APP_INSTALL_PATH)/music >/dev/null 2>&1 || true
	cd badgelink/tools; ./badgelink.sh $(BADGELINK_CONN) fs mkdir $(APP_INSTALL_PATH)/music/menu >/dev/null 2>&1 || true
	cd badgelink/tools; ./badgelink.sh $(BADGELINK_CONN) fs mkdir $(APP_INSTALL_PATH)/music/engineering >/dev/null 2>&1 || true
	cd badgelink/tools; ./badgelink.sh $(BADGELINK_CONN) fs mkdir $(APP_INSTALL_PATH)/music/space >/dev/null 2>&1 || true
	@for m in $(MUSIC_MENU); do \
	  ( cd badgelink/tools && ./badgelink.sh $(BADGELINK_CONN) fs upload $(APP_INSTALL_PATH)/music/menu/$$m ../../music/$$m >/dev/null ) || exit 1; \
	done
	@for m in $(MUSIC_ENGINEERING); do \
	  ( cd badgelink/tools && ./badgelink.sh $(BADGELINK_CONN) fs upload $(APP_INSTALL_PATH)/music/engineering/$$m ../../music/$$m >/dev/null ) || exit 1; \
	done
	@for m in $(MUSIC_SPACE); do \
	  ( cd badgelink/tools && ./badgelink.sh $(BADGELINK_CONN) fs upload $(APP_INSTALL_PATH)/music/space/$$m ../../music/$$m >/dev/null ) || exit 1; \
	done
	@for m in $(MUSIC_POOL); do \
	  ( cd badgelink/tools && ./badgelink.sh $(BADGELINK_CONN) fs upload $(APP_INSTALL_PATH)/music/$$m ../../music/$$m >/dev/null ) || exit 1; \
	done; echo "  $(words $(MUSIC_ALL)) music file(s) sent"
	@echo "=== Installation complete ==="

GRACELOADER_SLUG ?= at.cavac.graceloader

.PHONY: run
run:
	cd badgelink/tools; ./badgelink.sh $(BADGELINK_CONN) start $(GRACELOADER_SLUG) $(APP_INSTALL_PATH)/app.so

# The virtual graceloader (../virtual-tanmatsu-graceloader): this same
# build/app.so, run on this PC by an RV32 interpreter with an SDL window for
# the screen and the keyboard for the buttons. `nisp` is one of the apps that
# Makefile knows, and its `nisp-install` stages the assets metadata.json lists
# -- logo, music, licenses -- onto a host directory the emulator exports as
# /sd, so the app sees the same install path here as it does on a card.
#
#   make emu-run                 build, stage, run until the window is closed
#   make emu-run EMU_SECS=15     ... and quit by itself after 15 s
#   make emu-build               build the emulator only
#   make emu-test                run the test manifest, tools/emutests.txt
#
# Good for: menus, screens, layout, saves -- anything where the question is
# "does this look right", answered without anybody at the badge.
# Not good for: frame rate. The interpreter is not the P4, so its fps is not
# evidence about the render budget; those numbers come from a device.
#
# What still needs a badge: frame rate, and anything about real hardware --
# the interpreter is maybe a third of the P4's speed, so its fps is a shape
# (which phase, how many primitives) and not a number.
EMU_DIR ?= ../virtual-tanmatsu-graceloader
EMU_BIN ?= $(EMU_DIR)/build/graceloader-emulator
EMU_INT ?= $(EMU_DIR)/int
EMU_SD ?= $(EMU_DIR)/sd
EMU_CARD ?= $(EMU_DIR)/card
# Seconds to run before quitting by itself; 0 runs until the window closes.
EMU_SECS ?= 0
EMU_GUEST_PATH = $(APP_INSTALL_PATH)/app.so
EMU_ARGS = --int $(EMU_INT) --sd $(EMU_SD) --card $(EMU_CARD) $(EMU_GUEST_PATH)

.PHONY: emu emu-build emu-install emu-run
emu-build:
	$(MAKE) -C $(EMU_DIR) build

# Staging is the emulator's own target, driven off this repo's metadata.json --
# one manifest, not two lists that can disagree.
emu-install: build
	$(MAKE) -C $(EMU_DIR) nisp-install

emu-run: emu-install
	@if [ "$(EMU_SECS)" -gt 0 ]; then \
	  timeout "$(EMU_SECS)" "$(EMU_BIN)" $(EMU_ARGS) || test $$? -eq 124; \
	else \
	  "$(EMU_BIN)" $(EMU_ARGS); \
	fi

emu: emu-run

# The same test kit the badge runs (main/testkit/devtest.h), driven through
# the emulator's console: ../virtual-tanmatsu-graceloader backs USB-serial/JTAG
# with stdin, so a command piped in is a command the app receives, its
# @@SR-*@@ records come out on stdout, and it exits by itself when the test
# ends. No badge, no serial port, nobody at a device.
#
#   make emu-test
#
# Always those three words, never any arguments: what gets run lives in
# tools/emutests.txt, so a run is reproducible, reviewable in the repo, and
# identical every time instead of re-typed. The app exits to the launcher
# when a test ends, so each line of the manifest gets its own launch.
# Everything the app printed is kept in results/emulator/latest.log.
#
# The records are the same records tools/testrun.py parses off a badge. Frame
# rates are not comparable to a badge's (an RV32 interpreter, not a P4); what
# carries over is the split between phases and the primitive counts.
EMU_TEST_FILE ?= tools/emutests.txt
EMU_TEST_LOG  ?= results/emulator/latest.log
# Per test. The app quits by itself when a test ends, so this only fires if
# one hangs.
EMU_TEST_SECS ?= 180

# The manifest runs on an empty card (F-39). Some of its screens are drawn
# from whatever the card holds: the Spaceport hub names the slot and counts
# the blueprints, Designs IS the list, and the settings screens show the
# saved settings. Left alone, a run reproduces the last run's card rather
# than a known state, and a screenshot taken while playing quietly changes
# the reference set. So the player data goes away before the manifest, and
# only for emu-test -- `make emu-run` keeps whatever is on the card, because
# there the point of the card is that you saved something on it.
EMU_TEST_DATA = saves designs settings.txt

.PHONY: emu-test
emu-test: emu-install
	@if [ -d "$(EMU_SD)/nisp" ]; then \
	  for f in $(EMU_TEST_DATA); do rm -rf "$(EMU_SD)/nisp/$$f"; done; \
	  echo "--- cleared $(EMU_SD)/nisp of: $(EMU_TEST_DATA)"; \
	fi
	@mkdir -p $(dir $(EMU_TEST_LOG))
	@: > $(EMU_TEST_LOG)
	@n=0; fail=0; \
	 while IFS= read -r line; do \
	   case "$$line" in ''|'#'*) continue;; esac; \
	   n=$$((n+1)); \
	   echo "--- $$line"; \
	   out=$$(mktemp); \
	   printf '%s\n' "$$line" | timeout $(EMU_TEST_SECS) "$(EMU_BIN)" $(EMU_ARGS) > "$$out" 2>&1; \
	   rc=$$?; \
	   cat "$$out" >> "$(EMU_TEST_LOG)"; \
	   grep -E '@@SR-[A-Z]+@@' "$$out" | sed 's/^/    /'; \
	   rm -f "$$out"; \
	   if [ $$rc -ne 0 ] && [ $$rc -ne 124 ]; then echo "    exit $$rc"; fail=1; fi; \
	 done < $(EMU_TEST_FILE); \
	 echo "--- $$n test(s) -> $(EMU_TEST_LOG)"; \
	 test $$fail -eq 0

# USB mode switching
#
# The device's USB peripheral is in one of two modes: BadgeLink (USB_DEVICE),
# which is what install / run need, or flash-and-monitor (USB_DEBUG). An
# `install` that fails to reach the device usually means the launcher left it
# in debug mode -- `make mode_badgelink` puts it back.
#
# Ask the firmware (in USB_DEBUG mode) to switch its USB into BadgeLink mode
# by sending the token "BADGELINK\n" on the USB-serial/JTAG peripheral. The
# launcher listens for it (see ../tanmatsu-launcher/main/usb_device.c), so
# this only works against firmware that implements the listener.
#
# PORT accepts either a local device path (e.g. /dev/ttyACM0) or an rfc2217://
# URL pointing at ../tanmatsu-badgefs/rfc2217proxy when the device is
# forwarded over the network.
.PHONY: mode_badgelink
mode_badgelink:
	source "$(IDF_SOURCE)" >/dev/null && \
	python3 -c "import serial, sys; s=serial.serial_for_url('$(PORT)', timeout=1); s.write(b'BADGELINK\n'); s.flush(); sys.stdout.write(s.read(128).decode(errors='replace')); s.close()"

# The other direction: ask the firmware (in BadgeLink mode) to switch its USB
# back to flash/monitor mode, through BadgeLink's own `mode` command. Uses the
# badgelink checkout that `make badgelink` creates, and the same BADGELINK_CONN
# as install / run, so it follows a networked device too.
BADGELINK_SH := badgelink/tools/badgelink.sh

.PHONY: mode_debug
mode_debug:
	if [ ! -x "$(BADGELINK_SH)" ]; then \
	  echo "$(BADGELINK_SH) not found -- run 'make badgelink' first"; \
	  exit 1; \
	fi; \
	echo "Using $(BADGELINK_SH)"; \
	"$(BADGELINK_SH)" $(BADGELINK_CONN) mode debug

APP_REPO_PATH ?= ../tanmatsu-app-repository/$(APP_SLUG_NAME)

.PHONY: apprepo
apprepo: build
	@echo "=== Updating app repository ==="
	mkdir -p $(APP_REPO_PATH)/music/menu $(APP_REPO_PATH)/music/engineering $(APP_REPO_PATH)/music/space
	cp metadata/metadata.json $(APP_REPO_PATH)/metadata.json
	cp metadata/icon16.png $(APP_REPO_PATH)/icon16.png
	cp metadata/icon32.png $(APP_REPO_PATH)/icon32.png
	cp metadata/icon64.png $(APP_REPO_PATH)/icon64.png
	cp $(BUILD)/app.so $(APP_REPO_PATH)/app.so
	# Everything metadata.json promises. It lists the logo and the music,
	# so a repository that carries only app.so installs an app with no
	# artwork and no music.
	cp nullislandlogo_ball.png $(APP_REPO_PATH)/nullislandlogo_ball.png
	@for m in $(MUSIC_MENU); do cp music/$$m $(APP_REPO_PATH)/music/menu/$$m || exit 1; done
	@for m in $(MUSIC_ENGINEERING); do cp music/$$m $(APP_REPO_PATH)/music/engineering/$$m || exit 1; done
	@for m in $(MUSIC_SPACE); do cp music/$$m $(APP_REPO_PATH)/music/space/$$m || exit 1; done
	@for m in $(MUSIC_POOL); do cp music/$$m $(APP_REPO_PATH)/music/$$m || exit 1; done
	@echo "  $(words $(MUSIC_ALL)) music file(s) copied"
	@echo "=== App repository updated at $(APP_REPO_PATH) ==="

# Preparation

.PHONY: prepare
prepare: sdk

.PHONY: sdk
sdk:
	if test -d "$(IDF_PATH)"; then echo -e "ESP-IDF target folder exists!\r\nPlease remove the folder or un-set the environment variable."; exit 1; fi
	if test -d "$(IDF_TOOLS_PATH)"; then echo -e "ESP-IDF tools target folder exists!\r\nPlease remove the folder or un-set the environment variable."; exit 1; fi
	git clone --recursive --branch "$(IDF_VERSION)" https://github.com/espressif/esp-idf.git "$(IDF_PATH)" --depth=1 --shallow-submodules
	cd "$(IDF_PATH)"; git submodule update --init --recursive
	cd "$(IDF_PATH)"; bash install.sh all

.PHONY: reinstallsdk
reinstallsdk:
	cd "$(IDF_PATH)"; bash install.sh all

.PHONY: removesdk
removesdk:
	rm -rf "$(IDF_PATH)"
	rm -rf "$(IDF_TOOLS_PATH)"

.PHONY: refreshsdk
refreshsdk: removesdk sdk

# Verification

.PHONY: verify
verify: build
	@echo "=== Verifying symbols ==="
	@MISSING=$$(comm -23 \
	  <(nm -D --undefined-only $(BUILD)/app.so | awk '{print $$2}' | sort -u) \
	  <(nm -D --defined-only fakelib/liball.so | awk '{print $$3}' | sort -u)); \
	if [ -n "$$MISSING" ]; then \
	  echo "ERROR: app.so requires symbols not in fakelib:"; \
	  echo "$$MISSING"; \
	  exit 1; \
	else \
	  echo "All symbols satisfied."; \
	fi

# Cleaning

.PHONY: clean
clean:
	rm -rf $(BUILD)

.PHONY: fullclean
fullclean: clean

# Formatting

.PHONY: format
format:
	find main/ -iname '*.h' -o -iname '*.c' -o -iname '*.cpp' | xargs clang-format -i
