cmake_minimum_required(VERSION 3.16)

# Use the RISC-V cross-compiler toolchain
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/toolchain-app.cmake")
endif()

project(app C)

# --- Which build is this (main/app_version.h.in) -------------------------
#
# The commit and the minute of the build, plus the release out of
# metadata/metadata.json — the file the repository and every installed
# badge already go by. They land in a generated header the app reads
# through main/common/build_id.h, which is what the title screen shows and
# what tools/testrun.py refuses to test a mismatch against.
find_program(GIT_EXE git)
if(GIT_EXE)
    execute_process(COMMAND ${GIT_EXE} describe --always --dirty --abbrev=10
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    OUTPUT_VARIABLE APP_GIT_HASH OUTPUT_STRIP_TRAILING_WHITESPACE
                    ERROR_QUIET)
endif()
if(NOT APP_GIT_HASH)
    set(APP_GIT_HASH "unknown")
endif()
string(TIMESTAMP APP_BUILD_TIME "%Y-%m-%dT%H:%M:%SZ" UTC)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/metadata/metadata.json APP_METADATA)
string(JSON APP_VERSION GET ${APP_METADATA} version)
if(NOT APP_VERSION)
    set(APP_VERSION "")
endif()
message(STATUS "Null Island Space Program ${APP_VERSION} (${APP_GIT_HASH}, ${APP_BUILD_TIME})")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/main/app_version.h.in
               ${CMAKE_BINARY_DIR}/generated/app_version.h @ONLY)

# App source files — add your .c files here
set(APP_SOURCES
    main/main.c
    main/crt0.c
    main/audio/music.c
    main/common/rng.c
    main/common/quat.c
    main/common/vfs_compat.c
    main/save/save.c
    main/ui/settings.c
    main/ui/logo.c
    main/ui/title.c
    main/ui/menu.c
    main/game/input.c
    main/game/scene.c
    main/game/hud.c
    # The sim: orbital mechanics, patched conics, the atmosphere, the
    # surface. Pure arithmetic with no engine and no BSP in it, which is
    # what lets the same files be compiled by the host C compiler for
    # `make check` (tools/hostpurity.sh).
    main/sim/bodies.c
    main/sim/kepler.c
    main/sim/ephemeris.c
    main/sim/patches.c
    main/sim/atmosphere.c
    main/sim/surface.c
    # The rocket: the part registry, the stack maths, the delta-v budget,
    # design files and the heat model. Pure as well -- design.c is the only
    # one that talks to the filesystem, and it does it through se_save.h
    # behind an SM_HOST guard so the host check can read the same code.
    main/rocket/parts.c
    main/rocket/stack.c
    main/rocket/budget.c
    main/rocket/design.c
    main/rocket/heat.c
    # The flight sim: rigid-body dynamics, the SAS controller, warp and the
    # mission loop. Pure like the sim above (tools/hostpurity.sh), which is
    # what lets tools/nispcheck.c fly whole missions on the host.
    main/flight/physics.c
    main/flight/control.c
    main/flight/warp.c
    main/flight/mission.c
    main/flight/archive.c
    # Engineering (Part B): the blueprint sheet, its icons, its notes and the
    # editor that ties them together. 2-D direct-565 drawing and Hershey text
    # only -- no scene, no camera, no PPA (F-13) -- and therefore not part of
    # the host-pure set tools/hostpurity.sh checks.
    main/engineering/sheet.c
    main/engineering/icons.c
    main/engineering/notes.c
    main/engineering/builder.c
)

# Automated device tests (main/testkit/, README "Automated device tests").
# Opt-in: uncomment to build them in, and set where shots go. An app
# without SynthEngine3D adds TESTKIT_NO_ENGINE to the definitions.
list(APPEND APP_SOURCES
    main/testkit/debugcon.c
    main/testkit/devtest.c
    main/testkit/profile.c
    main/testkit/report.c
    main/testkit/screenshot.c
    main/testkit/showtime.c
)
add_compile_definitions(SCREENSHOT_DIR="/sd/nisp/test")
# The engine's binding table defaults to 16 actions; NISP has 26 (Part F2,
# plan 6.3), and se_bindings_init clamps the count silently otherwise.
add_compile_definitions(SE_BINDINGS_MAX=32)

# Include directories — add paths to your headers here
set(APP_INCLUDES
    main
    include
    include/esp_additions/include
    ${CMAKE_BINARY_DIR}/generated
)

# Compile app sources as a relocatable object library
add_library(app_obj OBJECT ${APP_SOURCES})
target_include_directories(app_obj PRIVATE ${APP_INCLUDES})
# -Werror=format: a format string and its arguments disagreeing is not a
# warning here. It is vfprintf dereferencing a pointer nobody passed, which
# is a Load access fault on the badge and nothing at all on a PC build.
target_compile_options(app_obj PRIVATE
    -Wall -Wextra -Wno-unused-parameter -Werror=format
    -Os -fPIC
    -ffunction-sections -fdata-sections -fno-common
    -D__thread=
    -DESP_PLATFORM
    # Per-function stack frames, written next to the objects as .su files and
    # read back by `make stackusage`. The app runs on graceloader's 8 KiB
    # main-task stack, and a stack overflow there is a Guru Meditation on the
    # badge and nothing at all in the emulator (F-45) -- so the number has to
    # be measured where the build actually happens.
    -fstack-usage
)

# ESP-IDF normally force-includes FreeRTOS headers for all components
target_compile_options(app_obj PRIVATE
    "SHELL:-include freertos/FreeRTOS.h"
    "SHELL:-include freertos/semphr.h"
    "SHELL:-include freertos/task.h"
    "SHELL:-include freertos/queue.h"
    "SHELL:-include freertos/event_groups.h"
)

# SynthEngine3D, the 3D engine (https://github.com/nullislandspace/synthengine3D),
# if this app has it. It is not part of the template: an app that wants 3D adds
# it as a git submodule at synthengine3D/ (`make engine`, see README), and this
# block then builds it. Without it every variable below stays empty and the
# link line is exactly the one a plain app has always had, so apps that do not
# use the engine are unaffected by merges from the template.
#
# In this plain-CMake build the engine is a relocatable OBJECT library, so
# add_subdirectory MUST come before the link script's file(GENERATE) for
# $<TARGET_OBJECTS:synthengine3d> to resolve. Linking it to app_obj propagates
# the engine's PUBLIC include directory (synthengine3D/include), which is what
# lets app sources #include "synthengine3d.h"; its object files are folded into
# app.so by the link script below, since the custom command, not CMake, links.
set(ENGINE_OBJS "")
set(ENGINE_DEPS "")
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/synthengine3D/CMakeLists.txt)
    message(STATUS "SynthEngine3D: building synthengine3D/")
    add_subdirectory(synthengine3D)
    target_link_libraries(app_obj PRIVATE synthengine3d)
    set(ENGINE_OBJS "$<JOIN:$<TARGET_OBJECTS:synthengine3d>, >")
    list(APPEND ENGINE_DEPS synthengine3d $<TARGET_OBJECTS:synthengine3d>)
endif()

# Collect object files into a list file, then link via a response file
# This avoids issues with generator expressions in custom commands
set(LINK_SCRIPT ${CMAKE_BINARY_DIR}/link_app.sh)
file(GENERATE OUTPUT ${LINK_SCRIPT} CONTENT
"#!/bin/sh
${CMAKE_C_COMPILER} \
    -march=rv32imafc_zicsr_zifencei_zaamo_zalrsc_xesploop_xespv2p1 -mabi=ilp32f \
    -shared -nostdlib \
    -Wl,--gc-sections \
    -Wl,-Bsymbolic \
    -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/app_version.script \
    -Wl,-T,${CMAKE_CURRENT_SOURCE_DIR}/app.ld \
    -Wl,-Map=${CMAKE_BINARY_DIR}/app.map \
    -L${CMAKE_CURRENT_SOURCE_DIR}/fakelib \
    $<JOIN:$<TARGET_OBJECTS:app_obj>, > ${ENGINE_OBJS} \
    -lall \
    -o ${CMAKE_BINARY_DIR}/app.so
")

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/app.so
    COMMAND sh ${LINK_SCRIPT}
    DEPENDS app_obj
        $<TARGET_OBJECTS:app_obj>
        ${ENGINE_DEPS}
        ${CMAKE_CURRENT_SOURCE_DIR}/app.ld
        ${CMAKE_CURRENT_SOURCE_DIR}/app_version.script
        ${CMAKE_CURRENT_SOURCE_DIR}/fakelib/liball.so
    COMMENT "Linking app.so"
)

add_custom_target(app ALL DEPENDS ${CMAKE_BINARY_DIR}/app.so)

# Print size after build
add_custom_command(TARGET app POST_BUILD
    COMMAND ${CMAKE_SIZE} ${CMAKE_BINARY_DIR}/app.so
    COMMENT "app.so size:"
)
