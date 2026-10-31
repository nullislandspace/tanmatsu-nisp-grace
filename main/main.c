// =====================================================================
//  Null Island Space Program  --  the program
// ---------------------------------------------------------------------
//  The engine owns the frame loop (se_run): the device bootstrap, the
//  input pump, the device-global keys, the page flip. What lives here is
//  the state machine and the ship:
//
//    title    the island drifting past, the wordmark over it, and the
//             title bar's Play / Settings / Livestream / Quit over that
//    play     the ship and the HUD
//    paused   the ship frozen under the pause menu
//
//  THE MENU DECIDES, THIS FILE ACTS (ui/menu.h). Opening a world,
//  starting the stream, leaving to the launcher, resuming after a pause:
//  those have an order and a frame they have to happen on, so they come
//  back from menu_update() as commands. Settings and bindings are the
//  menu's own business and never appear here.
//
//  THE CLOCK IS THE TEST KIT'S (testkit/showtime.h). Everything drawn
//  reads showtime_now() rather than an accumulated dt, so a `shots` run
//  can render one chosen instant and get the same pixels every time. A
//  flight steered by a stick cannot be that, so a test flies a SCRIPTED
//  path over the island instead (bench_pose) -- which makes the same
//  test a regression test of the renderer, the HUD and the menus.
//
//  NO CHUNK STREAMING. The island is one small fixed mesh, submitted in
//  full every frame (game/scene.h); there is nothing to page in, and no
//  loader thread.
// =====================================================================

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bsp/device.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "pax_gfx.h"
#include "se_audio.h"
#include "se_config.h"
#include "se_ppa.h"
#include "se_run.h"
#include "se_scene.h"
#include "se_splash.h"
#include "se_stream.h"
#include "se_version.h"
#include "synthengine3d.h"

#include "audio/music.h"
#include "common/build_id.h"
#include "game/hud.h"
#include "game/input.h"
#include "game/scene.h"
#include "save/save.h"
#include "testkit/devtest.h"
#include "testkit/profile.h"
#include "testkit/screenshot.h"
#include "testkit/showtime.h"
#include "ui/logo.h"
#include "ui/menu.h"
#include "ui/settings.h"
#include "ui/title.h"

static char const TAG[] = "nisp";

// --- Which world the title and the tests stand on -----------------------

// The title screen and the bench flight both need an island that is the
// same on every badge and every run, so they use a fixed seed rather than
// the seed of whatever world the player last had open.
#define BENCH_SEED 0x49534c50u  // "ISLP"

// The deep space behind the island. The stars are geometry, so this is
// only what the scene leaves uncovered.
#define SKY_ARGB 0xff05070du

// --- The state ----------------------------------------------------------

typedef enum {
    MODE_TITLE = 0,  // the island drifts; the title bar sits over it
    MODE_PLAY,       // the ship has the keyboard
    MODE_PAUSED      // the ship is frozen; the pause menu has it
} mode_t;

static mode_t s_mode = MODE_TITLE;

// The open world, and which slot it came from. `s_slot` is -1 on the title
// and on a bench run, where there is no save to write back to.
static nisp_game_t s_game;
static int s_slot = -1;

// This frame's camera: title_pose() on the title, the ship in flight, and
// wherever the ship stopped while paused.
static nisp_pose_t s_pose;
static float s_vel[3];

static float s_fps;         // what the HUD reports, updated each second
static bool s_info;         // the INFO key's diagnostic block
static char s_msg[64];      // the HUD's line of news ...
static double s_msg_until;  // ... and how long it is worth showing

// Half-resolution rendering (Graphics > Half resolution). The layer is the
// scene's target; the PPA scales it up onto the engine's framebuffer.
#define JOB_UPSCALE 1u
static se_ppa_layer_t s_half = {0};
static bool s_half_ok = true;

// The stream is started where the framebuffer is, which is on_render, not
// where the menu row that asks for it is (se_stream.h wants the fb).
static bool s_stream_wanted;

// The test kit's content (testkit/devtest.h): what is playing, when it
// started, and whether a test -- rather than a person -- is flying.
static char s_content[24] = "title";
static double s_content_t0;
static bool s_bench;
static bool s_shot_wanted;

// --- Small things -------------------------------------------------------

static void set_content(char const* name) {
    snprintf(s_content, sizeof(s_content), "%s", name);
    s_content_t0 = showtime_now();
}

static void flash(char const* msg) {
    snprintf(s_msg, sizeof(s_msg), "%s", msg);
    s_msg_until = showtime_now() + 2.5;
}

// The mixer follows the settings file (ui/settings.h). The menus apply
// their own changes as they make them; this is the boot-time pass, the one
// that has to happen after settings_init() has read the file.
static void apply_audio(void) {
    audio_mixer_set_music_enabled(settings_music());
    audio_mixer_set_music_volume(settings_music_volume());
    audio_mixer_set_group_enabled(0, settings_effects());
    audio_mixer_set_group_volume(0, settings_effects_volume());
}

// --- Opening and leaving a world ----------------------------------------

static void go_title(void) {
    s_mode = MODE_TITLE;
    s_slot = -1;
    s_bench = false;
    s_info = false;
    s_vel[0] = s_vel[1] = s_vel[2] = 0.0f;
    nisp_scene_init(BENCH_SEED);
    title_pose(0.0, &s_pose);
    menu_open_title();
    set_content("title");
}

// The world is open: the scene is built from its seed, the camera is where
// the save left it, and the ship is standing still.
static void open_game(int slot, nisp_game_t const* game) {
    s_game = *game;
    s_slot = slot;
    s_bench = false;
    s_vel[0] = s_vel[1] = s_vel[2] = 0.0f;
    s_pose = game->pose;
    nisp_scene_init(game->seed);
    s_mode = MODE_PLAY;
    menu_close();
    set_content("space");
}

// The scripted flight a test flies. It touches no save: a test of the
// renderer must not be a test that rewrites someone's world.
static void open_bench(void) {
    s_game = (nisp_game_t){.name = "Bench", .seed = BENCH_SEED, .flown = 0.0f};
    s_slot = -1;
    s_bench = true;
    s_vel[0] = s_vel[1] = s_vel[2] = 0.0f;
    nisp_scene_init(BENCH_SEED);
    s_mode = MODE_PLAY;
    menu_close();
    set_content("space");
}

static void save_open(char const* news) {
    if (s_slot < 0) return;  // the bench has nothing to save
    s_game.pose = s_pose;
    if (save_write(s_slot, &s_game)) {
        flash(news);
        menu_status(news);
    } else {
        flash("Could not save");
        menu_status("Could not save");
    }
}

static void leave(void) {
    if (s_slot >= 0) {
        s_game.pose = s_pose;
        save_write(s_slot, &s_game);  // best effort: leaving should not lose a flight
    }
    if (se_stream_running()) se_stream_stop();
    audio_mixer_shutdown();
    logo_free();
    ESP_LOGI(TAG, "back to the launcher");
    bsp_device_restart_to_launcher();
}

// --- What the menu decided ----------------------------------------------

static void handle_cmd(menu_cmd_t const* cmd) {
    switch (cmd->kind) {
        case MENU_CMD_PLAY: {
            nisp_game_t game;
            if (save_load(cmd->slot, &game)) {
                open_game(cmd->slot, &game);
            } else {
                menu_status("Could not open that world");
            }
            break;
        }
        case MENU_CMD_CREATE: {
            nisp_game_t game;
            if (save_new(cmd->slot, cmd->name, cmd->seed, &game)) {
                open_game(cmd->slot, &game);
                flash("A new world");
            } else {
                menu_status("Could not create that world");
            }
            break;
        }
        case MENU_CMD_RESUME:
            menu_close();
            s_mode = MODE_PLAY;
            break;
        case MENU_CMD_SAVE: save_open("Saved"); break;
        case MENU_CMD_SAVE_QUIT:
            save_open("Saved");
            go_title();
            break;
        case MENU_CMD_LEAVE: leave(); break;
        case MENU_CMD_STREAM: s_stream_wanted = true; break;
        case MENU_CMD_NONE: break;
    }
}

// --- The ship -----------------------------------------------------------

#define THRUST      9.0f   // m/s^2, on any axis
#define BOOST_SCALE 2.5f
#define DRAG        1.6f   // per second: off the stick, the ship settles
#define TOP_SPEED   26.0f  // m/s
#define CLEARANCE   1.7f   // the ship does not occupy the ground
#define PITCH_LIMIT 1.5f   // just short of straight up or down

static void fly(nisp_actions_t const mask, float dt) {
    // The ship's own axes at zero roll -- the same basis the camera is
    // built from (se_scene.c, camera_build_basis), so what you aim at is
    // what you thrust towards.
    float const cy = cosf(s_pose.yaw), sy = sinf(s_pose.yaw);
    float const cp = cosf(s_pose.pitch), sp = sinf(s_pose.pitch);
    float const fwd[3]   = {sy * cp, -sp, cy * cp};
    float const right[3] = {cy, 0.0f, -sy};

    float const scale = act_held(mask, NISP_BOOST) ? BOOST_SCALE : 1.0f;
    float a[3] = {0.0f, 0.0f, 0.0f};
    if (act_held(mask, NISP_FORWARD)) {
        a[0] += fwd[0];
        a[1] += fwd[1];
        a[2] += fwd[2];
    }
    if (act_held(mask, NISP_BACK)) {
        a[0] -= fwd[0];
        a[1] -= fwd[1];
        a[2] -= fwd[2];
    }
    if (act_held(mask, NISP_RIGHT)) {
        a[0] += right[0];
        a[1] += right[1];
        a[2] += right[2];
    }
    if (act_held(mask, NISP_LEFT)) {
        a[0] -= right[0];
        a[1] -= right[1];
        a[2] -= right[2];
    }
    // Up and down are the WORLD's, not the ship's: the keys say "Thrust
    // up", and a ship pointed at the horizon should still go up.
    if (act_held(mask, NISP_UP)) a[1] += 1.0f;
    if (act_held(mask, NISP_DOWN)) a[1] -= 1.0f;

    for (int i = 0; i < 3; i++) {
        s_vel[i] += a[i] * THRUST * scale * dt;
        s_vel[i] -= s_vel[i] * DRAG * dt;
    }

    float speed = sqrtf(s_vel[0] * s_vel[0] + s_vel[1] * s_vel[1] + s_vel[2] * s_vel[2]);
    if (speed > TOP_SPEED) {
        float const k = TOP_SPEED / speed;
        s_vel[0] *= k;
        s_vel[1] *= k;
        s_vel[2] *= k;
        speed = TOP_SPEED;
    }

    s_pose.x += s_vel[0] * dt;
    s_pose.y += s_vel[1] * dt;
    s_pose.z += s_vel[2] * dt;

    // Ground is ground. Landing sets the ship down and kills the descent,
    // which is the difference between a landing and a hole.
    float const floor = nisp_scene_ground(s_pose.x, s_pose.z) + CLEARANCE;
    if (s_pose.y < floor) {
        s_pose.y = floor;
        if (s_vel[1] < 0.0f) s_vel[1] = 0.0f;
    }

    s_game.flown += speed * dt;

    float dyaw = 0.0f, dpitch = 0.0f;
    input_look(mask, &dyaw, &dpitch);
    s_pose.yaw += dyaw * dt;
    s_pose.pitch += dpitch * dt;
    if (s_pose.pitch > PITCH_LIMIT) s_pose.pitch = PITCH_LIMIT;
    if (s_pose.pitch < -PITCH_LIMIT) s_pose.pitch = -PITCH_LIMIT;
}

// --- The scripted flight (testkit/devtest.h) ----------------------------

// Out from the pad, round the island, climbing: everything a frame needs
// is a function of the time since the content started, so the same
// millisecond draws the same picture in a realtime run and in a shot test
// that sets the clock.
#define BENCH_RATE  0.6f   // radians a second round the pad
#define BENCH_R0    14.0f  // metres, and ...
#define BENCH_RK    1.2f   // ... metres a second further out
#define BENCH_H0    6.0f   // metres up, and ...
#define BENCH_HK    0.8f   // ... metres a second higher
#define BENCH_SECS  20.0f  // what a perf test runs for, unasked

static void bench_pose(double t, nisp_pose_t* out) {
    float const k = (float)t;
    float const a = BENCH_RATE * k;
    float const r = BENCH_R0 + BENCH_RK * k;
    float const h = BENCH_H0 + BENCH_HK * k;

    out->x = cosf(a) * r;
    out->y = h;
    out->z = sinf(a) * r;
    // Looking back at the obelisk, and down by the ship's own height.
    out->yaw = atan2f(-cosf(a), -sinf(a));
    out->pitch = atan2f(h, r);
}

// The HUD's "m flown", for the same path: integrated over fixed steps of
// `t` rather than accumulated frame by frame, so it too is a function of
// the clock and a reference screenshot keeps its hash.
static float bench_flown(float t) {
    int const steps = 64;
    float const dt = t / (float)steps;
    if (dt <= 0.0f) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < steps; i++) {
        float const mid     = ((float)i + 0.5f) * dt;
        float const r       = BENCH_R0 + BENCH_RK * mid;
        float const tangent = BENCH_RATE * r;
        sum += sqrtf(tangent * tangent + BENCH_RK * BENCH_RK + BENCH_HK * BENCH_HK) * dt;
    }
    return sum;
}

// --- The test kit -------------------------------------------------------

static bool content_select(char const* name) {
    if (name == NULL || name[0] == '\0') return false;

    if (!strcmp(name, "title")) {
        go_title();
        return true;
    }
    if (!strcmp(name, "space") || !strcmp(name, "bench") || !strcmp(name, "island")) {
        open_bench();
        return true;
    }
    // Every menu screen is a piece of content too: they are drawn from
    // state, so a shot of each is a regression test of each.
    if (menu_show(name)) {
        set_content(name);
        return true;
    }
    ESP_LOGW(TAG, "no such content: '%s'", name);
    return false;
}

static float content_duration(void) {
    return s_bench ? BENCH_SECS : -1.0f;  // endless: a perf test picks its own length
}
static double content_started(void) {
    return s_content_t0;
}
static char const* content_name(void) {
    return s_content;
}
static char const* content_shot(void) {
    return "";  // no shots within a piece of content yet
}

static devtest_content_t const CONTENT = {
    .select    = content_select,
    .duration  = content_duration,
    .started   = content_started,
    .name      = content_name,
    .shot_name = content_shot,
};

static void stats_restart(void) {
    prof_reset();
}

static devtest_config_t const TEST = {
    .app           = "at.cavac.nisp",
    .shot_dir      = SCREENSHOT_DIR,
    .content       = &CONTENT,
    .stats_restart = stats_restart,
};

// Once a second: the frame rate, the phase split, and where the ship is.
static void frame_stats(void) {
    static int64_t s_window_us;
    static int s_frames;
    static int64_t s_last_us;

    int64_t const now = esp_timer_get_time();
    if (s_last_us != 0) s_window_us += now - s_last_us;
    s_last_us = now;
    s_frames++;

    // The present is the one phase the app cannot wrap in a begin/end pair
    // -- it happens after on_render returns -- so it reports the PREVIOUS
    // frame's, which over a reporting period is the same number. Without
    // this the blit and the vsync wait hide in the residual.
    int64_t blit_us = 0, vsync_us = 0;
    se_present_stats(&blit_us, &vsync_us);
    prof_add(PROF_BLIT, blit_us);
    prof_add(PROF_VSYNC, vsync_us);
    prof_frame();

    if (s_window_us < 1000000) return;

    float const fps      = (float)s_frames * 1000000.0f / (float)s_window_us;
    float const frame_ms = (float)s_window_us / 1000.0f / (float)s_frames;
    s_fps = fps;
    devtest_period(fps, frame_ms);

    char split[192];
    prof_flush(split, sizeof(split), frame_ms);
    ESP_LOGI(TAG, "%s | %s t=%.1f  x %+.1f y %+.1f z %+.1f", split, s_content, showtime_now() - s_content_t0,
             (double)s_pose.x, (double)s_pose.y, (double)s_pose.z);

    s_window_us = 0;
    s_frames    = 0;
}

// --- Callbacks ----------------------------------------------------------

// Once, after the engine has brought up the display, audio, input and
// scene.
static void on_init(void* user) {
    (void)user;
    ESP_LOGI(TAG, "Null Island Space Program %s on SynthEngine3D %s (%s, built %s)", APP_VERSION,
             se_version_string(), APP_GIT_HASH, APP_BUILD_TIME);

    // THE ENGINE'S OWN SPLASH, the first thing the program shows: the
    // SynthEngine logo and its version. The game's own name waits for the
    // title screen, which is where a title belongs (ui/title.c).
    se_splash();

    scene_init();
    // Bindings first: the settings file restores them onto whatever is
    // registered, so an unregistered action would keep its default.
    input_init();
    // Saves before settings: save_init() makes /sd/nisp, which is where
    // settings.txt lives (common/paths.h).
    save_init();
    settings_init();

    // The score: the shipped recording off the card, looped (audio/music.h).
    // The procedural generator is what plays when the card has no music,
    // not what plays instead of it.
    music_init();
    apply_audio();

    // The logo is a file on the card beside the app (ui/logo.h), not a
    // blob in the binary.
    if (!logo_load()) ESP_LOGW(TAG, "no %s beside the app; the title shows words only", LOGO_FILE);

    // The half-size layer the scene can draw into. It has to match the
    // framebuffer's format and orientation, which the engine resolved at
    // bootstrap.
    if (se_ppa_init()) {
        se_display_info_t di;
        se_display_info(&di);
        if (se_ppa_layer_alloc(&s_half, DISPLAY_LOG_W / 2, DISPLAY_LOG_H / 2, di.pax_format, di.reversed,
                               di.orientation)) {
            ESP_LOGI(TAG, "half-resolution layer: %dx%d", DISPLAY_LOG_W / 2, DISPLAY_LOG_H / 2);
        } else {
            ESP_LOGW(TAG, "no half-resolution layer; drawing at full resolution");
            s_half_ok = false;
        }
    } else {
        ESP_LOGW(TAG, "PPA unavailable; drawing at full resolution");
        s_half_ok = false;
    }
    menu_half_res_available(s_half_ok);

    // Both opt-in and both cheap to settle here: culling is a near-pure
    // win, depth ordering trades a sort against overdraw.
    scene_set_options(&(se_scene_options_t){.frustum_cull = true, .depth_order = true});

    go_title();
    devtest_start(&TEST);
}

// Every key the engine did not answer itself. The menus take all of them;
// the ship is steered by polling instead (input_sample).
static void on_input(bsp_input_event_t const* ev, void* user) {
    (void)user;
    menu_event(ev);
}

// The engine clears the backdrop only when there is no on_backdrop; this
// app paints every pixel of every frame, so the callback is empty on
// purpose.
static void on_backdrop(pax_buf_t* fb, void* user) {
    (void)fb;
    (void)user;
}

static void on_update(float dt, void* user) {
    (void)user;

    showtime_frame();
    devtest_update();

    menu_cmd_t const cmd = menu_update();
    handle_cmd(&cmd);

    double const t = showtime_now() - s_content_t0;

    // The keyboard is read every frame, menu up or not, so the "which keys
    // went down since the last frame" edge is never stale on the frame a
    // menu closes.
    nisp_actions_t const mask    = input_sample();
    nisp_actions_t const pressed = input_pressed();

    if (s_mode == MODE_TITLE) {
        title_pose(t, &s_pose);
        return;
    }

    if (s_bench) {
        bench_pose(t, &s_pose);
        s_game.flown = bench_flown((float)t);
        return;
    }

    if (menu_active()) return;  // paused: the ship holds its position

    fly(mask, dt);

    if (act_held(pressed, NISP_PAUSE)) {
        s_mode = MODE_PAUSED;
        menu_open_pause();
    } else if (act_held(pressed, NISP_SAVE)) {
        save_open("Saved");
    } else if (act_held(pressed, NISP_INFO)) {
        s_info = !s_info;
    } else if (act_held(pressed, NISP_SCREENSHOT)) {
        s_shot_wanted = true;
    }
}

static void on_render(pax_buf_t* fb, void* user) {
    (void)user;

    double const t = showtime_now() - s_content_t0;

    if (s_stream_wanted) {
        s_stream_wanted = false;
        if (se_stream_running()) {
            se_stream_stop();
            flash("Livestream off");
        } else if (se_stream_start(&(se_stream_cfg_t){.fps_hint = 30, .audio = true}, fb) == ESP_OK) {
            flash("Livestream on");
        } else {
            flash("Livestream failed");
        }
    }

    bool const half       = s_half_ok && settings_half_res();
    pax_buf_t* const target = half ? &s_half.buf : fb;
    scene_set_render_scale(half ? 2 : 1);

    // The sky. The engine would have cleared `fb`, but when the scene
    // draws into the half-size layer it is that buffer which needs a
    // background: whatever the island does not cover is what shows through
    // the upscale.
    prof_begin(PROF_FILL);
    if (half) {
        se_ppa_fill(target, 0, 0, DISPLAY_LOG_H / 2, SKY_ARGB);
        se_ppa_wait_job(0);
    } else {
        // Straight into the framebuffer, so a CPU clear: the CPU draws
        // into this buffer next, and a DMA fill under its cache is a
        // coherence problem this path is not worth having.
        pax_background(fb, SKY_ARGB);
    }
    prof_end(PROF_FILL);

    // The island, behind everything: the title's drift, the ship's view,
    // and the frozen view a menu sits on. It times its own phases.
    int const view = (s_mode == MODE_TITLE) ? NISP_VIEW_COUNT - 1 : settings_view();
    int64_t const rast_us = nisp_scene_render(target, &s_pose, view, t);

    if (half) {
        prof_begin(PROF_WAIT);
        // The CPU's pixels have to reach PSRAM before the PPA's DMA reads
        // them, and the framebuffer has to leave the CPU's cache after the
        // PPA has written it -- the screenshot path reads it.
        se_ppa_layer_sync(&s_half);
        if (se_ppa_blit_scaled(fb, JOB_UPSCALE, &s_half, 2)) {
            se_ppa_wait_job(JOB_UPSCALE);
            se_ppa_buf_invalidate(fb);
        }
        prof_end(PROF_WAIT);
    }

    // After the upscale, so the text is drawn at full resolution rather
    // than doubled up with the world; before the shot is taken, so a
    // screenshot shows what the player saw.
    // The artwork is timed on its own, ahead of the rest: it is the one
    // overlay item that covers enough pixels to be worth naming, and a
    // separate number is the difference between "the overlay is slow" and
    // "the logo is slow".
    prof_begin(PROF_LOGO);
    if (s_mode == MODE_TITLE && !menu_in_panel()) title_logo(fb);
    prof_end(PROF_LOGO);

    prof_begin(PROF_OVERLAY);
    if (s_mode == MODE_TITLE) {
        if (!menu_in_panel()) title_draw(fb, t);
    } else {
        hud_stats_t const stats = {
            .fps     = s_fps,
            .slot    = s_slot,
            .rast_us = rast_us,
            .half    = half,
        };
        hud_draw(fb, &s_game, &s_pose, s_info, &stats);
        if (s_mode == MODE_PLAY) hud_reticle(fb);
        if (showtime_now() < s_msg_until) hud_flash(fb, s_msg);
    }

    menu_draw(fb);
    prof_end(PROF_OVERLAY);

    // Last, so the shot has everything on it. The stall it causes comes
    // out of the show clock, so the next frame steps by a frame rather
    // than by the whole second the write took.
    if (s_shot_wanted) {
        s_shot_wanted = false;
        int64_t const t0 = esp_timer_get_time();
        if (!screenshot_capture(fb)) ESP_LOGW(TAG, "screenshot failed");
        showtime_exclude(esp_timer_get_time() - t0);
    }

    // The same frame to OBS, if the player asked for it: this is the last
    // moment the frame is ours, before the engine flips pages. A no-op and
    // one compare when the stream is off.
    se_stream_frame(fb);

    devtest_after_render(fb, rast_us);
    frame_stats();
}

static void on_shutdown(void* user) {
    (void)user;
    if (se_stream_running()) se_stream_stop();
    audio_mixer_shutdown();
    logo_free();
}

void app_main(void) {
    static se_app_config_t const cfg = {
        // F1 is the way out of an app on this badge and the menu has no
        // row for it, so the engine keeps it (se_run.h). Nothing here is
        // bound to F1 (game/input.c).
        .f1_exits      = true,
        .backdrop_argb = SKY_ARGB,
    };
    static se_app_callbacks_t const cb = {
        .on_init     = on_init,
        .on_input    = on_input,
        .on_update   = on_update,
        .on_backdrop = on_backdrop,
        .on_render   = on_render,
        .on_shutdown = on_shutdown,
    };
    se_run(&cfg, &cb, NULL);  // no user context: the state is static
}
