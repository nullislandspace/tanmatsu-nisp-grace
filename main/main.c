// =====================================================================
//  Null Island Space Program  --  the program
// ---------------------------------------------------------------------
//  The engine owns the frame loop (se_run): the device bootstrap, the
//  input pump, the device-global keys, the page flip. What lives here is
//  the state machine and the ship:
//
//    title    the solar system drifting past, the wordmark over it, and
//             the title bar's Play / Settings / Quit over that (the
//             livestream is a setting, not a title bar item -- ui/menu.c)
//    play     the mission, the chase camera on the ship, and the HUD
//    paused   the mission frozen under the pause menu
//    world    a save is open and nothing is flying: the Spaceport hub
//             (ui/menu.c) over the drifting sky
//    engineering  the blueprint sheet (engineering/builder.c), the one
//             screen in the game that is not the 3D scene at all
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
//  flight steered by a stick cannot be that, so the bench content is not
//  a flight at all: it is an orbit propagated analytically from a fixed
//  set of orbital elements (bench_orbit), which is a pure function of the
//  clock and still exercises the physics readout, the renderer, the HUD
//  and the menus. A real mission steps on the show clock too -- its frame
//  dt is the difference between two showtime_now() calls, not the engine's
//  dt -- so that the mission and the picture advance together.
//
//  NO CHUNK STREAMING, and no memory reserved for it at boot. The world
//  is three spheres and a starfield, submitted in full every frame
//  (game/scene.h); there is nothing to page in and no loader thread.
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
#include "common/quat.h"
#include "engineering/builder.h"
#include "flight/control.h"
#include "flight/mission.h"
#include "flight/physics.h"
#include "flight/warp.h"
#include "game/hud.h"
#include "game/input.h"
#include "game/scene.h"
#include "rocket/design.h"
#include "save/save.h"
#include "sim/bodies.h"
#include "sim/kepler.h"
#include "sim/surface.h"
#include "sim/units.h"
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

// One sky for everybody (D-50). The scene is seeded -- the starfield and
// the surface noise come out of this number -- but the seed is a constant in
// the program rather than something a player picks or a save carries. The
// title, the bench flight and every world stand on the same ground, which
// is also what makes a screenshot of one comparable to a screenshot of
// another.
#define NISP_WORLD_SEED 0x4e495350u  // "NISP"
#define BENCH_SEED NISP_WORLD_SEED

// The deep space behind the world. The stars are geometry, so this is
// only what the scene leaves uncovered.
#define SKY_ARGB 0xff05070du

// --- The state ----------------------------------------------------------

typedef enum {
    MODE_TITLE = 0,    // the sky drifts; the title bar sits over it
    MODE_PLAY,         // the ship has the keyboard
    MODE_PAUSED,       // the ship is frozen; the pause menu has it
    MODE_WORLD,        // a save is open, nothing is flying: the Spaceport hub
    MODE_ENGINEERING   // the blueprint sheet has the keyboard
} mode_t;

static mode_t s_mode = MODE_TITLE;

// The open world, and which slot it came from. `s_slot` is -1 on the title
// and on a bench run, where there is no save to write back to.
static nisp_game_t s_game;
static int s_slot = -1;

// This frame's camera: nisp_camera_title() on the title, the chase camera
// on the ship, and wherever the chase camera stopped while paused. The
// look keys are offsets on top of whichever of those it is, so releasing
// them and pressing them again never moves the world.
static nisp_camera_t s_cam;
static float s_look_yaw, s_look_pitch;  // radians

// The flight. One mission, one control struct, one readout: flight/ owns
// the ship and decides what it does, this file only decides what to ask
// for and how often to ask. `s_flying` says a mission exists at all, which
// is the difference between the HUD reporting a flight and reporting a
// name over a frozen view.
static nisp_mission_t s_mission;
static nisp_control_t s_ctl;
static nisp_readout_t s_readout;
static nisp_design_t s_design;
static bool s_flying;

// The drawing Engineering is holding (engineering/builder.h). It is the
// working design, not a saved one: it reaches the card only through Save
// (plan 4.6), and it is what a launch from Engineering flies.
static nisp_builder_t s_builder;

// The bench orbit (see the bench section): the elements it is defined by,
// so its state at any show time is one propagation away.
static nisp_elements_t s_bench_el;

// 100 km up, and a quarter of the way round from the pad: high enough to
// be vacuum and to show the curvature, low enough that Bob fills the
// frame, and off the pad so the pad is in shot as well.
#define BENCH_ALT_KM 100.0f
#define BENCH_NU0    (NISP_TWO_PI * 0.25)
#define BENCH_SECS   20.0f  // what a perf test runs for, unasked

// The show clock at the last frame. The mission's frame dt is the
// difference between two showtime_now() calls rather than the engine's dt,
// so the sim and the picture advance by exactly the same amount.
static double s_show_last;

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

// --- The ship: how big it is, how it is steered -------------------------

// The rocket's length along its own axis, metres. The stack stores parts at
// a distance from the nose, so the longest part is the tail.
static float ship_length_m(nisp_vessel_t const* v) {
    float x = 1.0f;
    if (v == NULL) return x;
    for (int i = 0; i < v->stack.count; i++) {
        if (v->stack.part[i].x1 > x) x = v->stack.part[i].x1;
    }
    return x;
}

// Chase standoff, from the rocket's own length. A 6 m pod wants a camera 20
// m back; a 40 m launcher wants one that can see all of it. A fixed number
// would be either inside the rocket or leave it a speck, and the whole
// point of the chase camera is to read the vehicle you are flying.
static float ship_back_m(nisp_vessel_t const* v) {
    return 2.5f * ship_length_m(v) + 15.0f;
}
static float ship_up_m(nisp_vessel_t const* v) {
    return 0.6f * ship_length_m(v) + 5.0f;
}

// The bench: an orbit, not a flight. The test kit sets the clock rather
// than running it (testkit/devtest.h), and a mission stepped frame by
// frame is a function of where the frames happened to fall. An analytic
// propagation from a fixed set of elements is a function of the clock
// alone, so `shots scene=space` means the same thing here that it means
// for a title screen. `t` is the bench's own time, seconds.
static void bench_orbit(double t) {
    nisp_design_demo(&s_design);
    nisp_elements_circular(BODY_BOB, nisp_body(BODY_BOB)->radius_km + BENCH_ALT_KM, BENCH_NU0, 0.0,
                           &s_bench_el);

    nisp_state_t st;
    nisp_elements_to_state(&s_bench_el, t, &st);

    // A nose along the velocity: the attitude a coasting ship keeps, and
    // the one a HUD is easiest to read against a circular orbit.
    float pro[3] = {(float)st.v[0], (float)st.v[1], (float)st.v[2]};
    nisp_vessel_orbit(&s_mission.ship, &s_design, BODY_BOB, &st, t, pro);

    nisp_mission_begin(&s_mission, &s_mission.ship, t);
    nisp_control_init(&s_ctl);
    s_ctl.sas = NISP_SAS_HOLD;
    s_ctl.rcs = true;
}

// The bench's frame: propagate, aim the nose along the orbit, read it. No
// mission step -- the propagation IS the step, and it is exact.
static void bench_frame(double t) {
    nisp_state_t st;
    nisp_elements_to_state(&s_bench_el, t, &st);
    for (int i = 0; i < 3; i++) {
        s_mission.ship.pos_km[i] = st.r[i];
        s_mission.ship.vel_kms[i] = st.v[i];
    }
    s_mission.ship.omega[0] = s_mission.ship.omega[1] = s_mission.ship.omega[2] = 0.0f;

    float pro[3];
    nisp_vessel_prograde(&s_mission.ship, pro);
    nisp_quat_from_dir(pro, &s_mission.ship.q);

    s_mission.t = t;
    nisp_vessel_readout(&s_mission.ship, t, &s_readout);
    s_mission.music_space = s_readout.alt_km >= NISP_MUSIC_SPACE_KM;
}

// The manual keys, in the pilot's terms rather than the ship's. W and S
// raise and lower the nose, A and D swing it left and right, Q and E roll
// it -- and the ship's own axes have no reason to line up with the horizon,
// because the attitude a rocket inherits from its orbit is whatever the
// last coast left it with. So the wanted rotation is built in the frame at
// the ship and turned into vehicle axes, which is the only place the
// control system reads torque from.
//
// In the frame at the ship: a rotation about minus the screen-right axis
// raises the nose, a rotation about the local vertical swings it right, and
// a rotation about minus the nose rolls it right on screen.
static void manual_torque(nisp_actions_t const mask, float out[3]) {
    out[0] = out[1] = out[2] = 0.0f;
    if (mask == 0u) return;

    float nose[3];
    nisp_vessel_nose(&s_mission.ship, nose);

    double up_d[3], north[3], east[3];
    nisp_local_frame(s_mission.ship.body, s_mission.ship.pos_km, up_d, north, east);
    float up[3] = {(float)up_d[0], (float)up_d[1], (float)up_d[2]};

    float right[3] = {up[1] * nose[2] - up[2] * nose[1], up[2] * nose[0] - up[0] * nose[2],
                      up[0] * nose[1] - up[1] * nose[0]};
    float const rn = sqrtf(right[0] * right[0] + right[1] * right[1] + right[2] * right[2]);
    if (rn < 1e-6f) return;  // straight up or straight down: there is no right
    for (int i = 0; i < 3; i++) right[i] /= rn;

    // Full deflection on every axis the keys are asking for, so a press is
    // "all the authority there is" (flight/control.h) and the ship's
    // response is the rocket's, not the keyboard's.
    float w[3] = {0.0f, 0.0f, 0.0f};
    float const a = NISP_MANUAL_TORQUE_KNM;
    for (int i = 0; i < 3; i++) {
        if (act_held(mask, NISP_BACK)) w[i] += up[i] * a;
        if (act_held(mask, NISP_FORWARD)) w[i] -= up[i] * a;
        if (act_held(mask, NISP_RIGHT)) w[i] += right[i] * a;
        if (act_held(mask, NISP_LEFT)) w[i] -= right[i] * a;
        if (act_held(mask, NISP_ROLL_RIGHT)) w[i] -= nose[i] * a;
        if (act_held(mask, NISP_ROLL_LEFT)) w[i] += nose[i] * a;
    }

    nisp_quat_rotate_inv(&s_mission.ship.q, w, out);
}

// The keys, once a frame, into what the mission is asked to do. The
// throttle is held and integrates; everything else is an edge, because a
// key held down is not the same command pressed sixty times.
static void fill_control(nisp_actions_t const mask, nisp_actions_t const pressed, float dt) {
    float const scale = act_held(mask, NISP_BOOST) ? 2.5f : 1.0f;
    if (act_held(mask, NISP_UP)) s_ctl.throttle += 0.6f * scale * dt;
    if (act_held(mask, NISP_DOWN)) s_ctl.throttle -= 0.6f * scale * dt;
    if (s_ctl.throttle < 0.0f) s_ctl.throttle = 0.0f;
    if (s_ctl.throttle > 1.0f) s_ctl.throttle = 1.0f;

    if (act_held(pressed, NISP_CUT)) {
        s_ctl.cut = true;
        s_ctl.throttle = 0.0f;
    }
    if (act_held(pressed, NISP_STAGE)) s_ctl.stage = true;
    if (act_held(pressed, NISP_CHUTE)) s_ctl.chute = true;
    if (act_held(pressed, NISP_LEGS)) s_ctl.legs = true;
    if (act_held(pressed, NISP_SAS)) s_ctl.sas = (s_ctl.sas + 1) % NISP_SAS_COUNT;
    if (act_held(pressed, NISP_RCS)) s_ctl.rcs = !s_ctl.rcs;

    if (act_held(pressed, NISP_WARP_UP)) {
        int const i = nisp_warp_index_for(s_ctl.warp) + 1;
        s_ctl.warp = nisp_warp_level(i < nisp_warp_levels() ? i : nisp_warp_levels() - 1);
    }
    if (act_held(pressed, NISP_WARP_DOWN)) {
        int const i = nisp_warp_index_for(s_ctl.warp) - 1;
        s_ctl.warp = nisp_warp_level(i > 0 ? i : 0);
    }

    manual_torque(mask, s_ctl.torque);
}

// --- Opening and leaving a world ----------------------------------------

static void go_title(void) {
    s_mode = MODE_TITLE;
    s_slot = -1;
    s_bench = false;
    s_flying = false;
    s_info = false;
    s_look_yaw = s_look_pitch = 0.0f;
    nisp_scene_init(BENCH_SEED);
    nisp_camera_title(&s_cam, 0.0);
    menu_open_title();
    music_set_context(MUSIC_MENU);
    set_content("title");
}

// The world is open and nothing is flying: the sky, and the Spaceport hub
// over it (plan 4.7). A world is not a mission -- which blueprint leaves the
// pad is a separate decision the hub makes.
static void open_world(int slot, nisp_game_t const* game) {
    s_game = *game;
    s_slot = slot;
    s_bench = false;
    s_flying = false;
    s_look_yaw = s_look_pitch = 0.0f;
    nisp_scene_init(game->seed);
    nisp_camera_title(&s_cam, 0.0);

    s_mode = MODE_WORLD;
    menu_open_spaceport(slot);
    music_set_context(MUSIC_MENU);
    set_content("spaceport");
}

// A blueprint leaves the pad (plan 5.6). The design is the one that was
// chosen and not a stand-in, so the pad, the physics and the HUD are the
// ones the drawing described. The mission clock starts at the launch, which
// is also where the content clock restarts: t == 0 is standing on the pad.
static void launch_design(nisp_design_t const* design) {
    s_design = *design;
    s_bench = false;
    s_look_yaw = s_look_pitch = 0.0f;

    nisp_mission_launch(&s_mission, &s_design, BODY_BOB, 0.0);
    nisp_control_init(&s_ctl);
    nisp_mission_readout(&s_mission, &s_readout);
    nisp_camera_chase(&s_cam, &s_mission.ship, 0.0, ship_back_m(&s_mission.ship),
                      ship_up_m(&s_mission.ship));
    s_flying = true;

    s_mode = MODE_PLAY;
    menu_close();
    music_set_context(MUSIC_NONE);  // on the pad, in the air: silence (plan 5.8)
    set_content("space");
}

// Engineering (plan 4.2-4.6): blue paper and nothing else. `design` == NULL
// starts a fresh unnamed drawing; otherwise the saved blueprint is what is
// on the paper, and Save writes over it.
static void open_engineering(nisp_design_t const* design) {
    nisp_builder_open(&s_builder, design);
    s_bench = false;
    s_mode = MODE_ENGINEERING;
    menu_close();
    music_set_context(MUSIC_ENGINEERING);
    set_content("engineering");
}

// Backing out of Engineering leaves the drawing behind it: an unsaved one is
// simply gone, which is what "working design" means (plan 4.6).
static void close_engineering(void) {
    if (s_slot >= 0) {
        s_mode = MODE_WORLD;
        menu_open_spaceport(s_slot);
        set_content("spaceport");
    } else {
        s_mode = MODE_TITLE;
        menu_open_title();
        set_content("title");
    }
    music_set_context(MUSIC_MENU);
}

// The scripted flight a test flies. It touches no save: a test of the
// renderer must not be a test that rewrites someone's world.
static void open_bench(void) {
    s_game = (nisp_game_t){.name = "Bench", .seed = BENCH_SEED, .flown = 0.0f};
    s_slot = -1;
    s_bench = true;
    s_look_yaw = s_look_pitch = 0.0f;
    nisp_scene_init(BENCH_SEED);

    bench_orbit(0.0);
    s_flying = true;

    s_mode = MODE_PLAY;
    menu_close();
    music_set_context(MUSIC_SPACE);
    set_content("space");
}

static void save_open(char const* news) {
    if (s_slot < 0) return;  // the bench has nothing to save
    // What a save keeps is the world and the name, not the flight: a
    // mission is resumed from the mission itself, which is plan 9.1.
    if (save_write(s_slot, &s_game)) {
        flash(news);
        menu_status(news);
    } else {
        flash("Could not save");
        menu_status("Could not save");
    }
}

static void leave(void) {
    if (s_slot >= 0) save_write(s_slot, &s_game);  // best effort: leaving should not lose a flight
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
                open_world(cmd->slot, &game);
            } else {
                menu_status("Could not open that world");
            }
            break;
        }
        case MENU_CMD_CREATE: {
            nisp_game_t game;
            if (save_new(cmd->slot, cmd->name, NISP_WORLD_SEED, &game)) {
                open_world(cmd->slot, &game);
                flash("A new world");
            } else {
                menu_status("Could not create that world");
            }
            break;
        }
        case MENU_CMD_CLOSE_WORLD:
            // The open world was deleted out from under the flight (plan 1.2).
            s_slot = -1;
            s_flying = false;
            s_mode = MODE_TITLE;
            nisp_camera_title(&s_cam, 0.0);
            break;
        case MENU_CMD_RESUME:
            // "Continue mission" is only a mission if one is flying (plan 4.7).
            if (s_flying) {
                menu_close();
                s_mode = MODE_PLAY;
            } else {
                menu_status("No mission in progress");
            }
            break;
        case MENU_CMD_SAVE: save_open("Saved"); break;
        case MENU_CMD_SAVE_QUIT:
            save_open("Saved");
            go_title();
            break;
        case MENU_CMD_LEAVE: leave(); break;
        case MENU_CMD_STREAM: s_stream_wanted = true; break;

        case MENU_CMD_BUILD: open_engineering(NULL); break;
        case MENU_CMD_ENGINEERING: {
            nisp_design_t design;
            if (nisp_design_load(cmd->name, &design)) {
                open_engineering(&design);
            } else {
                menu_status("Could not read that blueprint");
            }
            break;
        }
        case MENU_CMD_LAUNCH: {
            nisp_design_t design;
            if (nisp_design_load(cmd->name, &design)) {
                launch_design(&design);
                flash(cmd->name);
            } else {
                menu_status("Could not read that blueprint");
            }
            break;
        }
        case MENU_CMD_NAME:
            // The name the text entry accepted, for the drawing Engineering
            // is holding (D-11). The panel closed itself on the way out.
            nisp_builder_set_name(&s_builder, cmd->name);
            break;

        case MENU_CMD_NONE: break;
    }
}

// --- Flying one frame ---------------------------------------------------

// A mission frame: the keys into the control struct, the control struct
// into the mission, the mission into the camera and the HUD. The order is
// the mission's own (flight/mission.h); this only supplies the inputs and
// reads the outputs.
static void step_flight(nisp_actions_t const mask, nisp_actions_t const pressed, double dt) {
    fill_control(mask, pressed, (float)dt);
    nisp_mission_step(&s_mission, &s_ctl, dt);
    nisp_mission_readout(&s_mission, &s_readout);

    // Part N4: the mission decides the context, this decides that the
    // audio layer obeys it. Space above 70 km, silence below 60, and the
    // band is the mission's, not a threshold chosen here (flight/mission.h).
    music_set_context(nisp_mission_music_space(&s_mission) ? MUSIC_SPACE : MUSIC_NONE);
}

// The chase camera, and the look keys on top of it. The offsets are kept
// separately from the camera so that whatever aims the camera -- the ship,
// the title's drift -- is never disturbed by where the player happened to
// be looking last frame.
static void chase_camera(double t) {
    nisp_camera_chase(&s_cam, &s_mission.ship, t, ship_back_m(&s_mission.ship),
                      ship_up_m(&s_mission.ship));
    s_cam.yaw += s_look_yaw;
    s_cam.pitch += s_look_pitch;
}

// --- The test kit -------------------------------------------------------

static bool content_select(char const* name) {
    if (name == NULL || name[0] == '\0') return false;

    if (!strcmp(name, "title")) {
        go_title();
        return true;
    }
    // "space" is the flight bench. "orbit" and "bench" are aliases, and
    // "island" is what it was called before the planet replaced the height
    // map -- kept so an old manifest or an old muscle memory still works.
    if (!strcmp(name, "space") || !strcmp(name, "orbit") || !strcmp(name, "bench") ||
        !strcmp(name, "island")) {
        open_bench();
        return true;
    }
    if (!strcmp(name, "engineering")) {
        open_engineering(NULL);
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
    if (s_flying) {
        ESP_LOGI(TAG, "%s | %s t=%.1f  alt %.1f km  %.0f m/s  apo %.1f peri %.1f", split, s_content,
                 s_mission.t, (double)s_readout.alt_km, (double)s_readout.speed_ms,
                 (double)s_readout.apoapsis_km, (double)s_readout.periapsis_km);
    } else {
        ESP_LOGI(TAG, "%s | %s t=%.1f", split, s_content, showtime_now() - s_content_t0);
    }

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
    (void)dt;  // the mission runs on the show clock, not on the engine's dt

    showtime_frame();
    devtest_update();

    // The frame's length, as the show sees it. Taking it from the show
    // clock rather than from the engine is what makes a mission and a
    // screenshot advance by the same amount (testkit/devtest.h), and the
    // clamp is what keeps a stall -- a screenshot write, a slow frame --
    // from becoming a physics step nobody would survive.
    double const now = showtime_now();
    double frame_dt = (s_show_last > 0.0) ? (now - s_show_last) : 0.0;
    if (frame_dt < 0.0) frame_dt = 0.0;
    if (frame_dt > 0.1) frame_dt = 0.1;
    s_show_last = now;

    // Whether a menu had the keyboard when this frame started. It is the
    // only way to tell "the player pressed a key at the menu" apart from
    // "the player pressed a key at the thing the menu was covering".
    bool const menu_was = menu_active();
    menu_cmd_t const cmd = menu_update();
    handle_cmd(&cmd);

    double const t = now - s_content_t0;

    // The keyboard is read every frame, menu up or not, so the "which keys
    // went down since the last frame" edge is never stale on the frame a
    // menu closes.
    nisp_actions_t const mask    = input_sample();
    nisp_actions_t pressed       = input_pressed();

    // THE KEY THAT CLOSED A MENU BELONGS TO THE MENU (D-52). The keyboard is
    // sampled every frame, menu up or not, so the ENTER that accepted a name
    // or the ESC that cancelled one arrives as an edge on the very frame the
    // menu goes away -- and unhandled, the screen underneath acts on it too.
    // In Engineering that read as "Enter does nothing" (the Name action
    // re-opened the panel the same frame it closed) and as "Esc leaves
    // Engineering" from a panel that was only asking for a name.
    if (menu_was && !menu_active()) pressed = 0;

    music_update((float)frame_dt);

    // Engineering has its own keyboard handling: the sheet polls, the name
    // editor is a menu panel and owns every key while it is up (D-11).
    if (s_mode == MODE_ENGINEERING) {
        if (menu_active()) return;

        switch (nisp_builder_step(&s_builder, pressed, (float)frame_dt)) {
            case NISP_BUILD_SAVE:
                nisp_builder_say(&s_builder, nisp_design_save(&s_builder.design) ? "Saved to the card"
                                                                                 : "Could not save");
                break;
            case NISP_BUILD_NAME:
                menu_open_working_name(s_builder.design.name);
                break;
            case NISP_BUILD_LAUNCH:
                // A design is on the card before it leaves the pad (D-11), so
                // the hub's Launch list and the flight never disagree about
                // what flew.
                if (!nisp_design_save(&s_builder.design)) {
                    nisp_builder_say(&s_builder, "Could not save, so not launching");
                    break;
                }
                launch_design(&s_builder.design);
                break;
            case NISP_BUILD_LEAVE: close_engineering(); break;
            default: break;
        }
        return;
    }

    if (s_mode == MODE_TITLE || s_mode == MODE_WORLD) {
        nisp_camera_title(&s_cam, t);
        return;
    }

    if (s_bench) {
        bench_frame(t);
        chase_camera(t);
        return;
    }

    if (menu_active()) return;  // paused: the mission holds its position

    step_flight(mask, pressed, frame_dt);
    chase_camera(t);

    // The look keys are the camera's, not the ship's: they move where you
    // are looking, not where you are going (plan 6.2).
    float dyaw = 0.0f, dpitch = 0.0f;
    input_look(mask, &dyaw, &dpitch);
    s_look_yaw += dyaw * (float)frame_dt;
    s_look_pitch += dpitch * (float)frame_dt;
    if (s_look_pitch > 1.4f) s_look_pitch = 1.4f;
    if (s_look_pitch < -1.4f) s_look_pitch = -1.4f;

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

    // Engineering is a flat drawing on blue paper: no scene, no half-size
    // layer, no upscale, no PPA job (engineering/builder.c).
    bool const sheet      = (s_mode == MODE_ENGINEERING);
    bool const half       = !sheet && s_half_ok && settings_half_res();
    pax_buf_t* const target = half ? &s_half.buf : fb;
    scene_set_render_scale(half ? 2 : 1);

    // The sky. The engine would have cleared `fb`, but when the scene
    // draws into the half-size layer it is that buffer which needs a
    // background: whatever the planets do not cover is what shows through
    // the upscale.
    prof_begin(PROF_FILL);
    if (half) {
        se_ppa_fill(target, 0, 0, DISPLAY_LOG_H / 2, SKY_ARGB);
        se_ppa_wait_job(0);
    } else {
        // Straight into the framebuffer, so a CPU clear: the CPU draws
        // into this buffer next, and a DMA fill under its cache is a
        // coherence problem this path is not worth having.
        pax_background(fb, sheet ? NISP_BLUEPRINT_ARGB : SKY_ARGB);
    }
    prof_end(PROF_FILL);

    // The sky, behind everything: the title's drift, the ship's view,
    // and the frozen view a menu sits on. It times its own phases.
    int const view =
        (s_mode == MODE_TITLE || s_mode == MODE_WORLD) ? NISP_VIEW_COUNT - 1 : settings_view();
    int64_t const rast_us =
        sheet ? 0 : nisp_scene_render(target, &s_cam, s_flying ? &s_mission.ship : NULL, view, t);

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
    } else if (sheet) {
        nisp_builder_draw(fb, &s_builder);
    } else if (s_mode == MODE_WORLD) {
        // The hub is the screen; the only overlay it wants is the news line.
        if (showtime_now() < s_msg_until) hud_flash(fb, s_msg);
    } else {
        hud_stats_t const stats = {
            .fps     = s_fps,
            .slot    = s_slot,
            .rast_us = rast_us,
            .half    = half,
        };
        hud_draw(fb, &s_game, s_flying ? &s_mission : NULL, s_flying ? &s_readout : NULL, s_info, &stats);
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
