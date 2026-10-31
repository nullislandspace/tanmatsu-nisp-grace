#pragma once
// =====================================================================
//  Null Island Space Program  --  controls
// ---------------------------------------------------------------------
//  The ship's controls, as the engine's remappable bindings (se_bindings.h).
//  A binding is a raw BSP scancode, so it works both polled (held, for
//  steering) and matched against an event (an edge, for a single action).
//
//  LOOKING GOES THROUGH AN ABSTRACTION on purpose. input_look() returns a
//  delta per frame, and today it comes from the cursor keys being held.
//  When a mouse or the gyroscope arrives it comes from there instead and
//  NOT ONE CALL SITE CHANGES.
//
//  THE ACTIONS ARE THE BIT POSITIONS. Reordering them moves every saved
//  binding, so they may be appended to and never shuffled.
// =====================================================================

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    NISP_FORWARD = 0,  // pitch down: the nose follows the key
    NISP_BACK,         // pitch up
    NISP_LEFT,         // yaw left
    NISP_RIGHT,        // yaw right
    NISP_UP,           // throttle up
    NISP_DOWN,         // throttle down
    NISP_BOOST,        // the big step: fast throttle, big warp jumps
    NISP_LOOK_UP,
    NISP_LOOK_DOWN,
    NISP_LOOK_LEFT,
    NISP_LOOK_RIGHT,
    NISP_PAUSE,
    NISP_SAVE,
    NISP_INFO,
    NISP_SCREENSHOT,

    // Flight controls, appended 2026-11-01 (Part F2, plan 6.3). These are
    // the ones a rocket needs that a spaceship-with-a-joystick did not.
    NISP_ROLL_LEFT,
    NISP_ROLL_RIGHT,
    NISP_STAGE,     // drop the lowest live stage
    NISP_CUT,       // engines off, nothing discarded
    NISP_CHUTE,     // the capsule's parachute
    NISP_LEGS,      // the live stage's leg set
    NISP_SAS,       // cycle the stability-assist reference
    NISP_RCS,       // the RCS switch: off means no control torque at all
    NISP_WARP_UP,
    NISP_WARP_DOWN,
    NISP_MAP,       // TAB: flight view <-> map (plan 7.1)

    // Engineering's key (plan 4.1): the sheet's "activate this". ENTER,
    // because ENTER is what every other screen on the badge already means.
    NISP_CONFIRM,

    NISP_ACTION_COUNT
} nisp_action_t;

// The per-frame mask. 27 actions, so a uint32 with room to spare.
typedef uint32_t nisp_actions_t;

static inline bool act_held(nisp_actions_t m, nisp_action_t a) {
    return (m & ((nisp_actions_t)1u << a)) != 0;
}

// Register the control set with the engine. Call once at boot, BEFORE
// settings_init(): the settings file restores bindings onto whatever is
// registered, so an unregistered action silently keeps its default.
void input_init(void);

// Read the keyboard NOW and return the mask. Called once per frame.
nisp_actions_t input_sample(void);

// Actions that went DOWN between the last two samples. What a save key or
// a screenshot key wants -- holding one must not fire it sixty times.
nisp_actions_t input_pressed(void);

// The look delta for this frame, radians. `mask` is the frame's own
// sample. Positive dyaw turns right; positive dpitch looks DOWN.
void input_look(nisp_actions_t mask, float* dyaw, float* dpitch);

// The action's name, for the controls menu ("Forward", "Boost", ...).
char const* input_action_label(nisp_action_t a);

// The action's stable short id ("fwd", "boost"): what settings.txt keys
// its binding by. Never changes once shipped.
char const* input_action_id(nisp_action_t a);

// The scancode an action is bound to NOW, and the one it shipped with.
uint16_t input_key(nisp_action_t a);
uint16_t input_default_key(nisp_action_t a);

// Bind `a` to `sc`, saved to settings.txt. If another action already had
// `sc`, it takes `a`'s old key instead -- a SWAP, so two actions never
// share a key and no action is ever left with none.
void input_bind(nisp_action_t a, uint16_t sc);

// Every action back to its default, saved to settings.txt.
void input_reset_defaults(void);

// A keycap label for a scancode: "W", "Space", "Up", "F2". Written into
// `buf`, which is also returned.
char const* input_key_name(uint16_t sc, char* buf, int cap);
