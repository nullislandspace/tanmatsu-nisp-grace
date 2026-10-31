// =====================================================================
//  Null Island Space Program  --  Engineering, the blueprint designer
//  (see builder.h; the layout is Part B's diagram: 168 + 424 + 208)
// =====================================================================

#include "engineering/builder.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "engineering/icons.h"
#include "engineering/notes.h"
#include "rocket/parts.h"
#include "se_direct565.h"
#include "se_text.h"
#include "sim/bodies.h"

// --- The columns (Part B's diagram) -------------------------------------

#define COL_PARTS_X 0
#define COL_PARTS_W 168
#define COL_SHEET_X 168
#define COL_SHEET_W 424
#define COL_NOTES_X 592
#define COL_NOTES_W 208

#define HEADER_H 22
#define TITLE_H 16
#define BODY_Y (HEADER_H + TITLE_H + 6)  // where the columns start, below the titles

// The parameter editor slides the drawing right to make room for itself,
// rather than covering the rocket it is describing.
#define PARAM_SHIFT 90

// The sheet stops short of the bottom of the screen. The strip along the
// bottom is the actions' (Save / Name / Launch / Back), and a sheet that
// ran to the last pixel put a rocket's tail, its callouts and its title
// block on top of them (F-37).
#define SHEET_H 400

// --- The parameter rows (Part B2's ladders) -----------------------------

typedef enum {
    P_DIAMETER = 0,
    P_LENGTH,
    P_ENGINE,
    P_COUNT,
    P_RCS,
    P_BOOSTERS,
    P_BOOSTER_LEN,
    P_LEGS,
    P_REMOVE,
    P_ROW_COUNT
} param_row_t;

// The pod's ladder (D-51). It is a separate two-row ladder rather than two
// more entries in the stage one, because the capsule is not a stage: it has
// no diameter, no length, no engine and nothing to remove.
#define P_POD_RCS     0
#define P_POD_CHUTES  1
#define P_POD_ROW_COUNT 2

static bool cursor_is_pod(nisp_builder_t const* b) { return b->cursor_stage < 0; }

// How many rows the ladder has, which is the cursor's answer, not a constant.
static int param_row_count(nisp_builder_t const* b) {
    return cursor_is_pod(b) ? P_POD_ROW_COUNT : P_ROW_COUNT;
}

static char const* param_label(nisp_builder_t const* b, int row) {
    if (cursor_is_pod(b)) {
        if (row == P_POD_RCS) return "RCS pods";
        return "Parachutes";
    }
    switch (row) {
        case P_DIAMETER: return "Diameter";
        case P_LENGTH: return "Length";
        case P_ENGINE: return "Engine";
        case P_COUNT: return "Engines";
        case P_RCS: return "RCS pods";
        case P_BOOSTERS: return "Side boosters";
        case P_BOOSTER_LEN: return "Booster length";
        case P_LEGS: return "Landing legs";
        default: return "Remove stage";
    }
}

// The engines a stage type may be given. The solid Booster is a side
// booster and nothing else (parts.h), and a cargo section has no engine at
// all -- which is what makes it an adapter or a fuel-only tank.
static int engine_ladder_count(int kind) {
    if (kind == NISP_KIND_BOOSTER || kind == NISP_KIND_CARGO) return 1;
    return 4;
}

static int engine_ladder(int kind, int i) {
    if (kind == NISP_KIND_BOOSTER) return (i == 0) ? NISP_ENGINE_BOOSTER : -1;
    if (kind == NISP_KIND_CARGO) return (i == 0) ? NISP_ENGINE_NONE : -1;
    static int const LIST[4] = {NISP_ENGINE_FLEA, NISP_ENGINE_RELION, NISP_ENGINE_SKIPPER, NISP_ENGINE_MAINSAIL};
    if (i < 0 || i >= 4) return -1;
    return LIST[i];
}

static int engine_ladder_index(int kind, int engine) {
    for (int i = 0; i < engine_ladder_count(kind); i++) {
        if (engine_ladder(kind, i) == engine) return i;
    }
    return 0;
}

// Side boosters come in 0 / 2 / 4 (Part V2), which is a three-step ladder.
static int booster_ladder(int idx) {
    if (idx <= 0) return 0;
    return (idx == 1) ? 2 : 4;
}
static int booster_ladder_index(int n) {
    if (n >= 4) return 2;
    if (n >= 2) return 1;
    return 0;
}

static int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// --- Recompute: the one thing every edit ends with ----------------------

static void recompute(nisp_builder_t* b) {
    nisp_design_clamp(&b->design);
    nisp_stack_build(&b->design, &b->stack);
    nisp_budget_compute(&b->design, BODY_BOB, &b->budget);
    if (b->cursor_stage >= b->design.stage_count) b->cursor_stage = b->design.stage_count - 1;
    if (b->param_row >= param_row_count(b)) b->param_row = 0;
}

static void say(nisp_builder_t* b, char const* msg) {
    snprintf(b->msg, sizeof(b->msg), "%s", msg);
    b->msg_left = 3.0f;
}

// --- Opening ------------------------------------------------------------

void nisp_builder_open(nisp_builder_t* b, nisp_design_t const* design) {
    if (b == NULL) return;
    memset(b, 0, sizeof(*b));
    if (design != NULL) b->design = *design;
    else nisp_design_init(&b->design);
    nisp_design_clamp(&b->design);

    b->focus = NISP_FOCUS_SHEET;
    b->cursor_stage = 0;
    b->part_cursor = 0;
    b->action = 0;
    b->params_open = false;
    b->param_row = 0;
    recompute(b);
}

bool nisp_builder_can_launch(nisp_builder_t const* b) {
    return b != NULL && nisp_design_named(&b->design);
}

void nisp_builder_set_name(nisp_builder_t* b, char const* name) {
    if (b == NULL || name == NULL || name[0] == '\0') return;
    if (!nisp_design_name_ok(name)) {
        say(b, "Letters, digits, space, dash");
        return;
    }
    snprintf(b->design.name, sizeof(b->design.name), "%s", name);
    say(b, "Named");
}

void nisp_builder_say(nisp_builder_t* b, char const* msg) {
    if (b == NULL) return;
    say(b, msg);
}

// --- Editing ------------------------------------------------------------

static void edit_param(nisp_builder_t* b, int dir) {
    if (cursor_is_pod(b)) {
        if (b->param_row == P_POD_RCS) b->design.pod_rcs = !b->design.pod_rcs;
        else b->design.pod_chutes = clampi(b->design.pod_chutes + dir, 1, NISP_CHUTE_MAX);
        recompute(b);
        return;
    }
    if (b->cursor_stage >= b->design.stage_count) return;
    nisp_stage_t* s = &b->design.stage[b->cursor_stage];

    switch (b->param_row) {
        case P_DIAMETER: s->diameter = clampi(s->diameter + dir, 0, nisp_diameter_steps() - 1); break;
        case P_LENGTH: s->length = clampi(s->length + dir, 0, nisp_length_steps() - 1); break;
        case P_ENGINE:
            s->engine = engine_ladder(s->kind,
                                      clampi(engine_ladder_index(s->kind, s->engine) + dir, 0,
                                             engine_ladder_count(s->kind) - 1));
            break;
        case P_COUNT: s->engine_count = clampi(s->engine_count + dir, 1, nisp_engine_count_max()); break;
        case P_RCS: s->rcs = !s->rcs; break;
        case P_BOOSTERS:
            s->boosters = booster_ladder(clampi(booster_ladder_index(s->boosters) + dir, 0, 2));
            break;
        case P_BOOSTER_LEN: s->booster_length = clampi(s->booster_length + dir, 0, nisp_length_steps() - 1); break;
        case P_LEGS: s->legs = clampi(s->legs + dir, 0, NISP_LEGS_COUNT - 1); break;
        default: return;
    }
    recompute(b);
}

static void insert_stage(nisp_builder_t* b) {
    int const kind = nisp_legend_kind(b->part_cursor);
    if (!nisp_design_insert(&b->design, b->cursor_stage, kind)) {
        say(b, "Six stages is the most a rocket gets");
        return;
    }
    // The new stage is the one you are now looking at, and the editor opens
    // on it: adding a stage and setting it up is one motion.
    b->cursor_stage = (b->cursor_stage < 0) ? 0 : b->cursor_stage + 1;
    b->params_open = true;
    b->param_row = 0;
    recompute(b);
}

static void remove_stage(nisp_builder_t* b) {
    if (b->cursor_stage < 0) return;
    if (!nisp_design_remove(&b->design, b->cursor_stage)) {
        say(b, "A rocket needs a stage");
        return;
    }
    b->params_open = false;
    if (b->cursor_stage >= b->design.stage_count) b->cursor_stage = b->design.stage_count - 1;
    recompute(b);
}

static char const* const ACTIONS[NISP_BUILD_ACTION_COUNT] = {"Save", "Name", "Launch", "Back"};

static nisp_build_cmd_kind_t activate_action(nisp_builder_t* b) {
    switch (b->action) {
        case 0:
            if (!nisp_design_named(&b->design)) {
                say(b, "Name it before you can save it");
                return NISP_BUILD_NONE;
            }
            return NISP_BUILD_SAVE;
        case 1: return NISP_BUILD_NAME;
        case 2:
            // The user's rule, enforced here and visible in the title block
            // (D-11): an unnamed drawing cannot leave the ground.
            if (!nisp_design_named(&b->design)) {
                say(b, "Name it before you can launch it");
                return NISP_BUILD_NONE;
            }
            return NISP_BUILD_LAUNCH;
        default: return NISP_BUILD_LEAVE;
    }
}

nisp_build_cmd_kind_t nisp_builder_step(nisp_builder_t* b, nisp_actions_t pressed, float dt) {
    if (b == NULL) return NISP_BUILD_LEAVE;

    if (b->msg_left > 0.0f) {
        b->msg_left -= dt;
        if (b->msg_left <= 0.0f) b->msg[0] = '\0';
    }

    // TAB cycles the ring, from anywhere, and closes the parameter editor on
    // the way out -- a sub-mode must never trap the player.
    if (act_held(pressed, NISP_MAP)) {
        if (b->params_open) {
            b->params_open = false;
        } else {
            b->focus = (b->focus + 1) % NISP_FOCUS_COUNT;
        }
        return NISP_BUILD_NONE;
    }
    if (act_held(pressed, NISP_PAUSE)) {
        if (b->params_open) {
            b->params_open = false;
            return NISP_BUILD_NONE;
        }
        return NISP_BUILD_LEAVE;
    }

    // The parameter editor is a sub-mode of the sheet, and it owns the keys
    // while it is open (Part B2: up/down pick the row, left/right step the
    // ladder).
    if (b->params_open) {
        int const rows = param_row_count(b);
        if (act_held(pressed, NISP_LOOK_UP)) b->param_row = clampi(b->param_row - 1, 0, rows - 1);
        if (act_held(pressed, NISP_LOOK_DOWN)) b->param_row = clampi(b->param_row + 1, 0, rows - 1);
        if (act_held(pressed, NISP_LOOK_LEFT)) edit_param(b, -1);
        if (act_held(pressed, NISP_LOOK_RIGHT)) edit_param(b, +1);
        if (act_held(pressed, NISP_CONFIRM)) {
            if (b->param_row == P_REMOVE) remove_stage(b);
            else edit_param(b, +1);  // ENTER steps a ladder too: one key to try a size
        }
        return NISP_BUILD_NONE;
    }

    switch (b->focus) {
        case NISP_FOCUS_PARTS:
            if (act_held(pressed, NISP_LOOK_UP)) b->part_cursor = clampi(b->part_cursor - 1, 0, nisp_legend_count() - 1);
            if (act_held(pressed, NISP_LOOK_DOWN)) b->part_cursor = clampi(b->part_cursor + 1, 0, nisp_legend_count() - 1);
            if (act_held(pressed, NISP_CONFIRM)) insert_stage(b);
            break;

        case NISP_FOCUS_SHEET:
            if (act_held(pressed, NISP_LOOK_UP)) b->cursor_stage = clampi(b->cursor_stage - 1, -1, b->design.stage_count - 1);
            if (act_held(pressed, NISP_LOOK_DOWN)) b->cursor_stage = clampi(b->cursor_stage + 1, -1, b->design.stage_count - 1);
            // ENTER edits whatever the cursor is on, the pod included (D-51).
            // What the pod cannot do is be removed, which is why its ladder
            // has two rows and no "Remove" one.
            if (act_held(pressed, NISP_CONFIRM)) {
                b->params_open = true;
                b->param_row = 0;
            }
            break;

        case NISP_FOCUS_ACTIONS:
            if (act_held(pressed, NISP_LOOK_LEFT)) b->action = clampi(b->action - 1, 0, NISP_BUILD_ACTION_COUNT - 1);
            if (act_held(pressed, NISP_LOOK_RIGHT)) b->action = clampi(b->action + 1, 0, NISP_BUILD_ACTION_COUNT - 1);
            if (act_held(pressed, NISP_CONFIRM)) return activate_action(b);
            break;

        default: break;
    }
    return NISP_BUILD_NONE;
}

// --- Drawing ------------------------------------------------------------

static void text(pax_buf_t* fb, uint32_t col, float h, int x, int y, char const* s) {
    rendertext_draw(fb, col, NULL, h, (float)x, (float)y, s);
}
static int textw(float h, char const* s) {
    return (int)rendertext_size(NULL, h, s).x;
}

// A metre value with the trailing zeros cut off: 2.5 m, 4.375 m, 10 m. The
// sheet's own callouts and the parameter panel have to agree, and neither
// should read like a float.
static void fmt_m(char* buf, int cap, float v) {
    int const milli = (int)lroundf(v * 1000.0f);
    if (milli % 1000 == 0) snprintf(buf, (size_t)cap, "%.0f m", (float)(milli / 1000));
    else if (milli % 100 == 0) snprintf(buf, (size_t)cap, "%.1f m", (double)v);
    else if (milli % 10 == 0) snprintf(buf, (size_t)cap, "%.2f m", (double)v);
    else snprintf(buf, (size_t)cap, "%.3f m", (double)v);
}

static void draw_header(pax_buf_t* fb, nisp_builder_t* b) {
    text(fb, NISP_PENCIL_ARGB, 18.0f, 8, 3, "ENGINEERING");

    bool const named = nisp_design_named(&b->design);
    text(fb, named ? NISP_PENCIL_ARGB : NISP_MARK_ARGB, 18.0f, 150, 3, named ? b->design.name : "UNNAMED");

    char buf[64];
    snprintf(buf, sizeof(buf), "%d stage%s  %.1f t  %.0f m/s", b->budget.stage_count,
             b->budget.stage_count == 1 ? "" : "s", (double)b->budget.total_mass_t,
             (double)b->budget.total_delta_v_ms);
    text(fb, NISP_PENCIL_DIM_ARGB, 14.0f, COL_NOTES_X - 8 - textw(14.0f, buf), 5, buf);

    if (b->msg[0] != '\0') {
        text(fb, NISP_MARK_ARGB, 14.0f, COL_SHEET_X + 8, 5, b->msg);
    }
}

// The focus ring is visible where the ring is: the column whose name is
// marked is the one the keys are talking to. NOTES is never marked, because
// NOTES never has the keys (Part B2).
static void draw_column_titles(pax_buf_t* fb, nisp_builder_t* b) {
    int const y = HEADER_H + 2;
    text(fb, (b->focus == NISP_FOCUS_PARTS) ? NISP_MARK_ARGB : NISP_PENCIL_DIM_ARGB, TITLE_H, 8, y,
         (b->focus == NISP_FOCUS_PARTS) ? "> PARTS" : "PARTS");
    text(fb, (b->focus == NISP_FOCUS_SHEET) ? NISP_MARK_ARGB : NISP_PENCIL_DIM_ARGB, TITLE_H, COL_SHEET_X + 8, y,
         (b->focus == NISP_FOCUS_SHEET) ? "> BLUEPRINT" : "BLUEPRINT");
    text(fb, NISP_PENCIL_DIM_ARGB, TITLE_H, COL_NOTES_X + 8, y, "NOTES");
}

static void draw_parts(pax_buf_t* fb, nisp_builder_t* b) {
    int y = BODY_Y;

    // The pod, above the legend and dimmer than it: always there, and never
    // a row you can insert or remove (Part B3). Its two options are edited
    // on the sheet, where the cursor can be on it (D-51).
    nisp_icon_draw(fb, 8, y, NISP_KIND_CAPSULE, false);
    text(fb, NISP_PENCIL_DIM_ARGB, 13.0f, 8 + NISP_ICON_PX + 8, y + 4, "Command pod");
    text(fb, NISP_PENCIL_DIM_ARGB, 11.0f, 8 + NISP_ICON_PX + 8, y + 20, "enter edits it");
    y += 44;

    for (int i = 0; i < nisp_legend_count(); i++) {
        int const kind = nisp_legend_kind(i);
        bool const hot = (b->focus == NISP_FOCUS_PARTS && b->part_cursor == i);
        nisp_icon_draw(fb, 8, y, kind, hot);
        text(fb, hot ? NISP_MARK_ARGB : NISP_PENCIL_ARGB, 13.0f, 8 + NISP_ICON_PX + 8, y + 4,
             nisp_part_row(kind)->name);
        if (hot) text(fb, NISP_MARK_ARGB, 13.0f, 0, y + 4, ">");
        y += 40;
    }

    // Short lines, on purpose: this column is 168 px wide and a 12 px
    // Hershey line of more than ~26 characters walks into the sheet (F-37).
    y += 8;
    text(fb, NISP_PENCIL_DIM_ARGB, 12.0f, 8, y, "enter adds it below");
    text(fb, NISP_PENCIL_DIM_ARGB, 12.0f, 8, y + 16, "the cursor's stage");
    y += 44;

    text(fb, NISP_PENCIL_DIM_ARGB, 12.0f, 8, y, "TAB  focus");
    text(fb, NISP_PENCIL_DIM_ARGB, 12.0f, 8, y + 16, "ESC  back");
    text(fb, NISP_PENCIL_DIM_ARGB, 12.0f, 8, y + 32, "up/down  move");
    text(fb, NISP_PENCIL_DIM_ARGB, 12.0f, 8, y + 48, "left/right  change");
}

// Save / Name / Launch / Back, along the bottom of the sheet and to the
// left of the title block. They are a focus stop rather than a key each,
// because four more bindings would be four more things to remember and a
// ring you can see is cheaper than a chord you have to know.
static void draw_actions(pax_buf_t* fb, nisp_builder_t* b) {
    int const y = DISPLAY_LOG_H - 22;
    int x = COL_SHEET_X + 8;
    bool const named = nisp_design_named(&b->design);

    for (int i = 0; i < NISP_BUILD_ACTION_COUNT; i++) {
        bool const sel = (b->focus == NISP_FOCUS_ACTIONS && b->action == i);
        bool const refused = (i == 0 || i == 2) ? !named : false;
        uint32_t const col = sel ? NISP_MARK_ARGB : (refused ? NISP_PENCIL_DIM_ARGB : NISP_PENCIL_ARGB);

        if (sel) text(fb, NISP_MARK_ARGB, 15.0f, x - 12, y, ">");
        text(fb, col, 15.0f, x, y, ACTIONS[i]);
        x += textw(15.0f, ACTIONS[i]) + 22;
    }
}

static void param_value_pod(nisp_design_t const* d, int row, char* buf, int cap) {
    if (row == P_POD_RCS) {
        snprintf(buf, (size_t)cap, "%s", d->pod_rcs ? "yes" : "no");
        return;
    }
    snprintf(buf, (size_t)cap, "%d", clampi(d->pod_chutes, 1, NISP_CHUTE_MAX));
}

static void param_value(nisp_stage_t const* s, int row, char* buf, int cap) {
    switch (row) {
        case P_DIAMETER: fmt_m(buf, cap, nisp_diameter_m(s->diameter)); break;
        case P_LENGTH: fmt_m(buf, cap, nisp_length_m(s->length)); break;
        case P_ENGINE: snprintf(buf, (size_t)cap, "%s", nisp_engine_row(s->engine)->name); break;
        case P_COUNT: snprintf(buf, (size_t)cap, "%d", s->engine_count); break;
        case P_RCS: snprintf(buf, (size_t)cap, "%s", s->rcs ? "yes" : "no"); break;
        case P_BOOSTERS:
            if (s->boosters == 0) snprintf(buf, (size_t)cap, "none");
            else snprintf(buf, (size_t)cap, "%d", s->boosters);
            break;
        case P_BOOSTER_LEN: fmt_m(buf, cap, nisp_length_m(s->booster_length)); break;
        case P_LEGS: {
            nisp_legs_row_t const* legs = nisp_legs_row(s->legs);
            if (legs->count == 0) snprintf(buf, (size_t)cap, "none");
            else snprintf(buf, (size_t)cap, "%d legs", legs->count);
            break;
        }
        default: snprintf(buf, (size_t)cap, "remove it"); break;
    }
}

// The parameter editor: a darker card laid over the left margin of the
// sheet, with the drawing slid out of the way. Every value here is the
// number the sheet is drawing, and the sheet is redrawn on the same frame
// the ladder steps (Part B2).
static void draw_params(pax_buf_t* fb, nisp_builder_t* b) {
    bool const pod = cursor_is_pod(b);
    if (!pod && b->cursor_stage >= b->design.stage_count) return;
    nisp_stage_t const* const s = pod ? NULL : &b->design.stage[b->cursor_stage];
    int const rows = param_row_count(b);

    int const x = COL_SHEET_X + 8, y = BODY_Y + 4, w = 176;
    int const h = 24 + rows * 20 + 24;

    uint16_t* const px = (uint16_t*)pax_buf_get_pixels(fb);
    uint16_t const fill = direct_565_pack_for(fb, 0xff143f6fu);
    direct_565_tri(px, (float)x, (float)y, (float)(x + w), (float)y, (float)(x + w), (float)(y + h), fill);
    direct_565_tri(px, (float)x, (float)y, (float)(x + w), (float)(y + h), (float)x, (float)(y + h), fill);
    uint16_t const edge = direct_565_pack_for(fb, NISP_PENCIL_ARGB);
    direct_565_line(px, x, y, x + w, y, edge);
    direct_565_line(px, x, y + h - 1, x + w, y + h - 1, edge);
    direct_565_line(px, x, y, x, y + h - 1, edge);
    direct_565_line(px, x + w - 1, y, x + w - 1, y + h - 1, edge);

    char title[48];
    if (pod) snprintf(title, sizeof(title), "%s", "Command pod");
    else snprintf(title, sizeof(title), "%s  (stage %d)", nisp_part_row(s->kind)->name,
                  b->design.stage_count - b->cursor_stage);
    text(fb, NISP_PENCIL_ARGB, 14.0f, x + 8, y + 5, title);

    for (int row = 0; row < rows; row++) {
        int const ry = y + 26 + 20 * row;
        bool const sel = (row == b->param_row);
        uint32_t const col = sel ? NISP_MARK_ARGB : NISP_PENCIL_DIM_ARGB;
        if (sel) text(fb, NISP_MARK_ARGB, 13.0f, x - 12, ry, ">");
        text(fb, col, 13.0f, x + 8, ry, param_label(b, row));

        char val[32];
        if (pod) param_value_pod(&b->design, row, val, sizeof(val));
        else param_value(s, row, val, sizeof(val));
        text(fb, col, 13.0f, x + w - 8 - textw(13.0f, val), ry, val);
    }

    text(fb, NISP_PENCIL_DIM_ARGB, 11.0f, x + 8, y + h - 18, "left/right step   esc done");
}

void nisp_builder_draw(pax_buf_t* fb, nisp_builder_t* b) {
    if (fb == NULL || b == NULL) return;

    draw_header(fb, b);
    draw_column_titles(fb, b);
    draw_parts(fb, b);

    nisp_rect_t const sheet = {COL_SHEET_X, BODY_Y, COL_SHEET_W, SHEET_H};
    nisp_sheet_view_t view = nisp_sheet_fit(&b->stack, sheet, b->cursor_stage);
    if (b->params_open) view.axis_x += PARAM_SHIFT;
    nisp_sheet_draw(fb, sheet, &b->stack, view, b->design.name, b->cursor_stage, b->cursor_stage);

    nisp_rect_t const notes = {COL_NOTES_X, BODY_Y, COL_NOTES_W, DISPLAY_LOG_H - BODY_Y};
    nisp_notes_draw(fb, notes, &b->stack, &b->budget, b->cursor_stage);

    draw_actions(fb, b);
    if (b->params_open) draw_params(fb, b);
}
