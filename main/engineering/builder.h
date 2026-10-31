#pragma once
// =====================================================================
//  Null Island Space Program  --  Engineering, the blueprint designer
// ---------------------------------------------------------------------
//  Part B2's keyboard-only editor. One working design in RAM, a stage
//  cursor, a legend of stage types, and parameter ladders on the stage the
//  cursor is on. There are no numeric fields anywhere: a ladder steps
//  through the sizes the part registry actually has, so an edit cannot
//  produce a diameter the physics would not recognise.
//
//  THE FOCUS RING IS THE WHOLE UI. TAB cycles PARTS -> SHEET -> ACTIONS.
//  NOTES never takes focus (Part B2): it is a read-out, not a control.
//  The parameter editor is a sub-mode of SHEET, opened with ENTER and
//  closed with ESC, so the ring stays three stops long.
//
//  FUEL IS THE SIZE OF THE STAGE (D-10): there is no fuel knob. Length x
//  diameter *is* the propellant load, which is why a ladder works on a
//  keyboard and why the drawing is honest -- the sheet is redrawn from the
//  same design on the frame the ladder steps.
//
//  EVERY EDIT RECOMPUTES THE STACK AND THE BUDGET, on the same frame. The
//  stack is what the sheet draws and the budget is what the notes print, so
//  the picture, the numbers and (later) the flight all read one source
//  (Part B7).
//
//  LAUNCH IS REFUSED WHILE THE DESIGN IS UNNAMED (D-11), and the title
//  block on the sheet is where the unnamed state is visible.
// =====================================================================

#include <stdbool.h>

#include "engineering/sheet.h"
#include "game/input.h"
#include "pax_gfx.h"
#include "rocket/budget.h"
#include "rocket/design.h"
#include "rocket/stack.h"

typedef enum {
    NISP_BUILD_NONE = 0,
    NISP_BUILD_LAUNCH,  // fly the working design (it is named: see can_launch)
    NISP_BUILD_SAVE,    // write it to /sd/nisp/designs/<name>.nbp
    NISP_BUILD_NAME,    // the name field wants the keyboard (the menu owns text entry)
    NISP_BUILD_LEAVE    // back to the Spaceport
} nisp_build_cmd_kind_t;

// The focus ring. NOTES is deliberately absent.
typedef enum {
    NISP_FOCUS_PARTS = 0,  // the legend: ENTER inserts that stage below the cursor
    NISP_FOCUS_SHEET,      // the drawing: the stage cursor, and ENTER for its parameters
    NISP_FOCUS_ACTIONS,    // Save / Name / Launch / Back along the bottom of the sheet
    NISP_FOCUS_COUNT
} nisp_focus_t;

#define NISP_BUILD_ACTION_COUNT 4

typedef struct {
    nisp_design_t design;
    nisp_stack_t  stack;
    nisp_budget_t budget;

    int  focus;
    int  cursor_stage;  // -1 is the capsule: always there, never editable
    int  part_cursor;   // the legend row
    int  action;        // NISP_BUILD_ACTION_COUNT items
    bool params_open;
    int  param_row;

    char  msg[64];      // the sheet's line of news ("Name it first")
    float msg_left;     // seconds of it left to show
} nisp_builder_t;

// Start editing `design` (NULL starts a fresh, unnamed one).
void nisp_builder_open(nisp_builder_t* b, nisp_design_t const* design);

// One frame of keys. `dt` only ages the news line.
nisp_build_cmd_kind_t nisp_builder_step(nisp_builder_t* b, nisp_actions_t pressed, float dt);

// The whole screen: legend, sheet, notes, action strip, parameter panel.
void nisp_builder_draw(pax_buf_t* fb, nisp_builder_t* b);

// The name the menu's text entry accepted (main.c round-trips it). An empty
// name is refused, which is the same rule the file format has.
void nisp_builder_set_name(nisp_builder_t* b, char const* name);

// A line of news on the sheet, for the things main.c does on the sheet's
// behalf (a Save that could not write to the card).
void nisp_builder_say(nisp_builder_t* b, char const* msg);

bool nisp_builder_can_launch(nisp_builder_t const* b);
