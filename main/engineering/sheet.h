#pragma once
// =====================================================================
//  Null Island Space Program  --  the blueprint sheet (Part B1, B5)
// ---------------------------------------------------------------------
//  One function draws a stack into a rectangle at a scale that fits it.
//  That is the whole renderer: the Engineering sheet, the design-list
//  thumbnail in the Spaceport and the pre-launch confirmation are the
//  same call with a different rectangle, so a rocket looks identical
//  everywhere it appears (Part B5).
//
//  IT IS DRAWN FROM THE STACK, not from the design. The parts come out of
//  rocket/stack -- the same rows the physics integrates -- so the drawing
//  cannot lie about a length, a diameter or where a separator is (Part B7).
//
//  2-D DIRECT RGB565 ONLY: no scene, no camera, no scene lines, no PPA
//  upscale (F-13). The paper is the frame's own clear; the grid, the
//  pencil strokes and the callouts are `direct_565_line` segments.
// =====================================================================

#include <stdbool.h>

#include "pax_gfx.h"
#include "rocket/design.h"
#include "rocket/stack.h"

// A logical-pixel rectangle. Everything in Engineering is one of these.
typedef struct {
    int x, y, w, h;
} nisp_rect_t;

// The drafting blue (Part B1) and the pencil. Off-white on purpose: pure
// white on blue reads as a wireframe.
#define NISP_BLUEPRINT_ARGB 0xff1d4e89u
#define NISP_PENCIL_ARGB 0xffe8eef5u
#define NISP_PENCIL_DIM_ARGB 0xff9fc0d8u
#define NISP_GRID_MINOR_ARGB 0xff2a5f97u
#define NISP_GRID_MAJOR_ARGB 0xff3d75adu
#define NISP_MARK_ARGB 0xfffff0c0u  // the stage the cursor is on

// The scale clamp. At the low end a 2 m capsule is 8 px, which is still a
// drawing; past the high end a short stack would fill the sheet with a
// single stage and the grid would stop meaning anything.
#define NISP_SHEET_MIN_PX_PER_M 4.0f
#define NISP_SHEET_MAX_PX_PER_M 26.0f

typedef struct {
    float px_per_m;    // the scale actually used
    float total_m;     // the whole stack, nose to tail
    float view_top_m;  // the metre along the axis at the top of the rectangle
    float shown_m;     // how many metres the rectangle shows
    int   axis_x;      // the stack axis, logical px
    int   top_y;       // the y of view_top_m
} nisp_sheet_view_t;

// The scale that fits `s` into `r`, clamped to the range above, and the
// scroll that keeps `cursor_stage` (-1 for the capsule) inside the sheet.
nisp_sheet_view_t nisp_sheet_fit(nisp_stack_t const* s, nisp_rect_t r, int cursor_stage);

// Paper grid, the rocket, its dimension callouts and the title block.
// `name` goes in the title block; an empty name shows as UNNAMED (D-11).
// `callout_stage` (-1 for none) is the stage whose dimensions get arrows.
void nisp_sheet_draw(pax_buf_t* fb, nisp_rect_t r, nisp_stack_t const* s, nisp_sheet_view_t view,
                     char const* name, int cursor_stage, int callout_stage);

// The same drawing without the callouts or the title block, for a
// thumbnail (Part B5).
void nisp_sheet_thumb(pax_buf_t* fb, nisp_rect_t r, nisp_design_t const* d);
