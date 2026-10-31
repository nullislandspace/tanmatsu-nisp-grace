#pragma once
// =====================================================================
//  Null Island Space Program  --  the legend's part icons (Part B3)
// ---------------------------------------------------------------------
//  No PNGs, no sprite sheets, no asset pipeline: an icon is the part's own
//  geometry drawn at a fixed scale with the same pencil strokes the sheet
//  uses. The legend is a legend because it is drawn in the same language
//  as the drawing it belongs to.
//
//  The icons are 32x32 logical pixels, which is the smallest box that can
//  show a nose cone, a tank and a bell and still be read at arm's length.
// =====================================================================

#include <stdbool.h>

#include "pax_gfx.h"

#define NISP_ICON_PX 32

// The icon for one part row (NISP_KIND_*), into the 32x32 box whose top
// left is (x, y). `hot` is the cursor being on it.
void nisp_icon_draw(pax_buf_t* fb, int x, int y, int kind, bool hot);
