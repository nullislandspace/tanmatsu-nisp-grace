#pragma once
// =====================================================================
//  Null Island Space Program  --  the post-it notes (Part B4)
// ---------------------------------------------------------------------
//  The verdict column. Three to five stuck-on cards on the right of the
//  sheet: the selected stage, the whole rocket, and a warning card that
//  only exists when something is wrong.
//
//  THE NUMBERS ARE NOT COMPUTED HERE. The caller keeps a nisp_budget_t
//  recomputed from rocket/budget on every edit (Part B4: on every edit,
//  not on a timer and not on save) and hands it over; this file only
//  draws it as paper.
//
//  WHAT MAY BE PRINTED, and what may never be (D-37): a part's *rating*
//  is data -- a leg set's touchdown speed, an engine's thrust, a stage's
//  delta-v. What the assembled rocket does with those ratings on a
//  trajectory the player is flying is not data, so there is no stability
//  card, no heat card and no survivability card here. Engineering says
//  what your parts are rated for; the HUD says what they are doing.
// =====================================================================

#include "engineering/sheet.h"
#include "pax_gfx.h"
#include "rocket/budget.h"
#include "rocket/stack.h"

// `cursor_stage` is the sheet's stage cursor (-1 for the capsule), which
// decides which stage the first card is about.
void nisp_notes_draw(pax_buf_t* fb, nisp_rect_t r, nisp_stack_t const* s, nisp_budget_t const* b,
                     int cursor_stage);
