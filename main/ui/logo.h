#pragma once
// =====================================================================
//  Null Island Space Program  --  the logo
// ---------------------------------------------------------------------
//  The ball, drawn by hand and shipped as a PNG next to app.so:
//  `make install` uploads nullislandlogo_ball.png into the app's install
//  directory and metadata.json lists it as an asset, so an app repository
//  carries it the same way. It is NOT compiled into the binary -- the
//  artwork is data, and data belongs on the card where it can be replaced
//  without rebuilding anything.
//
//  Loaded once, at boot, from graceloader_get_install_basepath() (which
//  directory that is belongs to the launcher, not to us), and RESIZED at
//  the same time: the artwork is 940x937 and the title wants it 340 tall,
//  and resampling it every frame is what made the title screen run at a
//  crawl. So logo_load() scales it down to LOGO_DRAW_H and throws the
//  3.5 MB original away. One resample per boot instead of one per frame.
//
//  What it keeps afterwards is packed for the blit, not for drawing: the
//  resampled artwork becomes the framebuffer's own RGB565 halfwords laid
//  out column by column -- which is the order the raw buffer keeps a
//  column in -- with each column's transparent rows cut off at both ends
//  and its fully opaque middle recorded as a run. logo_draw() then walks
//  341 contiguous sweeps of the framebuffer: stores for the opaque
//  middle, a blend for the few rows the downscale left soft, and nothing
//  at all for the corners. pax_draw_image() spends four function-pointer
//  calls on every one of 116k pixels whether or not that pixel is even
//  part of the ball; that is what the 59 ms title screen was.
//
//  Alpha survives the packing because the ball is round: the corners of
//  the artwork are transparent, and the "_op" drawing variants would
//  paint them opaque, which on a starfield is a black square. On a
//  display that is not the RGB565 / PAX_O_ROT_CW layout this assumes,
//  logo_load() keeps the ARGB8888 layer and logo_draw() falls back to
//  pax_draw_image().
//
//  The downscale averages each destination pixel over its footprint in the
//  source, weighting the colours by their alpha. Averaging them straight
//  instead would mix the RGB of the fully transparent pixels -- which a
//  PNG stores as black whatever the artwork looks like -- into the edge of
//  the ball and give it a dark rim.
//
//  If the file is not there -- a card that predates it, an install that
//  went wrong -- everything else about the title still works, and the
//  wordmark stands on its own.
// =====================================================================

#include <stdbool.h>

#include "pax_gfx.h"

// The name it is installed under beside app.so, which is the name it is
// kept under in the repository, which is the name the Makefile uploads and
// metadata.json lists. Here rather than in logo.c so main.c can name it in
// the log line that says it is missing.
#define LOGO_FILE "nullislandlogo_ball.png"

// How tall the ball is on the title screen, in logical pixels. The layer
// is built at exactly this size, so drawing it is a blit and never a
// resample. There is no runtime choice to make: a different size means a
// different layer, and the layer is built in logo_load().
#define LOGO_DRAW_H 340

// Decode the shipped logo. True if it is now drawable. Safe to call more
// than once; it does not decode twice.
bool logo_load(void);

// Free the decoded image. Called on shutdown; the title keeps it otherwise.
void logo_free(void);

bool logo_ready(void);

// Blit it centred on (cx, cy) at the size it was built at, snapped to the
// pixel grid. A no-op when there is no logo.
void logo_draw(pax_buf_t* fb, float cx, float cy);
