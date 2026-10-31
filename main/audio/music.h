#pragma once
// =====================================================================
//  Null Island Space Program  --  the music
// ---------------------------------------------------------------------
//  Every part of the game that has music plays a real recording off the
//  card: `music/menu/`, `music/engineering/` and `music/space/`, each
//  beside app.so, each looped. That is what the engine's MP3 source is
//  for (se_mp3.h), and it is a `music_source_t` like the procedural
//  generator, so swapping one for the other is one call and nothing in
//  the mixer or the settings changes.
//
//  WHY A DIRECTORY AND NOT A FILE. se_mp3 plays every .mp3 in the
//  directory it is handed -- there is no "play this one file" -- so a
//  track that belongs to one place lives in a directory of its own.
//  `music/menu/` is the menu's playlist, `music/engineering/` the
//  blueprint sheet's, `music/space/` the vacuum's; the rest of the
//  shipped music sits in `music/` as a pool a player can add to, and a
//  context whose own directory is missing falls back through it. The
//  Makefile puts them there (`make install`) and metadata.json lists
//  them for a card install.
//
//  ONE SOURCE, NEVER THREE. The mixer frees whatever it was given when
//  it is handed a new one (se_audio.h), and an MP3 decoder is a task
//  with a ring buffer behind it, so a context switch builds the next
//  source at the moment the previous one is gone rather than keeping one
//  per context waiting to be installed. That is also why the switch is a
//  fade through silence rather than a swap: the fade is what makes the
//  gap free rather than audible.
//
//  ATMOSPHERIC FLIGHT HAS NO MUSIC (the user's rule, plan 5.8). MUSIC_NONE
//  is a context like the others, and it is the one that leaves the mixer
//  with nothing: the air has its own noise, and a tune over it would be
//  a lie about where the ship is.
//
//  THE PROCEDURAL GENERATOR IS THE FALLBACK, not the alternative. A
//  card without the music -- a bad install, a card that predates it --
//  gets the synthwave generator rather than silence, because a game
//  with no music at all reads as broken rather than as quiet.
//
//  THE DECODER IS A TASK, and it is the engine's, not ours: se_mp3 runs
//  minimp3 in its own task on core 1, resampling into a ring buffer the
//  mixer drains. Nothing here touches the audio thread, and nothing here
//  reads the card during a frame.
//
//  THE LICENSES TRAVEL WITH THE MUSIC. Each track has a .txt beside it
//  saying who made it and under what; music_attribution() reads the one
//  for whatever is playing, and the Audio menu shows it.
// =====================================================================

#include <stdbool.h>
#include <stddef.h>

// The part of the game that wants music. MUSIC_MENU is what the title and
// every menu screen play; MUSIC_ENGINEERING the blueprint sheet; MUSIC_SPACE
// a ship above the atmosphere. MUSIC_NONE is the air: nothing plays.
typedef enum {
    MUSIC_NONE = 0,
    MUSIC_MENU,
    MUSIC_ENGINEERING,
    MUSIC_SPACE,
    MUSIC_CONTEXT_COUNT
} music_ctx_t;

// Ask for a context. Nothing audible happens in this call: it records the
// request, and music_update() crossfades to it over the next frames. Safe
// to call every frame with the same value.
void music_set_context(int ctx);
int  music_context(void);  // what was asked for

// The context's word, for a log line or a menu ("menu", "space").
char const* music_context_name(int ctx);

// Advance the fade. Once a frame, from the show clock, after whatever
// decided the context. It also re-applies the volume the Audio menu sets,
// so a volume change is heard without any menu having to know this module
// exists.
void music_update(float dt);

// Build the boot-time source and hand it to the mixer. Called once, after
// the mixer exists, in the menu context. True if there is music at all.
bool music_init(void);

// True when the music is a recording off the card rather than the
// generator.
bool music_is_mp3(void);

// Filename of the track now playing ("brittle_rille.mp3"), or "" when the
// generator is playing. Belongs to the engine's source; valid until the
// mixer replaces it, which nothing here does.
char const* music_now_playing(void);

// The first line of the .txt beside the playing track -- who made it and
// what it is called, verbatim, because that line is the attribution the
// license asks for and an app that edits it is an app that misquotes it.
// Empty string when there is no such file, and when the generator is
// playing. Read off the card once per track and kept, so a menu that asks
// every frame does not read the card every frame. Valid until the next
// track starts.
char const* music_attribution(void);

// Jump to the next track in the playlist. No-op for the generator.
void music_skip(void);
