#pragma once
// =====================================================================
//  Null Island Space Program  --  the music
// ---------------------------------------------------------------------
//  The menu plays a real recording: `music/menu/` beside app.so on the
//  card, looped. That is what the engine's MP3 source is for
//  (se_mp3.h), and it is a `music_source_t` like the procedural
//  generator, so swapping one for the other is one call and nothing in
//  the mixer or the settings changes.
//
//  WHY A DIRECTORY AND NOT A FILE. se_mp3 plays every .mp3 in the
//  directory it is handed -- there is no "play this one file" -- so a
//  track that belongs to one place lives in a directory of its own.
//  `music/menu/` is the menu's playlist; the rest of the shipped music
//  sits in `music/` until something wants it, and a player can drop
//  their own .mp3 into either. The Makefile puts them there
//  (`make install`) and metadata.json lists them for a card install.
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

// Pick a source and hand it to the mixer: the shipped MP3 playlist if the
// card has one, the procedural generator otherwise. Called once at boot,
// after the mixer exists. True if there is music at all.
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
