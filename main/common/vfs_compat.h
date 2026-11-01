#pragma once
// =====================================================================
//  Null Island Space Program  --  the file operations graceloader does
//                                  not export
// ---------------------------------------------------------------------
//  stdio works: fopen, fread, fwrite, fprintf, fflush, fclose, mkdir,
//  stat, rmdir. These are NOT exported (checked against fakelib/liball.so):
//
//      remove   rename
//
//  An app that calls them links fine and then fails to LOAD, because the
//  symbol cannot be resolved at load time. So deleting a save slot and
//  moving settings.tmp into place go through FatFs (f_unlink, f_rename),
//  which IS exported -- the same finding SynthMiner reached (F-06).
//
//  FatFs paths are volume-relative and carry no VFS mount point, so the
//  "/sd/nisp/..." a caller has is not a FatFs path. Rather than hard-code
//  a mapping that would break silently if the mount changed, the
//  plausible spellings are tried and whichever works is kept.
// =====================================================================

#include <stdbool.h>

// Delete a FILE. True if it is gone afterwards (including if it was
// never there).
bool nisp_remove(char const* path);

// Rename / move a file. True on success.
bool nisp_rename(char const* from, char const* to);

// Create a directory and every missing parent. True if it exists
// afterwards.
bool nisp_mkdir_p(char const* path);

// --- The FatFs spelling of a VFS path ------------------------------------

// A directory listing goes through FatFs (f_opendir), and FatFs wants a
// volume-relative path with a drive prefix. graceloader mounts /int first and
// /sd second, IDF hands out drive numbers in mount order, and so the card is
// normally "1:" -- which is not the spelling anybody would guess (F-43).
// Every caller that asks FatFs for something tries all of them and keeps the
// one that answers, so this is the one place that knows the candidates.
#define NISP_FAT_TRIES 4

// `which` is 0..NISP_FAT_TRIES-1. Writes the candidate into `out` and returns
// it, so a caller can loop over the candidates without repeating the rules.
char const* nisp_fat_path(char* out, size_t cap, char const* vfs, int which);

