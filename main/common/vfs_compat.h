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
