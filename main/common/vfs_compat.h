#pragma once
// =====================================================================
//  Null Island Space Program  --  the file helpers this app adds
// ---------------------------------------------------------------------
//  Nothing here reaches for FatFs any more. graceloader 2.8.5 rev 19
//  exports the whole POSIX file surface -- stdio, opendir / readdir /
//  readdir_r / closedir, remove / rename / unlink / access, mkdir /
//  rmdir, stat / fstat -- so the app speaks exactly one dialect
//  everywhere: VFS paths like /sd/nisp.
//
//  It used to not. remove/rename were missing (F-06) and so deletes and
//  renames went through f_unlink/f_rename, and a directory listing had
//  to guess the card's FatFs drive number (F-43). Both workarounds are
//  gone; the upstream template now carries the exports (see the
//  `upstream` remote). What is left is the one thing POSIX does not
//  have: mkdir -p.
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
