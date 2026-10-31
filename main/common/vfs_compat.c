// =====================================================================
//  Null Island Space Program  --  see vfs_compat.h
// =====================================================================

#include "common/vfs_compat.h"

#include <stdio.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>

#include "ff.h"

bool nisp_mkdir_p(char const* path) {
    if (path == NULL || *path == '\0') return false;

    char buf[192];
    size_t const n = strlen(path);
    if (n >= sizeof(buf)) return false;
    memcpy(buf, path, n + 1);

    // Walk the separators, creating as we go. An existing directory is
    // success, not failure -- which is the whole point of the _p.
    for (char* p = buf + 1; *p != '\0'; p++) {
        if (*p != '/') continue;
        *p = '\0';
        mkdir(buf, 0777);
        *p = '/';
    }
    mkdir(buf, 0777);

    struct stat st;
    return stat(buf, &st) == 0;
}

// "/sd/nisp/x" is a VFS path; FatFs wants it volume-relative. Try the
// plausible spellings and keep whichever the filesystem accepts.
static void fat_candidate(char* out, size_t cap, char const* vfs, int which) {
    char const* rel = vfs;
    if (strncmp(vfs, "/sd", 3) == 0) {
        rel = vfs + 3;
    } else if (strncmp(vfs, "/int", 4) == 0) {
        rel = vfs + 4;
    }
    if (*rel == '\0') rel = "/";

    switch (which) {
        case 0: snprintf(out, cap, "%s", rel); break;
        case 1: snprintf(out, cap, "0:%s", rel); break;
        case 2: snprintf(out, cap, "1:%s", rel); break;
        default: snprintf(out, cap, "%s", vfs); break;
    }
}

#define FAT_TRIES 4

bool nisp_remove(char const* path) {
    char cand[160];
    for (int i = 0; i < FAT_TRIES; i++) {
        fat_candidate(cand, sizeof(cand), path, i);
        if (f_unlink(cand) == FR_OK) return true;
    }
    // ONLY FR_OK COUNTS. A candidate spelling that names the wrong volume
    // answers FR_NO_FILE too, and taking that for success would report a
    // deleted save that is still on the card.
    struct stat st;
    return stat(path, &st) != 0;
}

bool nisp_rename(char const* from, char const* to) {
    char a[160], b[160];
    for (int i = 0; i < FAT_TRIES; i++) {
        fat_candidate(a, sizeof(a), from, i);
        fat_candidate(b, sizeof(b), to, i);
        // f_rename's destination is relative to the source's volume, so
        // strip a drive prefix from it if the spelling added one.
        char const* dst = b;
        if (b[0] != '\0' && b[1] == ':') dst = b + 2;
        if (f_rename(a, dst) == FR_OK) return true;
    }
    return false;
}
