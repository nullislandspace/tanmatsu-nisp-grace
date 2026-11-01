// =====================================================================
//  Null Island Space Program  --  see vfs_compat.h
// =====================================================================

#include "common/vfs_compat.h"

#include <stdio.h>
#include <string.h>

#include <sys/stat.h>
#include <sys/types.h>

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

bool nisp_remove(char const* path) {
    if (path == NULL || *path == '\0') return false;
    if (remove(path) == 0) return true;
    // Already gone is success, not failure -- callers ask "is this file out
    // of the way?", and it is.
    struct stat st;
    return stat(path, &st) != 0;
}

bool nisp_rename(char const* from, char const* to) {
    if (from == NULL || to == NULL) return false;
    return rename(from, to) == 0;
}
