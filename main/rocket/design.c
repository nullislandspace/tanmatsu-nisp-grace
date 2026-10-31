// =====================================================================
//  Null Island Space Program  --  blueprints and their files (see design.h)
// =====================================================================

#include "rocket/design.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/paths.h"

#ifndef SM_HOST
#include "common/vfs_compat.h"  // remove / rename are not exported by graceloader (F-06)
#include "ff.h"                 // f_opendir / f_readdir: the card's directory listing
#else
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

// The host side of the same three file operations, so this file links into
// nispcheck without dragging common/vfs_compat (and FatFs) into it.
static bool nisp_mkdir_p(char const* path) { mkdir(path, 0777); struct stat st; return stat(path, &st) == 0; }
static bool nisp_remove(char const* path) { return remove(path) == 0; }
#endif

#define DESIGN_DIR NISP_DATA_DIR "/designs"
#define DESIGN_EXT ".nbp"
#define DESIGN_HEADER "nisp blueprint 1"

// --- The stage names the file uses -------------------------------------

static char const* const KIND_NAMES[NISP_KIND_COUNT] = {
    "capsule", "main", "upper", "booster", "cargo",
};

static char const* const ENGINE_NAMES[NISP_ENGINE_COUNT] = {
    "flea", "relion", "skipper", "mainsail", "booster", "none",
};

static char const* const LEGS_NAMES[NISP_LEGS_COUNT] = {"none", "3", "4", "6"};

static int name_index(char const* name, char const* const* table, int count, int fallback) {
    if (name == NULL) return fallback;
    for (int i = 0; i < count; i++) {
        if (!strcmp(name, table[i])) return i;
    }
    return fallback;
}

static int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// --- Construction ------------------------------------------------------

void nisp_design_init(nisp_design_t* d) {
    if (d == NULL) return;
    memset(d, 0, sizeof(*d));
    d->stage_count = 1;
    d->stage[0] = (nisp_stage_t){
        .kind = NISP_KIND_MAIN,
        .diameter = 2,                 // 2.5 m
        .length = 6,                  // 4.375 m
        .engine = NISP_ENGINE_SKIPPER,
        .engine_count = 1,
        .rcs = false,
        .boosters = 0,
        .booster_length = 4,          // 3.125 m
        .legs = NISP_LEGS_NONE,
    };
    d->pod_rcs = false;
    d->pod_chutes = 1;
    nisp_design_clamp(d);
}

// The rocket the game flies before the player has built one: the pad
// default (main.c) and the bench's ship. Two stages, RCS on both so it can
// be pointed, legs on the bottom stage so a landing is possible, and small
// enough that a test flight is cheap. Named, because a mission has to call
// the ship something.
void nisp_design_demo(nisp_design_t* d) {
    if (d == NULL) return;
    nisp_design_init(d);
    snprintf(d->name, sizeof(d->name), "%s", "DEMO");
    d->pod_rcs = true;  // a ship nobody has edited yet should be pointable
    d->stage_count = 2;
    d->stage[0] = (nisp_stage_t){
        .kind = NISP_KIND_UPPER,
        .diameter = 0,
        .length = 3,
        .engine = NISP_ENGINE_RELION,
        .engine_count = 1,
        .rcs = true,
    };
    d->stage[1] = (nisp_stage_t){
        .kind = NISP_KIND_MAIN,
        .diameter = 1,
        .length = 6,
        .engine = NISP_ENGINE_SKIPPER,
        .engine_count = 1,
        .rcs = true,
        .legs = NISP_LEGS_4,
    };
    nisp_design_clamp(d);
}

void nisp_design_clamp(nisp_design_t* d) {
    if (d == NULL) return;
    d->stage_count = clamp_int(d->stage_count, 1, NISP_DESIGN_STAGE_MAX);
    d->pod_chutes = clamp_int(d->pod_chutes, 1, NISP_CHUTE_MAX);
    for (int i = 0; i < d->stage_count; i++) {
        nisp_stage_t* s = &d->stage[i];
        if (s->kind == NISP_KIND_CAPSULE) s->kind = NISP_KIND_MAIN;
        s->kind = clamp_int(s->kind, 0, NISP_KIND_COUNT - 1);
        s->diameter = clamp_int(s->diameter, 0, nisp_diameter_steps() - 1);
        s->length = clamp_int(s->length, 0, nisp_length_steps() - 1);
        s->engine = clamp_int(s->engine, 0, NISP_ENGINE_COUNT - 1);
        s->engine_count = clamp_int(s->engine_count, 1, nisp_engine_count_max());
        if (!nisp_part_row(s->kind)->engines_allowed) s->engine = NISP_ENGINE_NONE;
        // Side boosters are solids, and only ever that one solid.
        s->boosters = (s->boosters >= 4) ? 4 : ((s->boosters >= 2) ? 2 : 0);
        s->booster_length = clamp_int(s->booster_length, 0, nisp_length_steps() - 1);
        s->legs = clamp_int(s->legs, 0, NISP_LEGS_COUNT - 1);
    }
    d->name[NISP_DESIGN_NAME_MAX] = '\0';
}

bool nisp_design_named(nisp_design_t const* d) {
    return d != NULL && d->name[0] != '\0';
}

bool nisp_design_name_ok(char const* name) {
    if (name == NULL) return false;
    int n = 0;
    while (name[n] != '\0') {
        char const c = name[n];
        bool const ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                        c == ' ' || c == '-' || c == '_';
        if (!ok) return false;
        n++;
        if (n > NISP_DESIGN_NAME_MAX) return false;
    }
    return n > 0;
}

bool nisp_design_insert(nisp_design_t* d, int at, int kind) {
    if (d == NULL || d->stage_count >= NISP_DESIGN_STAGE_MAX) return false;

    // "Below `at`" means what design.h says it means: the stage the cursor
    // is on stays where it is and the new one hangs under it. The capsule is
    // index -1, so directly under the pod is index 0, and an `at` past the
    // bottom appends. Getting this backwards is what made a new stage land
    // on top of the one you had just selected.
    int pos = (at < 0) ? 0 : at + 1;
    if (pos > d->stage_count) pos = d->stage_count;
    for (int i = d->stage_count; i > pos; i--) d->stage[i] = d->stage[i - 1];
    d->stage[pos] = (nisp_stage_t){
        .kind = kind,
        .diameter = 2,
        .length = 6,
        .engine = nisp_part_row(kind)->engine,
        .engine_count = 1,
        .rcs = false,
        .boosters = 0,
        .booster_length = 4,
        .legs = NISP_LEGS_NONE,
    };
    d->stage_count++;
    nisp_design_clamp(d);
    return true;
}

bool nisp_design_remove(nisp_design_t* d, int at) {
    if (d == NULL || d->stage_count <= 1) return false;
    if (at < 0 || at >= d->stage_count) return false;
    for (int i = at; i < d->stage_count - 1; i++) d->stage[i] = d->stage[i + 1];
    d->stage_count--;
    nisp_design_clamp(d);
    return true;
}

// --- Text --------------------------------------------------------------

int nisp_design_encode(nisp_design_t const* d, char* buf, int cap) {
    if (d == NULL || buf == NULL || cap <= 0) return -1;

    char tmp[1024];
    int n = snprintf(tmp, sizeof(tmp), "%s\nname=%s\nstages=%d\n", DESIGN_HEADER, d->name, d->stage_count);
    // Written clamped (D-51), because decode clamps: a design that never went
    // through nisp_design_clamp would otherwise write a number the file
    // cannot say back, and encode -> decode -> encode would not be the same
    // card twice.
    int const chutes = clamp_int(d->pod_chutes, 1, NISP_CHUTE_MAX);
    if (n >= 0 && n < (int)sizeof(tmp)) {
        n += snprintf(tmp + n, sizeof(tmp) - (size_t)n, "pod_rcs=%d pod_chutes=%d\n", d->pod_rcs ? 1 : 0,
                      chutes);
    }
    if (n < 0 || n >= (int)sizeof(tmp)) return -1;

    for (int i = 0; i < d->stage_count; i++) {
        nisp_stage_t const* s = &d->stage[i];
        int const m = snprintf(tmp + n, sizeof(tmp) - (size_t)n,
                               "stage=%d kind=%s diam=%d len=%d engine=%s count=%d rcs=%d "
                               "boosters=%d blen=%d legs=%s\n",
                               i, KIND_NAMES[s->kind], s->diameter, s->length, ENGINE_NAMES[s->engine],
                               s->engine_count, s->rcs ? 1 : 0, s->boosters, s->booster_length,
                               LEGS_NAMES[s->legs]);
        if (m < 0 || n + m >= (int)sizeof(tmp)) return -1;
        n += m;
    }

    if (n >= cap) return -1;
    memcpy(buf, tmp, (size_t)n);
    buf[n] = '\0';
    return n;
}

// A key is only a key when it starts a token: at the very start of the
// text, after a space, or at the start of a line. Without the newline case
// a key that begins a line -- which is every key that is not on a `stage=`
// line, `stages=` included -- answers nothing, and the pod's options were
// silently the defaults on every load (F-38).
static int token_start(char const* text, char const* p) {
    return p == text || p[-1] == ' ' || p[-1] == '\n';
}

static int field_int(char const* text, char const* key, int fallback) {
    char const* p = text;
    size_t const klen = strlen(key);
    while ((p = strstr(p, key)) != NULL) {
        // Only a real `key=` at the start of a token counts, so "len=" does
        // not answer a search for "blen=".
        if (token_start(text, p) && p[klen] == '=') {
            return atoi(p + klen + 1);
        }
        p += klen;
    }
    return fallback;
}

static char const* field_word(char const* text, char const* key, char* out, size_t cap) {
    char const* p = text;
    size_t const klen = strlen(key);
    while ((p = strstr(p, key)) != NULL) {
        if (token_start(text, p) && p[klen] == '=') {
            p += klen + 1;
            size_t i = 0;
            while (p[i] != '\0' && p[i] != ' ' && p[i] != '\n' && i + 1 < cap) {
                out[i] = p[i];
                i++;
            }
            out[i] = '\0';
            return out;
        }
        p += klen;
    }
    return NULL;
}

bool nisp_design_decode(char const* text, nisp_design_t* out) {
    if (text == NULL || out == NULL) return false;

    nisp_design_t d;
    memset(&d, 0, sizeof(d));

    char const* nl = strchr(text, '\n');
    size_t const head = nl ? (size_t)(nl - text) : strlen(text);
    if (head < strlen(DESIGN_HEADER) || memcmp(text, DESIGN_HEADER, strlen(DESIGN_HEADER)) != 0) return false;

    // The name is the rest of its line, spaces and all.
    char const* line = strstr(text, "name=");
    if (line != NULL && line < text + head + 4096) {
        line += 5;
        size_t i = 0;
        while (line[i] != '\0' && line[i] != '\n' && i < NISP_DESIGN_NAME_MAX) {
            d.name[i] = line[i];
            i++;
        }
        d.name[i] = '\0';
    }

    d.stage_count = field_int(text, "stages=", 0);
    // The pod's two knobs. Absent is the default, which is what a blueprint
    // written before they existed means (D-51).
    d.pod_rcs = field_int(text, "pod_rcs", 0) != 0;
    d.pod_chutes = field_int(text, "pod_chutes", 1);
    if (d.stage_count < 1) {
        // A hand-written file with `stage=` lines but no count is still a
        // rocket: count the markers.
        for (int i = 0; i < NISP_DESIGN_STAGE_MAX; i++) {
            char marker[16];
            snprintf(marker, sizeof(marker), "stage=%d ", i);
            if (strstr(text, marker) == NULL) break;
            d.stage_count = i + 1;
        }
    }
    if (d.stage_count < 1 || d.stage_count > NISP_DESIGN_STAGE_MAX) return false;

    for (int i = 0; i < d.stage_count; i++) {
        char marker[16];
        snprintf(marker, sizeof(marker), "stage=%d ", i);
        char const* start = strstr(text, marker);
        if (start == NULL) return false;
        char const* end = strchr(start, '\n');
        char linebuf[256];
        size_t const len = end ? (size_t)(end - start) : strlen(start);
        if (len >= sizeof(linebuf)) return false;
        memcpy(linebuf, start, len);
        linebuf[len] = '\0';

        char word[24];
        nisp_stage_t* s = &d.stage[i];
        s->kind = name_index(field_word(linebuf, "kind", word, sizeof(word)), KIND_NAMES, NISP_KIND_COUNT,
                             NISP_KIND_MAIN);
        s->diameter = field_int(linebuf, "diam", 2);
        s->length = field_int(linebuf, "len", 6);
        s->engine = name_index(field_word(linebuf, "engine", word, sizeof(word)), ENGINE_NAMES,
                               NISP_ENGINE_COUNT, nisp_part_row(s->kind)->engine);
        s->engine_count = field_int(linebuf, "count", 1);
        s->rcs = field_int(linebuf, "rcs", 0) != 0;
        s->boosters = field_int(linebuf, "boosters", 0);
        s->booster_length = field_int(linebuf, "blen", 4);
        s->legs = name_index(field_word(linebuf, "legs", word, sizeof(word)), LEGS_NAMES, NISP_LEGS_COUNT,
                             NISP_LEGS_NONE);
    }

    nisp_design_clamp(&d);
    *out = d;
    return true;
}

// --- The card ----------------------------------------------------------

static void design_path(char* out, size_t cap, char const* name) {
    snprintf(out, cap, DESIGN_DIR "/%s" DESIGN_EXT, name);
}

bool nisp_design_save(nisp_design_t const* d) {
    if (d == NULL || !nisp_design_named(d)) return false;
    if (!nisp_design_name_ok(d->name)) return false;

    nisp_mkdir_p(DESIGN_DIR);

    char text[1024];
    int const n = nisp_design_encode(d, text, sizeof(text));
    if (n < 0) return false;

    char path[192];
    design_path(path, sizeof(path), d->name);

    FILE* f = fopen(path, "wb");
    if (f == NULL) return false;
    size_t const wrote = fwrite(text, 1, (size_t)n, f);
    fclose(f);
    if (wrote != (size_t)n) {
        nisp_remove(path);
        return false;
    }
    return true;
}

static bool read_text(char const* path, char* buf, size_t cap) {
    FILE* f = fopen(path, "rb");
    if (f == NULL) return false;
    size_t const got = fread(buf, 1, cap - 1, f);
    fclose(f);
    if (got == 0) return false;
    buf[got] = '\0';
    return true;
}

bool nisp_design_load(char const* name, nisp_design_t* out) {
    if (!nisp_design_name_ok(name)) return false;
    char path[192], text[1024];
    design_path(path, sizeof(path), name);
    if (!read_text(path, text, sizeof(text))) return false;
    if (!nisp_design_decode(text, out)) return false;
    // The file is named for its name; a hand-edited file that disagrees is
    // loaded under the path it was found at, so the two cannot drift apart.
    snprintf(out->name, sizeof(out->name), "%s", name);
    return true;
}

bool nisp_design_exists(char const* name) {
    if (!nisp_design_name_ok(name)) return false;
    char path[192];
    design_path(path, sizeof(path), name);
    FILE* f = fopen(path, "rb");
    if (f == NULL) return false;
    fclose(f);
    return true;
}

bool nisp_design_rename(char const* from, char const* to) {
    if (!nisp_design_name_ok(from) || !nisp_design_name_ok(to)) return false;
    if (!strcmp(from, to)) return true;
    if (nisp_design_exists(to)) return false;

    nisp_mkdir_p(DESIGN_DIR);
    char a[192], b[192], text[1024];
    design_path(a, sizeof(a), from);
    design_path(b, sizeof(b), to);

    if (!read_text(a, text, sizeof(text))) return false;

    nisp_design_t d;
    if (!nisp_design_decode(text, &d)) return false;
    snprintf(d.name, sizeof(d.name), "%s", to);

    if (!nisp_design_save(&d)) return false;
    if (!nisp_remove(a)) return false;
    return true;
}

bool nisp_design_delete(char const* name) {
    if (!nisp_design_name_ok(name)) return false;
    char path[192];
    design_path(path, sizeof(path), name);
    return nisp_remove(path);
}

// The card's directory, spelled the way FatFs wants it (see vfs_compat.c:
// a VFS path and a FatFs path are not the same string). Fills `names` with
// up to `max` names and returns how many it found, capped at `max`.
#ifndef SM_HOST
static int list_designs(char names[][NISP_DESIGN_NAME_MAX + 1], int max) {
    char const* const spellings[] = {"/nisp/designs", "0:/nisp/designs", DESIGN_DIR};
    for (int s = 0; s < 3; s++) {
        FF_DIR dir;  // not DIR: graceloader exports FatFs, not the dirent wrappers (F-06)
        if (f_opendir(&dir, spellings[s]) != FR_OK) continue;
        int n = 0;
        FILINFO info;
        for (;;) {
            if (f_readdir(&dir, &info) != FR_OK || info.fname[0] == '\0') break;
            if (info.fattrib & AM_DIR) continue;
            char const* dot = strrchr(info.fname, '.');
            if (dot == NULL || strcmp(dot, DESIGN_EXT) != 0) continue;
            size_t const len = (size_t)(dot - info.fname);
            if (len == 0 || len > NISP_DESIGN_NAME_MAX) continue;
            if (n < max) {
                memcpy(names[n], info.fname, len);
                names[n][len] = '\0';
            }
            n++;
        }
        f_closedir(&dir);
        if (n > max) n = max;
        return n;
    }
    return 0;
}
#else
static int list_designs(char names[][NISP_DESIGN_NAME_MAX + 1], int max) {
    DIR* dir = opendir(DESIGN_DIR);
    if (dir == NULL) return 0;
    int n = 0;
    struct dirent* e;
    while ((e = readdir(dir)) != NULL) {
        char const* dot = strrchr(e->d_name, '.');
        if (dot == NULL || strcmp(dot, DESIGN_EXT) != 0) continue;
        size_t const len = (size_t)(dot - e->d_name);
        if (len == 0 || len > NISP_DESIGN_NAME_MAX) continue;
        if (n < max) {
            memcpy(names[n], e->d_name, len);
            names[n][len] = '\0';
        }
        n++;
    }
    closedir(dir);
    if (n > max) n = max;
    return n;
}
#endif

#define DESIGN_SCAN 16

static void sort_names(char names[][NISP_DESIGN_NAME_MAX + 1], int n) {
    for (int i = 1; i < n; i++) {
        char row[NISP_DESIGN_NAME_MAX + 1];
        memcpy(row, names[i], sizeof(row));
        int j = i - 1;
        while (j >= 0 && strcmp(names[j], row) > 0) {
            memcpy(names[j + 1], names[j], sizeof(row));
            j--;
        }
        memcpy(names[j + 1], row, sizeof(row));
    }
}

int nisp_design_list(char names[][NISP_DESIGN_NAME_MAX + 1], int max) {
    if (names == NULL || max <= 0) return 0;
    char all[DESIGN_SCAN][NISP_DESIGN_NAME_MAX + 1];
    int const n = list_designs(all, max < DESIGN_SCAN ? max : DESIGN_SCAN);
    for (int i = 0; i < n; i++) memcpy(names[i], all[i], sizeof(names[i]));
    sort_names(names, n);
    return n;
}

int nisp_design_count(void) {
    char all[DESIGN_SCAN][NISP_DESIGN_NAME_MAX + 1];
    return list_designs(all, DESIGN_SCAN);
}
