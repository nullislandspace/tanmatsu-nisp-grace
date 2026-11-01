// =====================================================================
//  Null Island Space Program  --  the save slots (see save.h)
// =====================================================================

#include "save/save.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "common/paths.h"
#include "common/rng.h"
#include "common/vfs_compat.h"
#include "esp_log.h"
#include "se_nbt.h"
#include "se_save.h"

static char const TAG[] = "save";

// The engine's own spelling of a slot file (se_save.c, slot_path()). Not
// public API -- see save.h. Kept in one place so it is one line to fix.
static void slot_file(int slot, char* buf, size_t cap) {
    snprintf(buf, cap, NISP_SAVE_DIR "/save%d.bin", slot);
}

// --- The game's schema -----------------------------------------------------

static void save_serialize(NbtWriter* w, void const* data) {
    nisp_game_t const* g = data;
    nbt_write_compound(w, "game");
    nbt_write_string(w, "name", g->name);
    nbt_write_int32(w, "seed", (int32_t)g->seed);
    nbt_write_double(w, "x", g->pose.x);
    nbt_write_double(w, "y", g->pose.y);
    nbt_write_double(w, "z", g->pose.z);
    nbt_write_double(w, "yaw", g->pose.yaw);
    nbt_write_double(w, "pitch", g->pose.pitch);
    nbt_write_double(w, "flown", g->flown);

    // One nested compound per craft left out there, named flight0..flight3
    // after its slot in the archive. se_nbt.h has no list type, so the
    // index is in the tag name -- which is also what makes an archive from a
    // newer build readable by an older one, which simply skips it.
    for (int i = 0; i < NISP_ARCHIVE_MAX; i++) {
        nisp_flight_t const* f = &g->archive.flight[i];
        if (!f->used) continue;

        char tag[16];
        snprintf(tag, sizeof(tag), "flight%d", i);
        nbt_write_compound(w, tag);
        nbt_write_string(w, "vessel", f->vessel);
        nbt_write_string(w, "design", f->design);
        nbt_write_double(w, "t", f->t);
        nbt_write_int32(w, "body", f->body);
        nbt_write_int32(w, "state", f->state);
        nbt_write_int32(w, "flags",
                        (f->airborne ? 1 : 0) | (f->landed ? 2 : 0) | (f->has_orbit ? 4 : 0) |
                        (f->chute ? 8 : 0) | (f->ignited ? 16 : 0) | (f->rcs ? 32 : 0));
        nbt_write_double(w, "lat", f->lat_rad);
        nbt_write_double(w, "lon", f->lon_rad);

        if (f->has_orbit) {
            nbt_write_double(w, "mu", f->el.mu);
            nbt_write_double(w, "a", f->el.a);
            nbt_write_double(w, "e", f->el.e);
            nbt_write_double(w, "inc", f->el.inc);
            nbt_write_double(w, "lan", f->el.lan);
            nbt_write_double(w, "aop", f->el.aop);
            nbt_write_double(w, "m0", f->el.m0);
        }

        int attached = 0, legs = 0;
        for (int st = 0; st < NISP_DESIGN_STAGE_MAX; st++) {
            if (f->attached[st]) attached |= 1 << st;
            if (f->legs[st]) legs |= 1 << st;
            char name[16];
            snprintf(name, sizeof(name), "fill%d", st);
            nbt_write_double(w, name, f->fill[st]);
        }
        nbt_write_int32(w, "attached", attached);
        nbt_write_int32(w, "legs", legs);

        nbt_write_double(w, "throttle", f->throttle);
        nbt_write_int32(w, "sas", f->sas);
        nbt_write_double(w, "max_ap", f->max_ap_km);
        nbt_write_double(w, "max_speed", f->max_speed_ms);
        nbt_write_double(w, "max_heat", f->max_heat);
        nbt_write_int32(w, "stages", f->stages_dropped);
        nbt_write_int32(w, "lost", f->parts_lost);
        nbt_write_end(w);
    }

    nbt_write_end(w);
}

// One flightN compound. The tag names are the ones save_serialize wrote;
// anything unrecognised is skipped rather than trusted.
static void read_flight(NbtReader* r, nisp_flight_t* f) {
    char tag[32];
    int type;
    while ((type = nbt_read_tag(r, tag, sizeof(tag))) != NBT_END) {
        if (type < 0) return;
        if (type == NBT_STRING) {
            if (!strcmp(tag, "vessel")) nbt_read_string(r, f->vessel, (int)sizeof(f->vessel));
            else if (!strcmp(tag, "design")) nbt_read_string(r, f->design, (int)sizeof(f->design));
            else nbt_skip_payload(r, type);
            continue;
        }
        if (type == NBT_INT32) {
            int32_t const v = nbt_read_int32(r);
            if (!strcmp(tag, "body")) f->body = (int)v;
            else if (!strcmp(tag, "state")) f->state = (int)v;
            else if (!strcmp(tag, "flags")) {
                f->airborne = (v & 1) != 0;
                f->landed = (v & 2) != 0;
                f->has_orbit = (v & 4) != 0;
                f->chute = (v & 8) != 0;
                f->ignited = (v & 16) != 0;
                f->rcs = (v & 32) != 0;
            } else if (!strcmp(tag, "attached")) {
                for (int st = 0; st < NISP_DESIGN_STAGE_MAX; st++) f->attached[st] = (v & (1 << st)) != 0;
            } else if (!strcmp(tag, "legs")) {
                for (int st = 0; st < NISP_DESIGN_STAGE_MAX; st++) f->legs[st] = (v & (1 << st)) != 0;
            } else if (!strcmp(tag, "sas")) f->sas = (int)v;
            else if (!strcmp(tag, "stages")) f->stages_dropped = (int)v;
            else if (!strcmp(tag, "lost")) f->parts_lost = (int)v;
            continue;
        }
        if (type == NBT_DOUBLE) {
            double const v = nbt_read_double(r);
            if (!strcmp(tag, "t")) f->t = v;
            else if (!strcmp(tag, "lat")) f->lat_rad = (float)v;
            else if (!strcmp(tag, "lon")) f->lon_rad = (float)v;
            else if (!strcmp(tag, "mu")) f->el.mu = (float)v;
            else if (!strcmp(tag, "a")) f->el.a = v;
            else if (!strcmp(tag, "e")) f->el.e = v;
            else if (!strcmp(tag, "inc")) f->el.inc = v;
            else if (!strcmp(tag, "lan")) f->el.lan = v;
            else if (!strcmp(tag, "aop")) f->el.aop = v;
            else if (!strcmp(tag, "m0")) f->el.m0 = v;
            else if (!strcmp(tag, "throttle")) f->throttle = (float)v;
            else if (!strcmp(tag, "max_ap")) f->max_ap_km = (float)v;
            else if (!strcmp(tag, "max_speed")) f->max_speed_ms = (float)v;
            else if (!strcmp(tag, "max_heat")) f->max_heat = (float)v;
            else if (strncmp(tag, "fill", 4) == 0) {
                int const st = atoi(tag + 4);
                if (st >= 0 && st < NISP_DESIGN_STAGE_MAX) f->fill[st] = (float)v;
            }
            continue;
        }
        nbt_skip_payload(r, type);
    }

    // Only the conic itself is on the card (a, e, the three angles, the mean
    // anomaly and its epoch). The rest -- p, the apses, the period -- is what
    // nisp_elements_derive computes from those, and nisp_elements_to_state
    // reads them (sim/kepler.h).
    if (f->has_orbit) {
        f->el.body = f->body;
        nisp_elements_derive(&f->el);
    }
}

static void save_deserialize(NbtReader* r, void* data) {
    nisp_game_t* g = data;
    char name[32];
    int  type;
    while ((type = nbt_read_tag(r, name, sizeof(name))) != NBT_END) {
        if (type < 0) return;
        if (type != NBT_COMPOUND || strcmp(name, "game") != 0) {
            // The engine's se_peek block, and anything a newer build
            // added: skip it rather than trip over it.
            nbt_skip_payload(r, type);
            continue;
        }
        while ((type = nbt_read_tag(r, name, sizeof(name))) != NBT_END) {
            if (type < 0) return;
            if (type == NBT_STRING && strcmp(name, "name") == 0) {
                nbt_read_string(r, g->name, (int)sizeof(g->name));
            } else if (type == NBT_INT32 && strcmp(name, "seed") == 0) {
                g->seed = (uint32_t)nbt_read_int32(r);
            } else if (type == NBT_COMPOUND && strncmp(name, "flight", 6) == 0) {
                int const i = atoi(name + 6);
                if (i >= 0 && i < NISP_ARCHIVE_MAX) {
                    nisp_flight_t* f = &g->archive.flight[i];
                    memset(f, 0, sizeof(*f));
                    f->used = true;
                    read_flight(r, f);
                    if (g->archive.count <= i) g->archive.count = i + 1;
                } else {
                    nbt_skip_payload(r, type);
                }
            } else if (type == NBT_DOUBLE) {
                double const v = nbt_read_double(r);
                if (!strcmp(name, "x")) g->pose.x = (float)v;
                else if (!strcmp(name, "y")) g->pose.y = (float)v;
                else if (!strcmp(name, "z")) g->pose.z = (float)v;
                else if (!strcmp(name, "yaw")) g->pose.yaw = (float)v;
                else if (!strcmp(name, "pitch")) g->pose.pitch = (float)v;
                else if (!strcmp(name, "flown")) g->flown = (float)v;
            } else {
                nbt_skip_payload(r, type);
            }
        }
    }
}

// The peek's free-text line, composed here because this is where both
// halves of it are known. ASCII only: the engine's vector font draws
// what it has glyphs for (se_text.h).
static void slot_info(char* out, size_t cap, nisp_game_t const* g) {
    int const away = nisp_archive_count(&g->archive);
    if (away > 0) {
        snprintf(out, cap, "%s - %dm - %d craft away", g->name, (int)g->flown, away);
        return;
    }
    snprintf(out, cap, "%s - %dm", g->name, (int)g->flown);
}

// --- Slots -----------------------------------------------------------------

void save_init(void) {
    // se_save_init() makes its own directory, but only one level, and
    // /sd/nisp/saves has two.
    if (!nisp_mkdir_p(NISP_SAVE_DIR)) ESP_LOGW(TAG, "could not create %s", NISP_SAVE_DIR);
    se_save_init(&(se_save_config_t){
        .dir = NISP_SAVE_DIR, .game_name = "Null Island Space Program", .game_version = 1,
        .serialize = save_serialize, .deserialize = save_deserialize,
    });
}

bool save_slot_exists(int slot) {
    return se_save_slot_exists(slot) != 0;
}

int save_slot_peek(int slot, nisp_slot_t* out) {
    if (out == NULL) return -1;
    memset(out, 0, sizeof(*out));
    se_save_peek_t peek;
    if (se_save_peek(slot, &peek) != 0) return -1;
    out->exists = peek.exists;
    snprintf(out->info, sizeof(out->info), "%s", peek.info);
    out->timestamp = peek.timestamp;
    // The seed and the name are not in the peek, so a slot list has to open
    // the file to say what the player called it. A peek is already a file
    // open; this is a second one, a few times a session, which is what a
    // menu costs. The seed is read out alongside them only because the
    // field is there and an old slot has one (D-50).
    nisp_game_t g;
    if (save_load(slot, &g)) {
        out->seed = g.seed;
        snprintf(out->name, sizeof(out->name), "%s", g.name);
    }
    return 0;
}

bool save_new(int slot, char const* name, uint32_t seed, nisp_game_t* game) {
    if (game == NULL) return false;
    memset(game, 0, sizeof(*game));
    snprintf(game->name, sizeof(game->name), "%s", (name != NULL && name[0] != '\0') ? name : "Null Island");
    game->seed = seed;
    // The launch pad is the origin; the player starts on it, looking along
    // +z at the obelisk that is the reason they came.
    game->pose = (nisp_pose_t){0.0f, 1.7f, -12.0f, 0.0f, 0.08f};
    game->flown = 0.0f;

    char info[SE_SAVE_INFO_MAX];
    slot_info(info, sizeof(info), game);
    if (se_save_write_slot(slot, SE_SAVE_MANUAL, game, info) != 0) return false;
    rng_init(seed);
    return true;
}

bool save_load(int slot, nisp_game_t* game) {
    if (game == NULL) return false;
    memset(game, 0, sizeof(*game));
    if (se_save_load_slot(slot, game) != 0) return false;
    if (game->name[0] == '\0') snprintf(game->name, sizeof(game->name), "%s", "Null Island");
    if (game->pose.y == 0.0f) game->pose.y = 1.7f;
    rng_init(game->seed);
    return true;
}

bool save_write(int slot, nisp_game_t const* game) {
    if (game == NULL) return false;
    char info[SE_SAVE_INFO_MAX];
    slot_info(info, sizeof(info), game);
    return se_save_write_slot(slot, SE_SAVE_MANUAL, game, info) == 0;
}

bool save_rename(int slot, char const* name) {
    if (name == NULL || name[0] == '\0') return false;
    nisp_game_t g;
    if (!save_load(slot, &g)) return false;
    snprintf(g.name, sizeof(g.name), "%s", name);
    return save_write(slot, &g);
}

bool save_delete(int slot) {
    char path[128];
    slot_file(slot, path, sizeof(path));
    if (!nisp_remove(path)) {
        ESP_LOGW(TAG, "could not delete %s", path);
        return false;
    }
    return true;
}

char const* save_stamp(char* buf, int cap, int64_t timestamp) {
    if (timestamp <= 0) {
        snprintf(buf, (size_t)cap, "%s", "");
        return buf;
    }
    time_t const t = (time_t)timestamp;
    struct tm tm_local;
    if (localtime_r(&t, &tm_local) == NULL) {
        snprintf(buf, (size_t)cap, "%s", "");
        return buf;
    }
    strftime(buf, (size_t)cap, "%Y-%m-%d %H:%M", &tm_local);
    return buf;
}
