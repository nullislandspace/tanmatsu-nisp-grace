// =====================================================================
//  Null Island Space Program  --  the save slots (see save.h)
// =====================================================================

#include "save/save.h"

#include <stdio.h>
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
    nbt_write_end(w);
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
