// =====================================================================
//  Null Island Space Program  --  the music (see music.h)
// =====================================================================

#include "audio/music.h"

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "graceloader.h"
#include "se_audio.h"
#include "se_audio_source.h"
#include "se_mp3.h"
#include "se_music_procedural.h"
#include "common/rng.h"
#include "ui/settings.h"

static char const TAG[] = "nisp_music";

// One directory per context (music.h): se_mp3 has no "play this file", so
// a track that belongs to one place lives in a directory of its own.
#define MUSIC_MENU_DIR        "music/menu"
#define MUSIC_ENGINEERING_DIR "music/engineering"
#define MUSIC_SPACE_DIR       "music/space"
// The rest of what ships, and what a player drops in.
#define MUSIC_POOL_DIR "music"

// How long a context switch takes to fade out and back in, seconds. Short
// enough that a menu that changes its mind is not noticeable, long enough
// that the seam is a crossfade rather than a click.
#define FADE_S 0.25f

static music_source_t* s_src;
static bool            s_mp3;
static char            s_dir[160];  // the directory the playlist came from

static int  s_ctx = MUSIC_MENU;  // what was asked for
static int  s_live = -1;         // what the mixer has, -1 while it has nothing
static float s_gain;             // 0..1, the fade's current value

// The attribution for the track named in s_attr_track: read off the card
// once per track rather than once per call (music_attribution).
static char s_attr[96];
static char s_attr_track[64];

// The generator's seed, when it is the fallback: fixed, so a card with no
// music still has the same tune every boot.
#define FALLBACK_SEED 0x494c414eu  // "ILAE" -- Null Island, eventually.

static music_source_t* try_mp3(char const* base, char const* sub) {
    char dir[160];
    snprintf(dir, sizeof(dir), "%s/%s", base, sub);
    music_source_t* src = se_mp3_create(&(se_mp3_config_t){.dir = dir, .shuffle = false, .loop = true});
    if (src != NULL) snprintf(s_dir, sizeof(s_dir), "%s", dir);
    return src;
}

// Build the source for `ctx`: its own directory, then the pool, then the
// generator. The fallbacks are what keeps a card that predates this layout
// -- or a context whose track was deleted -- from being silent (music.h).
static music_source_t* build_source(int ctx) {
    char const* const base = graceloader_get_install_basepath();
    if (base == NULL || base[0] == '\0') return NULL;

    char const* dir = NULL;
    switch (ctx) {
        case MUSIC_MENU: dir = MUSIC_MENU_DIR; break;
        case MUSIC_ENGINEERING: dir = MUSIC_ENGINEERING_DIR; break;
        case MUSIC_SPACE: dir = MUSIC_SPACE_DIR; break;
        default: return NULL;
    }

    music_source_t* src = try_mp3(base, dir);
    if (src == NULL && strcmp(dir, MUSIC_MENU_DIR) != 0) src = try_mp3(base, MUSIC_MENU_DIR);
    if (src == NULL) src = try_mp3(base, MUSIC_POOL_DIR);
    return src;
}

// Install a source for s_ctx, or clear the slot for MUSIC_NONE. Whatever
// was there is the mixer's to shut down (se_audio.h), so nothing here
// retains it -- which is why this runs once per switch and not per frame.
static void install(int ctx) {
    if (ctx == MUSIC_NONE) {
        s_src = NULL;
        s_mp3 = false;
        audio_mixer_set_music(NULL);
        s_live = MUSIC_NONE;
        return;
    }

    s_src = build_source(ctx);
    if (s_src != NULL) {
        s_mp3 = true;
        ESP_LOGI(TAG, "%s music from %s (%d track(s), looped)", music_context_name(ctx), s_dir,
                 se_mp3_track_count(s_src));
    } else {
        s_src = music_procedural_create(NULL, FALLBACK_SEED);
        s_mp3 = false;
        if (s_src == NULL) {
            s_live = MUSIC_NONE;  // nothing to play: leave the slot empty
            audio_mixer_set_music(NULL);
            return;
        }
        ESP_LOGW(TAG, "no music on the card; the procedural generator plays instead");
    }

    audio_mixer_set_music(s_src);
    s_live = ctx;
}

bool music_init(void) {
    s_ctx  = MUSIC_MENU;
    s_gain = 0.0f;
    install(s_ctx);
    s_live = (s_src != NULL) ? s_ctx : MUSIC_NONE;
    return s_src != NULL;
}

char const* music_context_name(int ctx) {
    switch (ctx) {
        case MUSIC_NONE: return "silence";
        case MUSIC_MENU: return "menu";
        case MUSIC_ENGINEERING: return "engineering";
        case MUSIC_SPACE: return "space";
        default: return "?";
    }
}

void music_set_context(int ctx) {
    if (ctx < 0 || ctx >= MUSIC_CONTEXT_COUNT) ctx = MUSIC_NONE;
    if (ctx == s_ctx) return;
    s_ctx = ctx;
    // The fade does the rest: down to nothing, swap, back up.
}

int music_context(void) {
    return s_ctx;
}

// The mixer's volume is the setting times the fade, so a context switch and
// a slider are the same mechanism, and a slider moved while a menu is up is
// heard on the next frame without anyone having to be told.
static void apply_volume(void) {
    uint8_t const set = settings_music_volume();
    audio_mixer_set_music_volume((uint8_t)((float)set * s_gain));
}

void music_update(float dt) {
    if (dt <= 0.0f) return;
    float const step = dt / FADE_S;

    if (s_live != s_ctx) {
        // Fading out. The swap happens at silence, which is the only
        // moment a decoder can be torn down without being heard.
        if (s_gain > 0.0f) {
            s_gain -= step;
            if (s_gain < 0.0f) s_gain = 0.0f;
            apply_volume();
            return;
        }
        install(s_ctx);
        return;
    }

    if (s_ctx != MUSIC_NONE && s_gain < 1.0f) {
        s_gain += step;
        if (s_gain > 1.0f) s_gain = 1.0f;
        apply_volume();
        return;
    }

    apply_volume();
}

bool music_is_mp3(void) {
    return s_mp3;
}

char const* music_now_playing(void) {
    if (!s_mp3 || s_src == NULL) return "";
    char const* const name = se_mp3_track_name(s_src);
    return name != NULL ? name : "";
}

// Read once per track, not once per call: the Audio menu asks every frame
// it is up, and a card read inside a frame is exactly what this module
// promises not to do (music.h). The track name is the cache key, so
// skipping a track is what makes it read again.
char const* music_attribution(void) {
    char const* const name = music_now_playing();
    if (name[0] == '\0') return "";
    if (s_attr_track[0] != '\0' && !strcmp(s_attr_track, name)) return s_attr;

    snprintf(s_attr_track, sizeof(s_attr_track), "%s", name);
    s_attr[0] = '\0';

    // The license beside the track: brittle_rille.mp3 -> brittle_rille.txt
    char stem[64];
    snprintf(stem, sizeof(stem), "%s", name);
    char* const dot = strrchr(stem, '.');
    if (dot != NULL) *dot = '\0';

    char path[256];
    snprintf(path, sizeof(path), "%s/%s.txt", s_dir, stem);

    FILE* f = fopen(path, "rb");
    if (f == NULL) {
        ESP_LOGW(TAG, "no %s", path);
        return s_attr;
    }

    // fread, not fgets: the loader exports no fgets (make verify checks the
    // app against the symbols it actually has). A license is three short
    // lines, so reading the head of the file is reading the file.
    char raw[192];
    size_t const got = fread(raw, 1, sizeof(raw) - 1, f);
    fclose(f);
    if (got == 0) return s_attr;
    raw[got] = '\0';

    // The first line is the attribution -- what the piece is called and who
    // made it, verbatim, because that line is the attribution the license
    // asks for. The lines under it are the license itself, which is the
    // file's business and stays on the card.
    size_t const len  = strcspn(raw, "\r\n");
    size_t const copy = len < sizeof(s_attr) - 1 ? len : sizeof(s_attr) - 1;
    memcpy(s_attr, raw, copy);
    s_attr[copy] = '\0';

    return s_attr;
}

void music_skip(void) {
    if (s_mp3 && s_src != NULL) se_mp3_skip(s_src);
}
