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

static char const TAG[] = "nisp_music";

// The menu's playlist, relative to the install directory (music.h).
#define MUSIC_MENU_DIR "music/menu"
// The rest of what ships, and what a player drops in.
#define MUSIC_POOL_DIR "music"

static music_source_t* s_src;
static bool            s_mp3;
static char            s_dir[160];  // the directory the playlist came from

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

bool music_init(void) {
    char const* const base = graceloader_get_install_basepath();

    if (base != NULL && base[0] != '\0') {
        s_src = try_mp3(base, MUSIC_MENU_DIR);
        if (s_src == NULL) {
            // No menu playlist -- maybe an install that predates the
            // layout. Anything else on the card is better than silence.
            s_src = try_mp3(base, MUSIC_POOL_DIR);
        }
    }

    if (s_src != NULL) {
        s_mp3 = true;
        ESP_LOGI(TAG, "menu music from %s (%d track(s), looped)", s_dir, se_mp3_track_count(s_src));
    } else {
        s_src = music_procedural_create(NULL, FALLBACK_SEED);
        s_mp3 = false;
        ESP_LOGW(TAG, "no music on the card; the procedural generator plays instead");
    }

    // The mixer owns it from here: it shuts the source down when it is
    // replaced or when the mixer shuts down, so nothing here frees it.
    audio_mixer_set_music(s_src);
    return s_src != NULL;
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
