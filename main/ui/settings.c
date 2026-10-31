// =====================================================================
//  Null Island Space Program  --  the game's own settings (see settings.h)
// =====================================================================

#include "ui/settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "common/paths.h"
#include "common/vfs_compat.h"
#include "esp_log.h"
#include "game/input.h"

static char const TAG[] = "settings";

#define KEY_PREFIX "key."

static int     s_view = 1;  // medium
static bool    s_half = false;
static bool    s_music = true;
static bool    s_effects = true;
static uint8_t s_music_vol = 60;
static uint8_t s_effects_vol = 80;

static char s_path[192];
static char s_tmp[192];

// True while the file is being read. A line that restores a binding calls
// input_bind(), which calls back in here to save -- so loading would
// rewrite the file once per key. It is suppressed during the read and
// written once afterwards instead.
static bool s_loading;

static uint8_t clamp_pct(int v) {
    if (v < 0) return 0;
    if (v > 100) return 100;
    return (uint8_t)v;
}

static void apply_line(char* line) {
    if (line[0] == '#' || line[0] == '\0') return;
    char* eq = strchr(line, '=');
    if (eq == NULL) return;
    *eq = '\0';
    char const* key = line;
    char* val = eq + 1;

    if (!strcmp(key, "view")) {
        int const v = atoi(val);
        if (v >= 0 && v < NISP_VIEW_COUNT) s_view = v;
    } else if (!strcmp(key, "half_res")) {
        s_half = atoi(val) != 0;
    } else if (!strcmp(key, "music")) {
        s_music = atoi(val) != 0;
    } else if (!strcmp(key, "effects")) {
        s_effects = atoi(val) != 0;
    } else if (!strcmp(key, "music_volume")) {
        s_music_vol = clamp_pct(atoi(val));
    } else if (!strcmp(key, "effects_volume")) {
        s_effects_vol = clamp_pct(atoi(val));
    } else if (!strncmp(key, KEY_PREFIX, sizeof(KEY_PREFIX) - 1)) {
        // A binding, by the action's stable id. Unknown ids are ignored,
        // so a file written by a newer build is still readable by this one.
        char const* id = key + (sizeof(KEY_PREFIX) - 1);
        for (int a = 0; a < NISP_ACTION_COUNT; a++) {
            if (strcmp(input_action_id((nisp_action_t)a), id) != 0) continue;
            input_bind((nisp_action_t)a, (uint16_t)strtoul(val, NULL, 0));
            return;
        }
    }
    // Anything else is from a newer build, or a typo: ignored.
}

// The whole file in one read: the stdio graceloader exports has fread but
// no fgets.
static bool read_file(char const* path) {
    FILE* f = fopen(path, "rb");
    if (f == NULL) return false;
    static char buf[2048];
    size_t const n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    buf[n] = '\0';
    char* line = buf;
    while (*line != '\0') {
        char* end = line + strcspn(line, "\r\n");
        char const was = *end;
        *end = '\0';
        apply_line(line);
        line = was == '\0' ? end : end + 1;
    }
    return true;
}

void settings_save(void) {
    if (s_loading) return;
    if (s_path[0] == '\0') return;
    FILE* f = fopen(s_tmp, "wb");
    if (f == NULL) {
        ESP_LOGW(TAG, "could not write %s", s_tmp);
        return;
    }
    fputs("# Null Island Space Program settings. Volume and brightness are the badge's own\n"
          "# and live with the launcher. Keys are BSP scancodes; delete a line to get its\n"
          "# default. The livestream is not here on purpose -- see ui/settings.h.\n",
          f);
    fprintf(f, "view=%d\nhalf_res=%d\nmusic=%d\neffects=%d\nmusic_volume=%u\neffects_volume=%u\n", s_view, s_half,
            s_music, s_effects, (unsigned)s_music_vol, (unsigned)s_effects_vol);
    for (int a = 0; a < NISP_ACTION_COUNT; a++) {
        fprintf(f, KEY_PREFIX "%s=0x%04x\n", input_action_id((nisp_action_t)a), (unsigned)input_key((nisp_action_t)a));
    }
    bool const ok = fflush(f) == 0;
    fclose(f);
    if (!ok) {
        ESP_LOGW(TAG, "writing %s failed", s_tmp);
        return;
    }
    // FAT will not rename over an existing file. Between the remove and
    // the rename only the .tmp exists, which settings_init also reads.
    nisp_remove(s_path);
    if (!nisp_rename(s_tmp, s_path)) ESP_LOGW(TAG, "could not move %s into place", s_tmp);
}

void settings_init(void) {
    snprintf(s_path, sizeof(s_path), "%s", NISP_SETTINGS);
    snprintf(s_tmp, sizeof(s_tmp), NISP_DATA_DIR "/settings.tmp");

    if (!nisp_mkdir_p(NISP_DATA_DIR)) ESP_LOGW(TAG, "could not create %s", NISP_DATA_DIR);

    s_loading = true;
    if (read_file(s_path)) {
        ESP_LOGI(TAG, "read %s", s_path);
    } else if (read_file(s_tmp)) {
        // A previous write died between the two names.
        ESP_LOGW(TAG, "%s was half-written; recovered %s", s_path, s_tmp);
    } else {
        ESP_LOGI(TAG, "no %s yet; using defaults", s_path);
    }
    s_loading = false;
    // Once, whatever was read or not read: it writes back what the file
    // now has to be, including any binding the loader applied.
    settings_save();
}

int settings_view(void) {
    return s_view;
}
void settings_set_view(int preset) {
    if (preset < 0 || preset >= NISP_VIEW_COUNT || preset == s_view) return;
    s_view = preset;
    settings_save();
}

bool settings_half_res(void) {
    return s_half;
}
void settings_set_half_res(bool on) {
    if (on == s_half) return;
    s_half = on;
    settings_save();
}

bool settings_music(void) {
    return s_music;
}
void settings_set_music(bool on) {
    if (on == s_music) return;
    s_music = on;
    settings_save();
}
uint8_t settings_music_volume(void) {
    return s_music_vol;
}
void settings_set_music_volume(uint8_t percent) {
    uint8_t const v = clamp_pct(percent);
    if (v == s_music_vol) return;
    s_music_vol = v;
    settings_save();
}

bool settings_effects(void) {
    return s_effects;
}
void settings_set_effects(bool on) {
    if (on == s_effects) return;
    s_effects = on;
    settings_save();
}
uint8_t settings_effects_volume(void) {
    return s_effects_vol;
}
void settings_set_effects_volume(uint8_t percent) {
    uint8_t const v = clamp_pct(percent);
    if (v == s_effects_vol) return;
    s_effects_vol = v;
    settings_save();
}
