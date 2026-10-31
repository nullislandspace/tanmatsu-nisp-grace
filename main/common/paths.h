#pragma once
// =====================================================================
//  Null Island Space Program  --  where the app's files live
// ---------------------------------------------------------------------
//  /sd/nisp, not the app's install directory: the launcher owns that
//  and may empty it when the app updates. Everything a player would
//  want to back up -- saves and settings -- is in one directory, so
//  copying it is the whole backup.
//
//  SCREENSHOT_DIR (the test kit's, set in CMakeLists.txt) is
//  /sd/nisp/test, under the same roof.
// =====================================================================

#define NISP_DATA_DIR "/sd/nisp"
#define NISP_SAVE_DIR NISP_DATA_DIR "/saves"
#define NISP_SETTINGS NISP_DATA_DIR "/settings.txt"
